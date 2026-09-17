import argparse
import os
import random
import tempfile
import subprocess
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from collections import deque
import pybullet as p

try:
    from PIL import Image
    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False

# ------------------------------------------------------------
# Configurazione principale
# ------------------------------------------------------------
NUM_DRONES = 12

AREA_WIDTH = 20.0
AREA_HEIGHT = 12.0
DT = 0.05

COMMUNICATION_RADIUS = 4.5
TARGET_SEPARATION = 2.8

# distance under which a drone is considered to have reached its target
TARGET_REACHED_DISTANCE = 0.3

MAX_TARGET_SPEED = 1.2
MAX_FORCE = 12.0

# Parametri di controllo dello sciame (tutti i valori sono empirici)
# Aging / damping del target: più alto -> il target "invecchia" più lentamente, quindi si muove più lentamente verso il target originario.
ALPHA = 0.82 # damping of target velocity (aging of the target)
BETA = 0.7 # scaling of total force applied to target (m/s^2 per Newton of total force)

MAX_DRONE_SPEED = 2.0

K_TARGET = 1.5 # scaling of desired velocity towards target (m/s per meter of error)
K_REPULSION = 3.0 # scaling of repulsion between targets of neighboring drones (N per meter of error)
K_ANCHOR = 0.9 # scaling of anchor force towards original target (N per meter of error)
K_FIRE = 1.4 # scaling of attraction towards nearest known fire (N per meter of error)
K_BOUNDARY = 1.8 # scaling of repulsion from map boundaries (N per meter of error)

FIRE_DETECTION_RADIUS = 2.7 # radius of the onboard fire sensor (meters)
BOUNDARY_MARGIN = 1.5 # margin within which boundary repulsion is active

RANDOM_SEED = 42

SHOW_FORCE_VECTORS = False # whether to show force vectors around each target (debug visualization)
SHOW_COMMUNICATION = True # whether to show lines between communicating drones (debug visualization)
SHOW_TRAILS = False # whether to show trails of drone movement (debug visualization)
TRAIL_MAX_SEGMENTS = 60
TRAIL_MIN_MOVE = 0.12 # minimum distance moved before adding a new trail segment (meters)

# Termination / monitoring
MAX_STEPS = 20000
LOG_INTERVAL = 100
# Stability check: if the average target movement over `STABILITY_WINDOW` steps
# falls below `STABILITY_THRESHOLD`, stop early.
STABILITY_WINDOW = 60
STABILITY_THRESHOLD = 1e-3

# Collision avoidance
# Minimum allowed distance between drone centers
DRONE_MIN_SEPARATION = 0.28 # meters

# Camera zoom factor (fraction of max dimension). Smaller -> more zoomed in
CAMERA_DISTANCE_FACTOR = 0.65

# Collision avoidance (decentralized predictive)
AVOID_LOOKAHEAD = 1.2  # seconds to look ahead when predicting collisions
AVOID_MIN_DISTANCE = 0.5  # desired minimum separation (meters)
AVOID_FORCE = 1.8  # scaling of avoidance steering

# Fire extinguishing basics
FIRE_HEALTH = 100.0
DRONE_WATER_CAPACITY = 50.0
DRONE_WATER_FLOW_RATE = 8.0  # units of water per second applied to fire
FIRE_EXTINGUISH_RADIUS = 1.2

# Stazione idrica / rifornimento
WATER_STATION_POS = np.array([AREA_WIDTH * 0.5, AREA_HEIGHT * 0.08], dtype=float)
LOW_WATER_THRESHOLD = DRONE_WATER_CAPACITY * 0.2   # sotto questa soglia il drone rientra a rifornirsi
DRONE_WATER_REFILL_RATE = 14.0                     # unità/sec mentre è parcheggiato alla stazione
WATER_TEXTURE_BUCKETS = 20                         # granularità del gauge (5% per bucket)

# Time constant for first-order velocity controller (seconds)
TAU = 0.3
# PID controller parameters for velocity control (per-axis)
PID_KP = 6.0
PID_KI = 1.2
PID_KD = 0.5
PID_INTEGRAL_LIMIT = 5.0
# limit for acceleration (m/s^2) applied by PID output
MAX_ACCEL = 6.0


# ------------------------------------------------------------
# Equazioni principali
# ------------------------------------------------------------
# Repulsione tra target dei vicini:
#   d = ||target_i - target_j||
#   se d >= TARGET_SEPARATION -> nessuna repulsione
#   altrimenti:
#       F_repulsion_ij = K_REPULSION * (TARGET_SEPARATION - d) * normalize(target_i - target_j)
#
# Damping / aging del target:
#   target_velocity = ALPHA * target_velocity + BETA * F_total
#   target = target + target_velocity * dt
#
# Anchor verso il target originario:
#   F_anchor = K_ANCHOR * (original_target - target)
#
# Attrazione verso gli incendi:
#   F_fire = K_FIRE * normalize(fire_position - target)
#   se ci sono più incendi, si usa il più vicino conosciuto
#
# Forza di bordo:
#   F_boundary cresce avvicinandosi ai bordi dell'area con una dipendenza lineare
#   entro BOUNDARY_MARGIN
#
# Movimento del drone:
#   desired_velocity = K_TARGET * (target - position)
#   velocity = clamp(desired_velocity, MAX_DRONE_SPEED)
#
# Per stabilità numerica, tutte le velocità e le forze vengono limitate con clamp
# e con la guardia contro distanze praticamente nulle.

# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------
def clamp_magnitude(vector: np.ndarray, limit: float) -> np.ndarray:
    magnitude = np.linalg.norm(vector)
    if magnitude <= 1e-12:
        return vector.copy()
    if magnitude > limit:
        return vector * (limit / magnitude)
    return vector.copy()


def normalize(vector: np.ndarray) -> np.ndarray:
    magnitude = np.linalg.norm(vector)
    if magnitude <= 1e-12:
        return np.zeros_like(vector)
    return vector / magnitude


def vec_to_tuple(v: np.ndarray) -> Tuple[float, float]:
    return (float(v[0]), float(v[1]))


def fire_health_to_color(health: float, max_health: float = FIRE_HEALTH) -> List[float]:
    """Colore/alpha del corpo dell'incendio in base alla salute residua:
    brace viva e opaca a piena salute, cenere grigia e trasparente vicino a zero.
    Un solo changeVisualShape per fuoco, nessun oggetto di debug aggiuntivo."""
    ratio = float(np.clip(health / max(max_health, 1e-9), 0.0, 1.0))
    vivid = np.array([1.0, 0.25, 0.05])
    embers = np.array([0.45, 0.42, 0.40])
    color = embers + (vivid - embers) * ratio
    alpha = 0.18 + 0.45 * ratio
    return [float(color[0]), float(color[1]), float(color[2]), float(alpha)]


def _build_water_bowl_texture(ratio: float, width: int = 32, height: int = 64) -> "Image.Image":
    """Texture equirettangolare per uno sphere: bande orizzontali che simulano
    un livello d'acqua che sale/scende dentro una bolla, con un piccolo
    'menisco' più chiaro sul pelo dell'acqua."""
    ratio = float(np.clip(ratio, 0.0, 1.0))
    fill_row = int(round((1.0 - ratio) * (height - 1)))
    glass = np.array([214, 232, 240])
    deep = np.array([10, 40, 120])
    mid = np.array([25, 110, 200])
    shallow = np.array([120, 200, 235])
    critical = np.array([200, 60, 40])
    meniscus = np.array([235, 245, 250])

    img = Image.new("RGB", (width, height))
    px = img.load()
    for y in range(height):
        if y < fill_row:
            color = glass
        elif y == fill_row:
            color = meniscus
        else:
            t = (y - fill_row) / max(1, (height - 1 - fill_row))
            if ratio < 0.15:
                base = critical
            elif t < 0.5:
                base = deep + (mid - deep) * (t / 0.5)
            else:
                base = mid + (shallow - mid) * ((t - 0.5) / 0.5)
            color = base
        rgb = tuple(int(c) for c in np.clip(color, 0, 255))
        for x in range(width):
            px[x, y] = rgb
    return img


# ------------------------------------------------------------
# SimulationWorld
# ------------------------------------------------------------
class SimulationWorld:
    """Interfaccia ristretta tra i Drone e il resto della simulazione.

    Un vero sciame non ha memoria globale condivisa: ogni unità conosce solo
    l'estensione della mappa (rilevata/impostata a priori) e ciò che la sua
    radio riceve dalle unità vicine. Questa classe rappresenta esattamente
    quel confine:

    - area_width / area_height: parametri della mappa (informazione statica).
    - get_neighbors(drone): simula la radio/il GPS -> "chi è nel raggio di
      comunicazione in questo istante". Nessun drone potrebbe saperlo da
      solo senza un canale di comunicazione, quindi è legittimo chiederlo
      al gestore centrale.
    - sense_fires(position): simula il sensore antincendio di bordo,
      interrogando lo stato della mappa in un raggio locale.
    - extinguish_with(...): l'incendio è l'unica risorsa realmente
      condivisa (la sua "salute" non appartiene a nessun singolo drone),
      quindi la sua mutazione passa da qui.

    Una volta ottenuta la lista dei vicini, lo scambio di informazioni tra
    droni (known_fires, posizione, velocità) avviene direttamente tra le
    istanze di Drone, senza ulteriore mediazione di questa classe.
    """

    def __init__(self, drones: List['Drone'], fires: List[dict], area_width: float, area_height: float):
        self._drones = drones
        self._fires = fires
        self.area_width = area_width
        self.area_height = area_height
        # Vicinato dell'ultimo refresh_neighbors(), tenuto per drone.idx.
        # None finché non viene calcolato per lo step corrente.
        self._neighbor_cache: Optional[Dict[int, List['Drone']]] = None

    def refresh_neighbors(self) -> Dict[int, List['Drone']]:
        """Calcola il vicinato di TUTTI i droni in un solo passaggio,
        sfruttando la simmetria della distanza: se A è entro il raggio di
        comunicazione di B, lo è anche B rispetto ad A. Ogni coppia (i, j)
        viene quindi valutata una sola volta e il risultato registrato per
        entrambi, invece di ricalcolare la stessa distanza una volta per
        drone (n chiamate O(n) = O(n^2) con distanze duplicate) come
        accade chiamando get_neighbors singolarmente per ognuno.
        Va richiamato una volta a inizio step, prima che i droni comunichino;
        get_neighbors() userà poi questo risultato senza ricalcolare nulla."""
        drones = self._drones
        neighbor_map: Dict[int, List['Drone']] = {d.idx: [] for d in drones}
        n = len(drones)
        for i in range(n):
            a = drones[i]
            for j in range(i + 1, n):
                b = drones[j]
                if np.linalg.norm(a.position - b.position) <= COMMUNICATION_RADIUS:
                    neighbor_map[a.idx].append(b)
                    neighbor_map[b.idx].append(a)
        self._neighbor_cache: Dict[int, List['Drone']] = neighbor_map
        return neighbor_map

    def get_neighbors(self, drone: 'Drone') -> List['Drone']:
        """Vicini di `drone` per lo step corrente (radio simulata).
        Se refresh_neighbors() non è ancora stato chiamato in questo step,
        lo fa ora (per tutti i droni in un colpo solo) come rete di sicurezza."""
        if self._neighbor_cache is None:
            self.refresh_neighbors()
        return self._neighbor_cache.get(drone.idx, [])

    def sense_fires(self, position: np.ndarray) -> Set[Tuple[float, float]]:
        detected = set()
        for fire in self._fires:
            if fire["health"] <= 0.0:
                continue
            if np.linalg.norm(position - fire["pos"]) <= FIRE_DETECTION_RADIUS:
                detected.add(vec_to_tuple(fire["pos"]))
        return detected

    def extinguish_with(self, position: np.ndarray, water_available: float, flow_rate: float, dt: float) -> float:
        """Applica acqua a tutti gli incendi raggiungibili dalla posizione data.
        Ritorna l'acqua effettivamente usata, mai più di `water_available`.
        """
        total_used = 0.0
        for fire in self._fires:
            if fire["health"] <= 0.0:
                continue
            if np.linalg.norm(position - fire["pos"]) > FIRE_EXTINGUISH_RADIUS:
                continue
            remaining_water = water_available - total_used
            if remaining_water <= 0.0:
                break
            applied = min(remaining_water, flow_rate * dt)
            fire["health"] -= applied
            total_used += applied
        return total_used


# ------------------------------------------------------------
# Drone: logica interamente decentralizzata
# ------------------------------------------------------------
class Drone:
    def __init__(self, idx: int, rng: random.Random, world: SimulationWorld):
        self.idx = idx
        self.world = world
        self.position = np.array(
            [rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float
        )
        self.velocity = np.zeros(2, dtype=float)
        self.original_target = np.array(
            [rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float
        )
        self.target = self.original_target.copy()
        self.target_velocity = np.zeros(2, dtype=float)
        self.known_fires: Set[Tuple[float, float]] = set()

        self.target_reached = False
        # debug viz ids and trail
        self.force_line_ids: Dict[str, int] = {}
        self.trail_ids: deque = deque(maxlen=TRAIL_MAX_SEGMENTS)
        self.last_trail_pos = self.position.copy()
        # lightweight history for trail rendering (world positions)
        self.history: deque = deque(maxlen=TRAIL_MAX_SEGMENTS + 1)
        self.history.append(self.position.copy())
        # communication line ids reservoir (keyed by partner index when partner>self.idx)
        self.comm_line_ids: Dict[int, int] = {}
        # RNG reference (stored to allow Drone to pick random targets without sim)
        self.rng = rng
        # desired velocity command (set by decision layer, tracked separately from actual velocity)
        self.desired_velocity = np.zeros(2, dtype=float)
        # PID controller state
        self.pid_integral = np.zeros(2, dtype=float)
        self.pid_last_error = np.zeros(2, dtype=float)
        # water payload for fire extinguishing
        self.water = DRONE_WATER_CAPACITY
        # internal state: whether the drone is currently extinguishing an active fire
        self.extinguishing_state = False
        self.ring_marker_ids: List[int] = []
        self._last_ring_pos: Optional[np.ndarray] = None
        self._last_ring_state: bool = False
        # stato interno: True mentre il drone è diretto/parcheggiato alla stazione idrica
        self.reloading = False

    # ---------------- percezione & comunicazione ----------------
    def sense_environment(self) -> None:
        """Fase 1: il sensore di bordo del drone rileva gli incendi nel proprio
        raggio locale. Unica chiamata al mondo esterno per informazioni sulla mappa."""
        self.known_fires = self.world.sense_fires(self.position)

    def communicate(self, neighbors: List['Drone']) -> None:
        """Fase 2: scambio diretto (drone-to-drone) della conoscenza accumulata.
        Nessuna mediazione della simulazione centrale: ogni drone legge
        direttamente lo stato pubblico dei vicini che la radio gli ha indicato."""
        for other in neighbors:
            self.known_fires |= other.known_fires # bitwise ?

    # ---------------- forze decisionali (decentralizzate) ----------------
    def compute_repulsion(self, neighbors: List['Drone']) -> np.ndarray:
        """Calcola la forza di repulsione tra target dei vicini."""
        F_repulsion = np.zeros(2, dtype=float)
        for other in neighbors:
            delta = self.target - other.target
            dist = np.linalg.norm(delta)
            if dist >= TARGET_SEPARATION:
                continue
            if dist < 1e-6: # se troppo vicini: usa la posizione reale per calcolare la direzione di repulsione
                pos_delta = self.position - other.position
                pos_dist = np.linalg.norm(pos_delta)
                if pos_dist > 1e-6:
                    direction = pos_delta / pos_dist
                else:
                    angle = (self.idx + 1) * 2.399963 # numero irrazionale per evitare allineamenti
                    direction = np.array([np.cos(angle), np.sin(angle)], dtype=float)
                F_repulsion += K_REPULSION * TARGET_SEPARATION * direction
            else:
                direction = delta / dist
                F_repulsion += K_REPULSION * (TARGET_SEPARATION - dist) * direction
        return clamp_magnitude(F_repulsion, MAX_FORCE)

    def compute_boundary_force(self) -> np.ndarray:
        force = np.zeros(2, dtype=float)
        margin = BOUNDARY_MARGIN
        area_width = self.world.area_width
        area_height = self.world.area_height
        if self.target[0] < margin:
            force[0] += K_BOUNDARY * (margin - self.target[0]) / max(margin, 1e-6)
        if self.target[0] > area_width - margin:
            force[0] -= K_BOUNDARY * (self.target[0] - (area_width - margin)) / max(margin, 1e-6)
        if self.target[1] < margin:
            force[1] += K_BOUNDARY * (margin - self.target[1]) / max(margin, 1e-6)
        if self.target[1] > area_height - margin:
            force[1] -= K_BOUNDARY * (self.target[1] - (area_height - margin)) / max(margin, 1e-6)
        return force

    def compute_fire_force(self) -> np.ndarray:
        if not self.known_fires:
            return np.zeros(2, dtype=float)
        nearest_fire = min(
            self.known_fires,
            key=lambda fire: np.linalg.norm(np.array(fire, dtype=float) - self.target),
        )
        nearest_vector = np.array(nearest_fire, dtype=float) - self.target
        if np.linalg.norm(nearest_vector) < 1e-9:
            return np.zeros(2, dtype=float)
        return K_FIRE * normalize(nearest_vector)

    def compute_total_force(self, neighbors: List['Drone']) -> np.ndarray:
        F_repulsion = self.compute_repulsion(neighbors)
        F_anchor = K_ANCHOR * (self.original_target - self.target)
        F_fire = np.zeros(2, dtype=float) if self.reloading else self.compute_fire_force()
        F_boundary = self.compute_boundary_force()
        total = F_repulsion + F_anchor + F_fire + F_boundary
        return clamp_magnitude(total, MAX_FORCE)

    # ---------------- decisione + movimento ----------------
    def decide_and_move(self, neighbors: List['Drone'], dt: float) -> None:
        """Fase 3: usando solo il proprio stato e quello (pubblico) dei vicini
        ottenuti via radio, il drone sposta il proprio target e si muove verso di esso.
        Nessuna informazione arriva dalla simulazione centrale in questa fase,
        a parte i limiti geometrici della mappa (self.world.area_*)."""
        # The internal extinguishing state is derived by the drone itself, so we do
        # not reset it externally here. The drone state is re-computed by the
        # world query in is_extinguishing_fire() and by try_extinguish().
        area_width = self.world.area_width
        area_height = self.world.area_height

        # se l'acqua è sotto soglia e non sta già estinguendo un fuoco, devia
        # verso la stazione idrica: riusa interamente target/anchor/repulsione/avoidance
        if not self.reloading and not self.is_extinguishing_fire() and self.water <= LOW_WATER_THRESHOLD:
            self.reloading = True
            self.original_target = WATER_STATION_POS.copy()
            self.target = WATER_STATION_POS.copy()
            self.target_velocity[:] = 0.0
            self.target_reached = False

        F_total = self.compute_total_force(neighbors)
        self.target_velocity = ALPHA * self.target_velocity + BETA * F_total
        self.target_velocity = clamp_magnitude(self.target_velocity, MAX_TARGET_SPEED)
        self.target += self.target_velocity * dt
        self.target[0] = np.clip(self.target[0], 0.0, area_width)
        self.target[1] = np.clip(self.target[1], 0.0, area_height)

        # 1) base desired velocity towards target
        desired_velocity = K_TARGET * (self.target - self.position)

        # 2) Decentralized predictive avoidance: compute steering correction
        for other in neighbors:
            r = self.position - other.position
            v_rel = self.velocity - other.velocity
            v_rel_norm2 = np.dot(v_rel, v_rel)
            if v_rel_norm2 < 1e-8:
                continue
            t_ca = -np.dot(r, v_rel) / v_rel_norm2
            if t_ca < 0 or t_ca > AVOID_LOOKAHEAD:
                continue
            d_ca = np.linalg.norm(r + v_rel * t_ca)
            if d_ca < AVOID_MIN_DISTANCE:
                perp = np.array([-v_rel[1], v_rel[0]])
                if np.linalg.norm(perp) < 1e-6:
                    perp = normalize(r)
                else:
                    perp = normalize(perp)
                strength = AVOID_FORCE * (1.0 - (d_ca / AVOID_MIN_DISTANCE))
                # apply steering to the desired velocity (command), not to the actual velocity
                desired_velocity += perp * strength

        # 3) saturate desired velocity and store as command
        self.desired_velocity = clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)

        # 4) PID controller: compute acceleration command to track desired_velocity
        error = self.desired_velocity - self.velocity
        deriv = (error - self.pid_last_error) / max(dt, 1e-12)
        self.pid_integral += error * dt
        # anti-windup: clamp integral per-component
        self.pid_integral = np.clip(self.pid_integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)
        pid_output = PID_KP * error + PID_KI * self.pid_integral + PID_KD * deriv
        # clamp acceleration magnitude
        pid_output = clamp_magnitude(pid_output, MAX_ACCEL)
        # apply acceleration to update velocity
        self.velocity += pid_output * dt
        # clamp speed
        self.velocity = clamp_magnitude(self.velocity, MAX_DRONE_SPEED)
        # integrate position
        self.position += self.velocity * dt
        self.position[0] = np.clip(self.position[0], 0.0, area_width)
        self.position[1] = np.clip(self.position[1], 0.0, area_height)
        # store last error for derivative term
        self.pid_last_error = error.copy()

        # if drone reached its current target, assign a new random target only when
        # it is not currently standing on an active fire. If it is actively
        # extinguishing a fire, it should stay there until the fire is cleared.
        if np.linalg.norm(self.position - self.target) < TARGET_REACHED_DISTANCE:
            self.target_reached = True
            if self.is_extinguishing_fire():
                # finalize the fire attack: remain in place and keep extinguishing
                # until the active fire disappears.
                pass
            elif self.reloading:
                # parcheggiato alla stazione: ci pensa try_reload() a rifornire e a liberarlo
                pass
            else:
                # otherwise, pick a new random target within the area bounds.
                new_t = np.array(
                    [self.rng.uniform(0.0, area_width), self.rng.uniform(0.0, area_height)], dtype=float
                )
                self.original_target = new_t.copy()
                self.target = new_t.copy()
                self.target_velocity[:] = 0.0
                self.target_reached = False   # <-- nuovo target: torna in volo
        else:
            self.target_reached = False

        # update lightweight history for trails (used by screenshot renderer)
        moved = np.linalg.norm(self.position - self.last_trail_pos)
        if moved > TRAIL_MIN_MOVE:
            self.history.append(self.position.copy())
            self.last_trail_pos = self.position.copy()

    # ---------------- azione sul mondo condiviso ----------------
    def is_extinguishing_fire(self) -> bool:
        """True quando il drone si trova entro il raggio di estinzione di un
        incendio ancora attivo, ha raggiunto il suo target e ha ancora acqua disponibile."""
        if not self.target_reached:
            return False
        if self.water <= 0.0:
            return False
        for fire in self.world._fires:
            if fire["health"] <= 0.0:
                continue
            if np.linalg.norm(self.position - fire["pos"]) <= FIRE_EXTINGUISH_RADIUS:
                return True
        return False

    def try_extinguish(self, dt: float) -> None:
        """Fase 4: il drone agisce sulla risorsa condivisa (salute dell'incendio)
        tramite l'unica interfaccia che la possiede: SimulationWorld.

        L'unico stato attivo è mantenuto internamente dal drone, in base alla sua
        azione effettiva sul mondo, non da una variabile manipolata dal simulatore."""
        self.extinguishing_state = False
        if not self.target_reached:
            return
        if self.water <= 0.0:
            return
        used = self.world.extinguish_with(self.position, self.water, DRONE_WATER_FLOW_RATE, dt)
        self.water -= used
        self.extinguishing_state = used > 1e-9

    def try_reload(self, dt: float) -> None:
        """Fase 5b: se in rientro alla stazione, gestisce il rifornimento una volta
        raggiunta. Stato interamente interno al drone, nessuna scrittura esterna."""
        if not self.reloading:
            return
        if not self.target_reached:
            return  # ancora in volo verso la stazione
        self.water = min(DRONE_WATER_CAPACITY, self.water + DRONE_WATER_REFILL_RATE * dt)
        if self.water >= DRONE_WATER_CAPACITY - 1e-6:
            self.reloading = False
            new_t = np.array(
                [self.rng.uniform(0.0, self.world.area_width), self.rng.uniform(0.0, self.world.area_height)],
                dtype=float,
            )
            self.original_target = new_t.copy()
            self.target = new_t.copy()
            self.target_velocity[:] = 0.0
            self.target_reached = False

    # ---------------- debug / rendering ----------------
    def update_extinguish_marker(self) -> None:
        """Aggiorna l'anello di debug solo quando serve.

        Il collo di bottiglia è il redraw completo del ring a ogni frame:
        per PyBullet, anche un piccolo numero di debug line è costoso. Qui
        costruiamo il ring una sola volta, lo aggiorniamo solo in caso di
        variazione di stato o di movimento rilevante del drone, e lo rimuoviamo
        quando non è più necessario.
        """
        radius, segments, z = 0.16, 12, 0.22
        active = bool(self.extinguishing_state)

        if not active:
            if self.ring_marker_ids:
                for line_id in self.ring_marker_ids:
                    try:
                        if line_id is not None and line_id >= 0:
                            p.removeUserDebugItem(line_id)
                    except Exception:
                        pass
                self.ring_marker_ids = []
            self._last_ring_pos = None
            self._last_ring_state = False
            return

        center = np.array(self.position, dtype=float)
        if self._last_ring_state != active or self._last_ring_pos is None:
            self._last_ring_pos = center.copy()
            self._last_ring_state = True
        elif np.linalg.norm(center - self._last_ring_pos) < 1e-3:
            return

        if not self.ring_marker_ids:
            for k in range(segments):
                a0, a1 = 2.0 * np.pi * k / segments, 2.0 * np.pi * (k + 1) / segments
                p0 = [center[0] + radius * np.cos(a0), center[1] + radius * np.sin(a0), z]
                p1 = [center[0] + radius * np.cos(a1), center[1] + radius * np.sin(a1), z]
                try:
                    line_id = p.addUserDebugLine(p0, p1, [1.0, 0.9, 0.1], 1.0)
                except Exception:
                    line_id = -1
                self.ring_marker_ids.append(line_id)
        else:
            for k, line_id in enumerate(self.ring_marker_ids):
                if line_id is None or line_id < 0:
                    continue
                a0, a1 = 2.0 * np.pi * k / segments, 2.0 * np.pi * (k + 1) / segments
                p0 = [center[0] + radius * np.cos(a0), center[1] + radius * np.sin(a0), z]
                p1 = [center[0] + radius * np.cos(a1), center[1] + radius * np.sin(a1), z]
                try:
                    p.addUserDebugLine(p0, p1, [1.0, 0.9, 0.1], 1.0, replaceItemUniqueId=line_id)
                except Exception:
                    pass

        self._last_ring_pos = center.copy()
        self._last_ring_state = True


class SwarmSimulation:
    """Gestore centrale: stato fisico condiviso (posizioni globali, incendi),
    vincolo fisico di non-compenetrazione tra corpi, rendering e I/O.
    Non contiene alcuna logica decisionale dello sciame: quella vive
    interamente in Drone / SimulationWorld."""

    def __init__(self, headless: bool = False, show_force_vectors: bool = False, show_communication: bool = True):
        self.headless = headless
        self.show_force_vectors = show_force_vectors
        self.show_communication = show_communication

        self.rng = random.Random(RANDOM_SEED)
        # fires: list of dicts with 'pos' (np.array) and 'health'
        self.fires = [
            {"pos": np.array([AREA_WIDTH * 0.25, AREA_HEIGHT * 0.75], dtype=float), "health": FIRE_HEALTH, "detection_ids": []},
            {"pos": np.array([AREA_WIDTH * 0.68, AREA_HEIGHT * 0.32], dtype=float), "health": FIRE_HEALTH, "detection_ids": []},
            {"pos": np.array([AREA_WIDTH * 0.5, AREA_HEIGHT * 0.55], dtype=float), "health": FIRE_HEALTH, "detection_ids": []},
        ]

        # self.drones è popolata dopo la creazione di world: la lista è condivisa
        # per riferimento, quindi world "vede" i droni non appena vengono aggiunti.
        self.drones: List[Drone] = []
        self.world = SimulationWorld(self.drones, self.fires, AREA_WIDTH, AREA_HEIGHT)
        self.drones.extend(Drone(i, self.rng, self.world) for i in range(NUM_DRONES))

        self.physics_client = None
        self._setup_pybullet()

        self.drone_body_ids = []
        self.original_target_body_ids = []
        self.target_body_ids = []
        self.fire_body_ids = []
        # persistent debug ids for fire detection circles: list of lists (one per fire)
        self.fire_detection_ids: List[List[int]] = []
        # persistent map border ids (user debug lines)
        self.map_border_ids: List[int] = []
        # stazione idrica: body + cache texture "livello acqua" per bucket, tenuta per drone
        self.water_station_body_id: Optional[int] = None
        self._water_texture_cache: Dict[int, int] = {}
        self._drone_water_bucket: Dict[int, int] = {}
        self._texture_tmp_dir = tempfile.mkdtemp(prefix="swarm_water_textures_")
        self._create_visual_objects()
        self.total_collisions = 0
        self.step_collisions = 0
        # dynamic debug ids to remove each frame (lines, texts)
        self.dynamic_debug_ids: List[int] = []
        self.collision_text_id: int = -1
        # neighbor lists computed during step(), reused by draw_scene() to
        # avoid recomputing O(n^2) neighbor queries multiple times per frame
        self._last_neighbors: Dict[int, List[Drone]] = {}

    def _setup_pybullet(self):
        if self.headless:
            self.physics_client = p.connect(p.DIRECT)
        else:
            self.physics_client = p.connect(p.GUI)

        p.resetSimulation()
        p.setGravity(0, 0, 0)
        p.setTimeStep(DT)
        p.setRealTimeSimulation(0)
        # disable PyBullet default GUI panels (tabs) to avoid unused UI and grid overlay
        # this hides the default tabs while keeping the 3D viewport
        try:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        except Exception:
            # older/newer pybullet variations: ignore if unsupported
            pass
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=max(AREA_WIDTH, AREA_HEIGHT) * 0.4,
            cameraYaw=0,
            cameraPitch=-89,
            cameraTargetPosition=[AREA_WIDTH / 2.0, AREA_HEIGHT / 2.0, 0.0],
        )

    def _create_visual_objects(self):
        # Batch creation: disabling rendering while adding many bodies/debug lines
        # avoids an incremental redraw per call and is noticeably faster on init.
        try:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        except Exception:
            pass

        for drone in self.drones:
            drone_visual = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.15,
                rgbaColor=[1.0, 1.0, 1.0, 1.0],  # bianco: il colore vero arriva dalla texture "bolla d'acqua"
            )
            drone_body = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=drone_visual,
                basePosition=[drone.position[0], drone.position[1], 0.1],
            )
            self.drone_body_ids.append(drone_body)

            orig_visual = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.08,
                rgbaColor=[0.2, 1.0, 0.4, 1.0],
            )
            orig_body = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=orig_visual,
                basePosition=[drone.original_target[0], drone.original_target[1], 0.08],
            )
            self.original_target_body_ids.append(orig_body)

            target_visual = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=0.09,
                rgbaColor=[1.0, 0.2, 0.2, 1.0],
            )
            target_body = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=target_visual,
                basePosition=[drone.target[0], drone.target[1], 0.09],
            )
            self.target_body_ids.append(target_body)

        for fire in self.fires:
            fire_visual = p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=0.18,
                length=0.04,
                rgbaColor=[1.0, 0.0, 0.0, 0.35],
            )
            fire_body = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=fire_visual,
                basePosition=[fire["pos"][0], fire["pos"][1], 0.02],
            )
            self.fire_body_ids.append(fire_body)
            # draw detection radius circle once and keep the debug ids.
            # Fires never move once created, so this circle never needs to be
            # recreated later (see draw_scene: no per-frame maintenance needed).
            det_ids = self._create_fire_detection(fire["pos"])
            self.fire_detection_ids.append(det_ids)
            fire["detection_ids"] = det_ids

        # create persistent map border lines (one-time)
        try:
            self.map_border_ids = []
            self.map_border_ids.append(p.addUserDebugLine([0, 0, 0], [AREA_WIDTH, 0, 0], [0.3, 0.3, 0.3], 1.0))
            self.map_border_ids.append(p.addUserDebugLine([AREA_WIDTH, 0, 0], [AREA_WIDTH, AREA_HEIGHT, 0], [0.3, 0.3, 0.3], 1.0))
            self.map_border_ids.append(p.addUserDebugLine([AREA_WIDTH, AREA_HEIGHT, 0], [0, AREA_HEIGHT, 0], [0.3, 0.3, 0.3], 1.0))
            self.map_border_ids.append(p.addUserDebugLine([0, AREA_HEIGHT, 0], [0, 0, 0], [0.3, 0.3, 0.3], 1.0))
        except Exception:
            pass

        # stazione idrica: un solo cilindro + etichetta, nessuna manutenzione per-frame
        station_visual = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.4,
            length=0.05,
            rgbaColor=[0.15, 0.55, 0.9, 0.85],
        )
        self.water_station_body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=station_visual,
            basePosition=[WATER_STATION_POS[0], WATER_STATION_POS[1], 0.03],
        )
        try:
            p.addUserDebugText(
                "WATER", [WATER_STATION_POS[0], WATER_STATION_POS[1] + 0.5, 0.2],
                textColorRGB=[0.6, 0.85, 1.0], textSize=1.1,
            )
        except Exception:
            pass

        try:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        except Exception:
            pass

    def close(self):
        if self.physics_client is not None:
            p.disconnect(self.physics_client)

    def step(self):
        """Orchestrazione minimale: la simulazione chiama, nell'ordine, le fasi
        decentralizzate di ogni Drone. Non calcola nessuna forza e non decide
        nulla per conto dei droni."""
        # 1) percezione locale (ogni drone interroga la mappa per il proprio sensore)
        for drone in self.drones:
            drone.sense_environment()

        # 2) comunicazione: un solo passaggio simmetrico calcola il vicinato
        #    di TUTTI i droni (se A è vicino di B, B è vicino di A "gratis",
        #    senza dover richiedere/ricalcolare la stessa distanza una
        #    seconda volta), poi ogni drone si scambia direttamente le
        #    informazioni con i propri vicini. Il risultato viene tenuto
        #    anche per il resto dello step (decisione) e per draw_scene().
        self._last_neighbors = self.world.refresh_neighbors()
        for drone in self.drones:
            drone.communicate(self._last_neighbors[drone.idx])

        # 3) decisione + movimento (interamente dentro Drone)
        for drone in self.drones:
            drone.decide_and_move(self._last_neighbors[drone.idx], DT)

        # 4) vincolo fisico centrale: impedisce la compenetrazione tra corpi.
        #    Non è una decisione dello sciame ma un risolutore di vincoli
        #    geometrici globali (nessun singolo drone potrebbe garantirla da
        #    solo in modo consistente), quindi resta qui.
        self.step_collisions = 0
        self._resolve_collisions()

        # 5) azione dei droni sulla risorsa condivisa (incendi)
        for drone in self.drones:
            drone.try_extinguish(DT)
        self._remove_extinguished_fires()

        # 6) rifornimento idrico (stato interno al drone, target-based, nessuna
        #    logica di evitamento aggiuntiva: riusa repulsione/avoidance esistenti)
        for drone in self.drones:
            drone.try_reload(DT)

    def _remove_extinguished_fires(self):
        remove_indices = [fi for fi, fire in enumerate(self.fires) if fire["health"] <= 0.0]
        for idx in sorted(remove_indices, reverse=True):
            try:
                body_id = self.fire_body_ids.pop(idx)
                p.removeBody(body_id)
            except Exception:
                pass
            try:
                det = self.fire_detection_ids.pop(idx)
                for uid in det:
                    try:
                        p.removeUserDebugItem(uid)
                    except Exception:
                        pass
            except Exception:
                pass
            try:
                fire = self.fires.pop(idx)
                for uid in fire.get("detection_ids", []):
                    try:
                        p.removeUserDebugItem(uid)
                    except Exception:
                        pass
            except Exception:
                pass

    def draw_scene(self):
        # remove previous dynamic debug items (lines/texts) to avoid global clear
        for uid in list(self.dynamic_debug_ids):
            try:
                if uid is None or int(uid) < 0:
                    continue
                p.removeUserDebugItem(uid)
            except Exception:
                pass
        self.dynamic_debug_ids.clear()

        # Area di simulazione (rectangle): bordo persistente creato una volta in
        # _create_visual_objects(). I cerchi di rilevamento incendio sono
        # anch'essi persistenti e non richiedono manutenzione per-frame: i
        # fire non si spostano mai, solo la loro salute cambia (e la loro
        # rimozione è già gestita in _remove_extinguished_fires()).

        # Aggiorna oggetti visivi degli agenti e dei target
        for idx, drone in enumerate(self.drones):
            ratio = drone.water / max(DRONE_WATER_CAPACITY, 1e-9)
            bucket = int(round(np.clip(ratio, 0.0, 1.0) * WATER_TEXTURE_BUCKETS))
            if self._drone_water_bucket.get(idx) != bucket:
                tex_id = self._get_water_texture_id(ratio)
                if tex_id is not None:
                    p.changeVisualShape(self.drone_body_ids[idx], -1, textureUniqueId=tex_id)
                self._drone_water_bucket[idx] = bucket
            p.resetBasePositionAndOrientation(
                self.drone_body_ids[idx],
                [drone.position[0], drone.position[1], 0.1],
                [0, 0, 0, 1],
            )
            p.resetBasePositionAndOrientation(
                self.original_target_body_ids[idx],
                [drone.original_target[0], drone.original_target[1], 0.08],
                [0, 0, 0, 1],
            )
            p.resetBasePositionAndOrientation(
                self.target_body_ids[idx],
                [drone.target[0], drone.target[1], 0.09],
                [0, 0, 0, 1],
            )
            drone.update_extinguish_marker()

        for fi, fire in enumerate(self.fires):
            if fi < len(self.fire_body_ids):
                p.changeVisualShape(self.fire_body_ids[fi], -1, rgbaColor=fire_health_to_color(fire["health"]))

        # Mostra connessioni tra droni vicini (riusa i vicini già calcolati in step())
        if self.show_communication:
            for drone in self.drones:
                i = drone.idx
                neighbor_idxs = {n.idx for n in self._last_neighbors.get(i, [])}
                # We only create/manage one line per pair with j > i to avoid duplicates
                wanted = {j for j in neighbor_idxs if j > i}
                # remove stale lines
                for old in list(drone.comm_line_ids.keys()):
                    if old not in wanted:
                        try:
                            uid = drone.comm_line_ids.pop(old)
                            if uid is not None and int(uid) >= 0:
                                p.removeUserDebugItem(uid)
                        except Exception:
                            pass
                # create or update desired lines
                for j in wanted:
                    prev_id = drone.comm_line_ids.get(j, -1)
                    start = [drone.position[0], drone.position[1], 0.0]
                    end = [self.drones[j].position[0], self.drones[j].position[1], 0.0]
                    try:
                        if prev_id is not None and int(prev_id) >= 0:
                            lid = p.addUserDebugLine(start, end, [0.5, 0.8, 1.0], 1.0, replaceItemUniqueId=prev_id)
                        else:
                            lid = p.addUserDebugLine(start, end, [0.5, 0.8, 1.0], 1.0)
                    except TypeError:
                        try:
                            if prev_id and prev_id != -1:
                                p.removeUserDebugItem(prev_id)
                        except Exception:
                            pass
                        lid = p.addUserDebugLine(start, end, [0.5, 0.8, 1.0], 1.0)
                    drone.comm_line_ids[j] = lid

        # Vettori delle forze (riuso linee per ridurre overhead, vicini già disponibili)
        if self.show_force_vectors:
            for drone in self.drones:
                neigh_objs = self._last_neighbors.get(drone.idx, [])
                self._draw_force_vector(
                    drone, name="repulsion", start=drone.target.copy(),
                    vec=drone.compute_repulsion(neigh_objs), color=[1.0, 0.8, 0.2], scale=0.15,
                )
                self._draw_force_vector(
                    drone, name="anchor", start=drone.target.copy(),
                    vec=K_ANCHOR * (drone.original_target - drone.target), color=[0.2, 1.0, 0.8], scale=0.2,
                )
                self._draw_force_vector(
                    drone, name="fire", start=drone.target.copy(),
                    vec=drone.compute_fire_force(), color=[1.0, 0.5, 0.0], scale=0.25,
                )
                self._draw_force_vector(
                    drone, name="boundary", start=drone.target.copy(),
                    vec=drone.compute_boundary_force(), color=[0.2, 0.2, 0.9], scale=0.22,
                )
                self._draw_force_vector(
                    drone, name="total", start=drone.target.copy(),
                    vec=drone.compute_total_force(neigh_objs), color=[1.0, 1.0, 1.0], scale=0.18,
                )

        # Trails
        if SHOW_TRAILS:
            for drone in self.drones:
                moved = np.linalg.norm(drone.position - drone.last_trail_pos)
                if moved > TRAIL_MIN_MOVE:
                    if len(drone.trail_ids) == drone.trail_ids.maxlen:
                        old = drone.trail_ids.popleft()
                        try:
                            p.removeUserDebugItem(old)
                        except Exception:
                            pass
                    line_id = p.addUserDebugLine(
                        [drone.last_trail_pos[0], drone.last_trail_pos[1], 0.02],
                        [drone.position[0], drone.position[1], 0.02],
                        [0.3, 0.6, 1.0], 1.0,
                    )
                    drone.trail_ids.append(line_id)
                    drone.last_trail_pos = drone.position.copy()

        # show collision count this frame
        if not self.headless:
            try:
                text = f"collisions_step={self.step_collisions} total={self.total_collisions}"
                if self.collision_text_id == -1:
                    tid = p.addUserDebugText(text, [0.5, -0.5, 0.2], textColorRGB=[1, 1, 0], textSize=1.2)
                    self.collision_text_id = tid
                else:
                    # remove previous and re-add (replaceItemUniqueId for text is not reliably available)
                    p.removeUserDebugItem(self.collision_text_id)
                    tid = p.addUserDebugText(text, [0.5, -0.5, 0.2], textColorRGB=[1, 1, 0], textSize=1.2)
                    self.collision_text_id = tid
                self.dynamic_debug_ids.append(self.collision_text_id)
            except Exception:
                pass

    def _draw_force_vector(self, drone: Drone, name: str, start: np.ndarray, vec: np.ndarray, color, scale: float):
        # Reuse previous line id for this drone & vector name to reduce draw overhead
        if np.linalg.norm(vec) < 1e-8:
            # if previously drawn, remove it
            prev = drone.force_line_ids.get(name, -1)
            if prev != -1:
                try:
                    p.removeUserDebugItem(prev)
                except Exception:
                    pass
                drone.force_line_ids.pop(name, None)
            return
        end = start + clamp_magnitude(vec, MAX_FORCE) * scale
        prev_id = drone.force_line_ids.get(name, -1)
        try:
            if prev_id is not None and int(prev_id) >= 0:
                # only pass replaceItemUniqueId when we have a valid previous id
                line_id = p.addUserDebugLine([start[0], start[1], 0.0], [end[0], end[1], 0.0], color, 1.0, replaceItemUniqueId=prev_id)
            else:
                line_id = p.addUserDebugLine([start[0], start[1], 0.0], [end[0], end[1], 0.0], color, 1.0)
        except TypeError:
            # older pybullet versions may not support replaceItemUniqueId as kwarg
            try:
                if prev_id and prev_id != -1:
                    p.removeUserDebugItem(prev_id)
            except Exception:
                pass
            line_id = p.addUserDebugLine([start[0], start[1], 0.0], [end[0], end[1], 0.0], color, 1.0)
        drone.force_line_ids[name] = line_id

    def _create_fire_detection(self, center: np.ndarray, radius: float = None, color=(1.0, 0.4, 0.1), z: float = 0.05, segments: int = 48) -> List[int]:
        """Create persistent debug lines for a fire detection circle and return list of ids."""
        if radius is None:
            radius = FIRE_DETECTION_RADIUS
        pts = []
        for k in range(segments + 1):
            a = 2.0 * np.pi * k / segments
            pts.append((center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)))
        ids = []
        for k in range(segments):
            try:
                lid = p.addUserDebugLine([pts[k][0], pts[k][1], z], [pts[k + 1][0], pts[k + 1][1], z], color, 1.0)
                ids.append(lid)
            except Exception:
                pass
        return ids

    def _get_water_texture_id(self, ratio: float) -> Optional[int]:
        """Texture "bolla d'acqua" per il livello del serbatoio del drone, con
        cache per bucket quantizzato: evita di rigenerare/ricaricare la texture
        a ogni frame, solo quando il livello attraversa un nuovo bucket."""
        if not _PIL_AVAILABLE:
            return None
        bucket = int(round(np.clip(ratio, 0.0, 1.0) * WATER_TEXTURE_BUCKETS))
        tex_id = self._water_texture_cache.get(bucket)
        if tex_id is None:
            img = _build_water_bowl_texture(bucket / WATER_TEXTURE_BUCKETS)
            path = os.path.join(self._texture_tmp_dir, f"water_{bucket}.png")
            img.save(path)
            tex_id = p.loadTexture(path)
            self._water_texture_cache[bucket] = tex_id
        return tex_id

    def _resolve_collisions(self):
        n = len(self.drones)
        min_sep = DRONE_MIN_SEPARATION
        for i in range(n):
            for j in range(i + 1, n):
                a = self.drones[i]
                b = self.drones[j]
                delta = a.position - b.position
                dist = np.linalg.norm(delta)
                if dist < 1e-8:
                    # practically coincident: apply tiny random jitter
                    jitter = (np.array([self.rng.uniform(-1, 1), self.rng.uniform(-1, 1)]) * 1e-3)
                    a.position += jitter
                    b.position -= jitter
                    self.step_collisions += 1
                    self.total_collisions += 1
                    continue
                if dist < min_sep:
                    # move each drone half the overlap along the separation direction
                    overlap = min_sep - dist
                    direction = delta / dist
                    correction = direction * (overlap * 0.5)
                    a.position += correction
                    b.position -= correction
                    # keep inside bounds
                    a.position[0] = np.clip(a.position[0], 0.0, AREA_WIDTH)
                    a.position[1] = np.clip(a.position[1], 0.0, AREA_HEIGHT)
                    b.position[0] = np.clip(b.position[0], 0.0, AREA_WIDTH)
                    b.position[1] = np.clip(b.position[1], 0.0, AREA_HEIGHT)
                    self.step_collisions += 1
                    self.total_collisions += 1


# ------------------------------------------------------------
# Rendering raster (usato per lo screenshot headless)
# ------------------------------------------------------------
def _draw_filled_circle(img: np.ndarray, center_px: Tuple[int, int], radius_px: int, color: Tuple[int, int, int]) -> None:
    """Disegna un cerchio pieno con una maschera numpy vettorizzata invece di
    un doppio ciclo Python pixel-per-pixel (molto più veloce su immagini grandi)."""
    height, width = img.shape[0], img.shape[1]
    cx, cy = center_px
    r0, r1 = max(0, cy - radius_px), min(height, cy + radius_px + 1)
    c0, c1 = max(0, cx - radius_px), min(width, cx + radius_px + 1)
    if r0 >= r1 or c0 >= c1:
        return
    yy, xx = np.ogrid[r0:r1, c0:c1]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius_px * radius_px
    img[r0:r1, c0:c1][mask] = np.array(color, dtype=np.uint8)


def _draw_segment(img: np.ndarray, p0: Tuple[float, float], p1: Tuple[float, float], color: Tuple[int, int, int]) -> None:
    """Traccia un segmento a spessore 1px campionando lungo la direzione dominante
    (niente doppio ciclo per lo spessore, dato che qui è sempre 1)."""
    height, width = img.shape[0], img.shape[1]
    x0, y0 = p0
    x1, y1 = p1
    steps = int(max(abs(x1 - x0), abs(y1 - y0), 1))
    t = np.linspace(0.0, 1.0, steps + 1)
    xs = np.clip(np.round(x0 + (x1 - x0) * t).astype(int), 0, width - 1)
    ys = np.clip(np.round(y0 + (y1 - y0) * t).astype(int), 0, height - 1)
    img[ys, xs] = np.array(color, dtype=np.uint8)


def render_top_down_image(sim: SwarmSimulation, width: int = 1024, height: int = 768) -> np.ndarray:
    """Renderizza un'immagine top-down della scena corrente, senza dipendere
    dalla telecamera di PyBullet (utile in modalità headless)."""
    px_per_x = float(width) / float(max(1.0, AREA_WIDTH))
    px_per_y = float(height) / float(max(1.0, AREA_HEIGHT))
    img = np.ones((height, width, 3), dtype=np.uint8) * 255

    def world_to_pixel(pt) -> Tuple[int, int]:
        x, y = float(pt[0]), float(pt[1])
        col = int(np.clip(x * px_per_x, 0, width - 1))
        row = int(np.clip(height - 1 - y * px_per_y, 0, height - 1))
        return col, row

    # incendi: cerchio di rilevamento + centro (fire["pos"], non fire direttamente)
    for fire in sim.fires:
        cx, cy = world_to_pixel(fire["pos"])
        r_px = int(max(2, FIRE_DETECTION_RADIUS * (px_per_x + px_per_y) * 0.5))
        _draw_filled_circle(img, (cx, cy), r_px, (255, 140, 40))
        _draw_filled_circle(img, (cx, cy), 4, (200, 40, 0))

    # scie (history leggera)
    trail_color = (160, 200, 255)
    for drone in sim.drones:
        hist = list(drone.history)
        for k in range(len(hist) - 1):
            p0 = world_to_pixel(hist[k])
            p1 = world_to_pixel(hist[k + 1])
            _draw_segment(img, p0, p1, trail_color)

    # link di comunicazione (usa i vicini già calcolati nell'ultimo step)
    if sim.show_communication:
        comm_color = (200, 200, 200)
        for drone in sim.drones:
            for other in sim._last_neighbors.get(drone.idx, []):
                if other.idx > drone.idx:
                    p0 = world_to_pixel(drone.position)
                    p1 = world_to_pixel(other.position)
                    _draw_segment(img, p0, p1, comm_color)

    # droni e target
    for drone in sim.drones:
        dc, dr = world_to_pixel(drone.position)
        _draw_filled_circle(img, (dc, dr), 3, (50, 120, 255))
        ogc, ogr = world_to_pixel(drone.original_target)
        _draw_filled_circle(img, (ogc, ogr), 1, (40, 200, 80))
        tc, tr = world_to_pixel(drone.target)
        _draw_filled_circle(img, (tc, tr), 2, (220, 40, 40))

    return img


def save_image(img: np.ndarray, out_path: str) -> None:
    """Salva l'immagine come PPM; se è richiesto un .png tenta la conversione
    con ImageMagick (`convert`), mantenendo il PPM se non è disponibile."""
    height, width = img.shape[0], img.shape[1]
    wants_png = out_path.lower().endswith(".png")
    ppm_path = out_path
    if wants_png:
        tmp_ppm = tempfile.NamedTemporaryFile(suffix=".ppm", delete=False)
        ppm_path = tmp_ppm.name
        tmp_ppm.close()

    with open(ppm_path, "wb") as f:
        f.write(f"P6 {width} {height} 255\n".encode("ascii"))
        f.write(img.astype("uint8").tobytes())

    if not wants_png:
        print(f"[sim] Screenshot saved to {out_path} (PPM)")
        return

    try:
        subprocess.run(["convert", ppm_path, out_path], check=True)
        print(f"[sim] Screenshot saved to {out_path} (converted from PPM)")
        try:
            os.remove(ppm_path)
        except Exception:
            pass
    except FileNotFoundError:
        print("[sim] 'convert' not found; kept PPM at", ppm_path)
    except subprocess.CalledProcessError as e:
        print(f"[sim] 'convert' failed: {e}; PPM kept at {ppm_path}")


def run_simulation(
    headless: bool,
    show_force_vectors: bool = False,
    show_communication: bool = True,
    screenshot_path: Optional[str] = None,
    screenshot_step: int = 0,
) -> None:
    sim = SwarmSimulation(headless=headless, show_force_vectors=show_force_vectors, show_communication=show_communication)
    try:
        step_count = 0
        paused = False
        # track recent average target movements for stability detection
        recent_moves = deque(maxlen=STABILITY_WINDOW)
        prev_targets = [drone.target.copy() for drone in sim.drones]

        while p.isConnected():
            # Keyboard handling (only meaningful in GUI)
            if not headless:
                keys = p.getKeyboardEvents()
                # 'q' to quit, 'p' to pause/unpause, 'v' to toggle force vectors
                if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
                    print("[sim] Quit requested (q)")
                    break
                if ord("p") in keys and keys[ord("p")] & p.KEY_WAS_TRIGGERED:
                    paused = not paused
                    print(f"[sim] Paused={paused}")
                if ord("v") in keys and keys[ord("v")] & p.KEY_WAS_TRIGGERED:
                    sim.show_force_vectors = not sim.show_force_vectors
                    print(f"[sim] show_force_vectors={sim.show_force_vectors}")

            if not paused:
                sim.step()

            sim.draw_scene()

            # Advance physics only if not headless pause semantics differ
            p.stepSimulation()

            # Monitoring / statistics
            total_move = 0.0
            for i, drone in enumerate(sim.drones):
                move = np.linalg.norm(drone.target - prev_targets[i])
                total_move += move
                prev_targets[i][:] = drone.target
            avg_move = total_move / max(1, len(sim.drones))
            recent_moves.append(avg_move)

            step_count += 1

            # Screenshot capture happens inside the single simulation run, so
            # a --screenshot request never depends on --headless having been
            # passed explicitly (the effective headless mode is whatever this
            # run is actually using).
            if screenshot_path is not None and screenshot_step > 0 and step_count == screenshot_step:
                img = render_top_down_image(sim)
                save_image(img, screenshot_path)

            if step_count % LOG_INTERVAL == 0:
                mean_recent = float(np.mean(recent_moves)) if recent_moves else 0.0
                avg_target_speed = float(np.mean([np.linalg.norm(d.target_velocity) for d in sim.drones]))
                print(f"[sim] step={step_count} avg_target_move={avg_move:.6f} mean_recent={mean_recent:.6f} avg_target_speed={avg_target_speed:.4f}")

            # Termination conditions
            if step_count >= MAX_STEPS:
                print(f"[sim] Reached MAX_STEPS={MAX_STEPS}")
                break
            if len(recent_moves) == STABILITY_WINDOW and float(np.mean(recent_moves)) < STABILITY_THRESHOLD:
                print(f"[sim] Stability detected (mean recent move {np.mean(recent_moves):.6f} < {STABILITY_THRESHOLD}) — stopping")
                break

        # if a screenshot was requested for step 0 (i.e. "at the end") or the
        # simulation ended before reaching screenshot_step, capture it now.
        if screenshot_path is not None and (screenshot_step <= 0 or step_count < screenshot_step):
            img = render_top_down_image(sim)
            save_image(img, screenshot_path)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulazione 2D di swarm decentralizzato con PyBullet")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Esegue la simulazione in modalità headless (utile in ambienti senza display)",
    )
    parser.add_argument(
        "--show-vectors",
        action="store_true",
        help="Abilita il disegno dei vettori di forza (di default off)",
    )
    parser.add_argument(
        "--no-comm",
        action="store_true",
        help="Disabilita il disegno delle connessioni di comunicazione",
    )
    parser.add_argument(
        "--screenshot",
        nargs="?",
        const="screenshot.png",
        help="Salva uno screenshot (al passo indicato da --screenshot-step, o alla fine se 0); opzionale percorso file",
    )
    parser.add_argument(
        "--screenshot-step",
        type=int,
        default=0,
        help="Se >0, salva lo screenshot al passo specificato (al termine se 0)",
    )
    args = parser.parse_args()

    display_available = os.environ.get("DISPLAY") is not None or os.environ.get("WAYLAND_DISPLAY") is not None
    effective_headless = args.headless or not display_available

    run_simulation(
        headless=effective_headless,
        show_force_vectors=args.show_vectors,
        show_communication=not args.no_comm,
        screenshot_path=args.screenshot,
        screenshot_step=args.screenshot_step,
    )