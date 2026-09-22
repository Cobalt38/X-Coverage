"""
X-Coverage — simulazione 2D di uno sciame decentralizzato di droni antincendio.

Architettura: "controllo multi-agente decentralizzato simulato in un ambiente centralizzato".
    - SimulationWorld / SwarmSimulation: fisica, incendi, canale radio, statistiche (conoscenza globale,
      perché sono il simulatore) — non prendono MAI decisioni operative per conto dei droni.
    - Drone: decide solo con i propri sensori locali, la propria memoria e i messaggi dei vicini.
      Non possiede riferimenti ad altri droni.
    - Renderer: solo disegno (Pygame), separato dalla simulazione (permette la modalità --headless).
"""

import argparse
import math
import random
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# ------------------------------------------------------------
# Configurazione
# ------------------------------------------------------------

# SIM
NUM_DRONES = 12
AREA_WIDTH = 20.0
AREA_HEIGHT = 12.0
SIM_TIME_STEP = 0.01
RANDOM_SEED = 42

WINDOW_WIDTH = 1600  # finestra Pygame
WINDOW_HEIGHT = 960  # aspect ratio 20:12

SHOW_FORCE_VECTORS = False         # default di --show-vectors (tasto V per cambiarlo a runtime)
SHOW_DRONES_COMMUNICATION = False  # default di --comm (tasto C per cambiarlo a runtime)
# Stile linee tratteggiate (RGBA)
DASHED_LINE_TARGET_COLOR = (255, 110, 255, 100)
DASHED_LINE_ANCHOR_COLOR = (110, 250, 110, 100)
COMMUNICATION_LINE_COLOR = (100, 160, 220, 60)

FIRE_GROWTH_RATE = 0.5  # Health points per second

# DRONI
COMMUNICATION_RADIUS = 2.5
TARGET_SEPARATION = 2.0
TARGET_REACHED_DISTANCE = 0.2
MAX_TARGET_SPEED = 20.0
MAX_FORCE_ON_TARGET = 10.0
# Smorzamento esponenziale della velocità del target [1/s]: decay = exp(-rate * dt).
# exp(-10.5 * 0.01) ≈ 0.90 -> identico al vecchio "TARGET_VEL_AGING_FACTOR = 0.1 per step",
# ma ora il valore ha un significato temporale e non dipende da SIM_TIME_STEP.
TARGET_VEL_DAMPING_RATE = 10.5
MAX_DRONE_SPEED = 1.0
ANCHOR_TO_TARGET_INTENSITY = 0.0075  # [1/s] L'ancora segue lentamente il target (costante di tempo ≈ 133 s)

K_DESIRED_VEL_TO_TARGET = 0.70
K_REPULSION_BETWEEN_TARGETS = 1.0
K_ANCHOR_DRAGGING = 0.2
K_BOUNDARY_REPULSION = 1.0

FIRE_DETECTION_RADIUS = 2.0
FIRE_GENERATION_MARGIN = 1.5
WATER_STATION_GENERATION_MARGIN = 2.0  # >= WATER_STATION_WAIT_RADIUS: l'anello di attesa resta nell'area
MARGIN_REPULSION_BOUNDARY = 1.5  # Distanza minima dal bordo per la repulsione del target

MAX_IDLE_STEPS = 30  # Step d'inattività prima di riprendere l'esplorazione

# Collision avoidance & Safety parameters
# Gerarchia esplicita delle distanze (controllata da validate_config()):
#   DRONE_IMPACT_RADIUS < EMERGENCY_AVOID_DISTANCE < AVOID_MIN_DISTANCE < COMMUNICATION_RADIUS
#   0.1 m collisione  <  0.8 m emergenza  <  1.0 m distanza operativa  <  2.5 m orizzonte di percezione
DRONE_IMPACT_RADIUS = 0.1       # Soglia reale di collisione fisica (solo rilevamento statistico)
EMERGENCY_AVOID_DISTANCE = 0.8  # Soglia locale di emergenza: l'evitamento prevale sul compito
AVOID_MIN_DISTANCE = 1.0        # Distanza di separazione minima desiderata (metri)
SAFE_DISTANCE_K_VEL = 0.6       # [s] Moltiplicatore ellisse di sicurezza su velocità: margine = AVOID_MIN_DISTANCE + K * |v|
AVOID_LOOKAHEAD = 1.5           # Orizzonte temporale predittivo per evitamento (secondi)
K_AVOID_REPULSION = 7.2         # Guadagno repulsivo campo potenziale (ex K_DAMPING_REPULSION, prima inutilizzato)
K_AVOID_DAMPING = 2.2           # Smorzamento velocità relativa di avvicinamento (ex K_DAMPING_DAMP, prima inutilizzato)
AVOID_MAX_CORRECTION = 3.0      # [m/s] Correzione massima di velocità dovuta all'evitamento predittivo
EMERGENCY_AVOID_GAIN = 6.0      # Guadagno repulsivo del livello di emergenza
EMERGENCY_DAMPING = 0.8         # Smorzamento della velocità relativa nel livello di emergenza
EMERGENCY_TARGET_WEIGHT = 0.25  # Quota di navigazione verso il target mantenuta durante l'emergenza
MAX_JERK = 8.0                  # Variazione massima accelerazione (m/s^3)

# Fire extinguishing & Saturation Behavior
FIRE_HEALTH = 200.0
FIRE_SPAWN_THRESHOLD = 1.20        # Soglia vita per propagazione incendio
FIRE_SPAWN_PROB_PER_STEP = 0.003   # Probabilità per passo di generare un nuovo incendio
FIRE_SPAWN_OFFSET_MAX = 3.0        # Raggio massimo offset incendio figlio
FIRE_SPAWN_INITIAL_HEALTH = 0.15   # Vita iniziale incendio figlio
DRONE_WATER_CAPACITY = 20.0
DRONE_WATER_FLOW_RATE = 5.0
FIRE_EXTINGUISH_RADIUS = 1.2
FIRE_WORK_RADIUS = 0.8             # Distanza dal centro a cui il drone si posiziona per spegnere (anello di lavoro)
MAX_DRONES_ON_FIRE = 3             # Max droni sullo stesso incendio (stima LOCALE di ciascun drone)
FIRE_SATURATION_BOUNCE_DISTANCE = 3.5  # Distanza (m) di allontanamento radiale se il fuoco è saturo (non è una forza)
FIRE_SATURATION_MEMORY_S = 5.0     # Per quanti secondi un incendio saturo viene escluso dalla scelta

# Stazione idrica / rifornimento
LOW_WATER_THRESHOLD = DRONE_WATER_CAPACITY * 0.1
WATER_STATION_REFILL_RATE = 10.0
NUM_WATER_STATIONS = 3
WATER_STATION_CAPACITY = 2         # Droni riforniti contemporaneamente (uno per slot di servizio)
WATER_STATION_SERVICE_RADIUS = 0.8
WATER_STATION_SLOT_RADIUS = 0.55   # Distanza degli slot di servizio dal centro della stazione
WATER_STATION_WAIT_RADIUS = 1.9    # Anello di attesa: abbastanza vicino da "sentire" chi è in servizio
WATER_STATION_MIN_SEPARATION = 5.0  # Separazione minima tra stazioni casuali (gli anelli di attesa non si sovrappongono)

# PID velocity control
PID_KP = 3.4
PID_KI = 0.6
PID_KD = 0.25
PID_INTEGRAL_LIMIT = 3.0
PID_MAX_OUTPUT_ACCEL = 4.0

# TTL memoria incendi (in secondi, convertiti in step)
# Un incendio non si sposta e non scompare da solo: può essere ricordato a lungo. Se nel frattempo
# viene spento, la notizia arriva via gossip (extinguished_fires) oppure il drone lo verifica
# arrivando sul posto. Prima era 100 step = 1 s, troppo poco rispetto ai tempi di volo (~20 s).
FIRE_MEMORY_TTL_S = 15.0
FIRE_MEMORY_TTL_STEPS = int(round(FIRE_MEMORY_TTL_S / SIM_TIME_STEP))
# Deve essere > FIRE_MEMORY_TTL_STEPS: una vecchia voce "attivo" non deve sopravvivere alla notizia "spento".
EXTINGUISHED_FIRE_MEMORY_TTL_STEPS = FIRE_MEMORY_TTL_STEPS * 3
FIRE_SATURATION_MEMORY_STEPS = int(round(FIRE_SATURATION_MEMORY_S / SIM_TIME_STEP))

# Numero incendi generati con --random-fires
NUM_FIRES = 3

GOLDEN_ANGLE = 2.399963  # rad, distribuisce direzioni di fallback senza sovrapposizioni

FirePos = Tuple[float, float]


def validate_config() -> List[str]:
    """Controlla che i parametri rispettino le ipotesi geometriche su cui si basa la logica."""
    problems = []
    if not DRONE_IMPACT_RADIUS < EMERGENCY_AVOID_DISTANCE < AVOID_MIN_DISTANCE < COMMUNICATION_RADIUS:
        problems.append("gerarchia distanze violata: IMPACT < EMERGENCY < AVOID_MIN < COMMUNICATION")
    # La saturazione è stimata solo dai messaggi dei vicini: due droni che spengono lo stesso incendio
    # (entrambi entro FIRE_EXTINGUISH_RADIUS) devono potersi sentire.
    if 2.0 * FIRE_EXTINGUISH_RADIUS > COMMUNICATION_RADIUS:
        problems.append("2*FIRE_EXTINGUISH_RADIUS > COMMUNICATION_RADIUS: la saturazione non è stimabile localmente")
    if FIRE_WORK_RADIUS + TARGET_REACHED_DISTANCE > FIRE_EXTINGUISH_RADIUS:
        problems.append("FIRE_WORK_RADIUS troppo grande: il drone sull'anello potrebbe non spegnere")
    # L'anello di lavoro deve ospitare MAX_DRONES_ON_FIRE droni a distanza >= AVOID_MIN_DISTANCE.
    if MAX_DRONES_ON_FIRE > 1 and 2.0 * FIRE_WORK_RADIUS * math.sin(math.pi / MAX_DRONES_ON_FIRE) < AVOID_MIN_DISTANCE:
        problems.append("anello di lavoro troppo piccolo per MAX_DRONES_ON_FIRE droni separati")
    if WATER_STATION_CAPACITY > 1 and 2.0 * WATER_STATION_SLOT_RADIUS * math.sin(math.pi / WATER_STATION_CAPACITY) < AVOID_MIN_DISTANCE:
        problems.append("slot della stazione troppo vicini tra loro")
    if WATER_STATION_SLOT_RADIUS + TARGET_REACHED_DISTANCE > WATER_STATION_SERVICE_RADIUS:
        problems.append("uno slot raggiunto potrebbe cadere fuori dall'area di servizio")
    if WATER_STATION_WAIT_RADIUS + WATER_STATION_SLOT_RADIUS > COMMUNICATION_RADIUS:
        problems.append("chi attende non riesce a sentire i droni in servizio")
    if WATER_STATION_WAIT_RADIUS - WATER_STATION_SLOT_RADIUS < AVOID_MIN_DISTANCE:
        problems.append("anello di attesa troppo vicino agli slot di servizio")
    if EXTINGUISHED_FIRE_MEMORY_TTL_STEPS <= FIRE_MEMORY_TTL_STEPS:
        problems.append("la memoria 'spento' deve durare più della memoria 'attivo'")
    return problems

# ------------------------------------------------------------
# Utils
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

def unit_from_angle(angle: float) -> np.ndarray:
    return np.array([math.cos(angle), math.sin(angle)], dtype=float)

def vec_to_tuple(v: np.ndarray) -> FirePos:
    return (float(v[0]), float(v[1]))


class PIDController:
    """Controllore PID generico 2D con limitazione di integrale, output e jerk.

    Ingresso: errore di velocità. Uscita: accelerazione limitata.
    Il jerk limita la variazione di accelerazione tra due step (inerzia dinamica del drone).
    """

    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float, max_output: float, max_jerk: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.max_output = max_output
        self.max_jerk = max_jerk
        self.integral = np.zeros(2, dtype=float)
        self.last_error = np.zeros(2, dtype=float)

    def reset(self) -> None:
        self.integral[:] = 0.0
        self.last_error[:] = 0.0

    def step(self, desired_velocity: np.ndarray, current_velocity: np.ndarray, current_acceleration: np.ndarray, dt: float) -> np.ndarray:
        """Calcola la nuova accelerazione dall'errore di velocità con limitazione jerk."""
        error = desired_velocity - current_velocity
        deriv = (error - self.last_error) / max(dt, 1e-12)
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        pid_output = self.kp * error + self.ki * self.integral + self.kd * deriv
        target_accel = clamp_magnitude(pid_output, self.max_output)

        accel_err = target_accel - current_acceleration
        max_accel_change = self.max_jerk * dt
        accel_change = clamp_magnitude(accel_err, max_accel_change)
        new_acceleration = current_acceleration + accel_change
        new_acceleration = clamp_magnitude(new_acceleration, self.max_output)

        self.last_error = error.copy()
        return new_acceleration

@dataclass
class Fire:
    pos: np.ndarray
    health: float = FIRE_HEALTH

    @property
    def active(self) -> bool:
        return self.health > 0.0

    def grow(self, dt: float) -> None:
        if self.active:
            self.health += FIRE_GROWTH_RATE * dt

    def extinguish(self, water: float) -> float:
        applied = min(max(water, 0.0), self.health)
        self.health -= applied
        return applied


# ------------------------------------------------------------
# Comunicazione
# ------------------------------------------------------------

@dataclass
class DroneMessage:
    """Tutto ciò che un drone sa degli altri arriva da qui: nessun accesso diretto agli oggetti Drone."""
    position: np.ndarray
    velocity: np.ndarray
    target: np.ndarray
    reloading: bool
    water_station_idx: Optional[int]
    refuel_claim_age: int
    station_slot: Optional[int]
    extinguishing: bool
    fire_target: Optional[FirePos]
    known_fires: Dict[FirePos, int]
    extinguished_fires: Dict[FirePos, int]


class CommunicationModule:
    """Memoria locale dei messaggi ricevuti dai droni vicini tramite doppio buffer.

    neighbors: messaggi già committati e utilizzabili nel round corrente.
    _incoming: messaggi ricevuti durante il broadcast e destinati al prossimo commit.
    Così tutti i droni, in un round, usano informazioni dello stesso istante logico.
    """

    def __init__(self, drone: 'Drone'):
        self.drone = drone
        self.neighbors: Dict[int, DroneMessage] = {}
        self._incoming: Dict[int, DroneMessage] = {}

    def begin_round(self) -> None:
        """Promuove a 'attivi' i messaggi accumulati per il round corrente."""
        self.neighbors = self._incoming
        self._incoming = {}

    def deliver(self, sender_idx: int, msg: DroneMessage) -> None:
        """Riceve un messaggio in scrittura per il commit del prossimo round."""
        self._incoming[sender_idx] = msg

    def merge_fire_knowledge(self) -> None:
        drone = self.drone

        # 1. Informazioni incendio spento (priorità massima)
        for message in self.neighbors.values():
            for fire_pos, age in message.extinguished_fires.items():
                if fire_pos not in drone.extinguished_fires or age < drone.extinguished_fires[fire_pos]:
                    drone.extinguished_fires[fire_pos] = age

        # 2. Propagazione incendi attivi
        for message in self.neighbors.values():
            for fire_pos, age in message.known_fires.items():
                if fire_pos in drone.extinguished_fires:
                    continue
                if fire_pos not in drone.known_fires or age < drone.known_fires[fire_pos]:
                    drone.known_fires[fire_pos] = age

        # 3. Pulizia della memoria attiva per incendi confermati spenti
        for fire_pos in list(drone.known_fires):
            if fire_pos in drone.extinguished_fires:
                del drone.known_fires[fire_pos]


# ------------------------------------------------------------
# SimulationWorld
# ------------------------------------------------------------

class SimulationWorld:
    """Ambiente: geometria, sensori simulati, canale radio e azioni fisiche.

    Mantiene lo stato fisico globale perché è il simulatore, ma non decide nulla per i droni.
    """

    def __init__(self, drones: List['Drone'], fires: List[Fire], area_width: float, area_height: float,
                 water_stations: List[np.ndarray]):
        self._drones = drones
        self._fires = fires
        self.area_width = area_width
        self.area_height = area_height
        self.water_stations = water_stations  # mappa nota a priori (infrastruttura fissa)
        self._neighbor_cache: Dict[int, List['Drone']] = {}
        self.step_counter = 0
        self.extinguished_count = 0

    @property
    def fires(self) -> List[Fire]:
        return self._fires

    def refresh_neighbors(self) -> Dict[int, List['Drone']]:
        """Modello di propagazione radio: chi può ricevere i messaggi di chi (raggio COMMUNICATION_RADIUS).

        È fisica del canale, non coordinamento: il drone non riceve mai questa mappa, solo i messaggi.
        """
        neighbor_map: Dict[int, List['Drone']] = {d.idx: [] for d in self._drones}
        for i, drone1 in enumerate(self._drones):
            for drone2 in self._drones[i + 1:]:
                if np.linalg.norm(drone1.position - drone2.position) <= COMMUNICATION_RADIUS:
                    neighbor_map[drone1.idx].append(drone2)
                    neighbor_map[drone2.idx].append(drone1)
        self._neighbor_cache = neighbor_map
        return neighbor_map

    def broadcast(self, sender_idx: int, msg: DroneMessage) -> None:
        """Consegna il messaggio a tutti i droni nel raggio radio del mittente."""
        for receiver in self._neighbor_cache.get(sender_idx, []):
            receiver.communication.deliver(sender_idx, msg)

    def sense_fires(self, position: np.ndarray) -> List[Fire]:
        """Sensore di bordo: incendi attivi entro FIRE_DETECTION_RADIUS."""
        return [
            fire for fire in self._fires
            if fire.active and np.linalg.norm(position - fire.pos) <= FIRE_DETECTION_RADIUS
        ]

    def has_active_fire_near(self, position: np.ndarray, radius: float) -> bool:
        return any(fire.active and np.linalg.norm(position - fire.pos) <= radius for fire in self._fires)

    def request_extinguish(self, position: np.ndarray, water_available: float, flow_rate: float) -> Tuple[float, List[FirePos]]:
        """Attuatore: spruzza acqua sugli incendi nel raggio.

        Restituisce l'acqua usata e le posizioni degli incendi spenti da questo getto
        (il drone osserva direttamente l'esito della propria azione).
        """
        remaining = min(max(water_available, 0.0), flow_rate * SIM_TIME_STEP)
        total_used = 0.0
        extinguished: List[FirePos] = []
        for fire in self._fires:
            if remaining <= 1e-12:
                break
            if not fire.active or np.linalg.norm(position - fire.pos) > FIRE_EXTINGUISH_RADIUS:
                continue
            used = fire.extinguish(remaining)
            total_used += used
            remaining -= used
            if not fire.active:
                extinguished.append(vec_to_tuple(fire.pos))
        if extinguished:
            self.extinguished_count += len(extinguished)
            self._remove_dead_fires()
        return total_used, extinguished

    def update_fires(self, rng: random.Random) -> None:
        self.step_counter += 1
        spawn_queue: List[Fire] = []
        for fire in self._fires:
            fire.grow(SIM_TIME_STEP)
            if fire.health > FIRE_HEALTH * FIRE_SPAWN_THRESHOLD:
                if rng.random() < FIRE_SPAWN_PROB_PER_STEP:
                    angle = rng.uniform(0.0, 2.0 * math.pi)
                    dist = rng.uniform(0.5, FIRE_SPAWN_OFFSET_MAX)
                    new_pos = fire.pos + dist * unit_from_angle(angle)
                    new_pos[0] = float(np.clip(new_pos[0], FIRE_GENERATION_MARGIN, self.area_width - FIRE_GENERATION_MARGIN))
                    new_pos[1] = float(np.clip(new_pos[1], FIRE_GENERATION_MARGIN, self.area_height - FIRE_GENERATION_MARGIN))
                    spawn_queue.append(Fire(pos=new_pos, health=FIRE_HEALTH * FIRE_SPAWN_INITIAL_HEALTH))
        self._fires.extend(spawn_queue)
        self._remove_dead_fires()

    def _remove_dead_fires(self) -> None:
        self._fires[:] = [fire for fire in self._fires if fire.active]


# ------------------------------------------------------------
# Drone
# ------------------------------------------------------------

class Drone:
    """Logica autonoma del drone: percezione, comunicazione, decisione e attuazione.

    Pipeline di uno step (vedi step()):
        1. commit dei messaggi ricevuti;
        2. percezione locale e aging della memoria;
        3. fusione della conoscenza dei vicini (gossip);
        4. scelta del compito: rifornimento > incendio > esplorazione;
        5. navigazione (target) + collision avoidance -> velocità desiderata;
        6. PID -> accelerazione -> velocità -> posizione;
        7. azione fisica (spegnimento / rifornimento).

    Il drone NON ha riferimenti agli altri droni: li conosce solo tramite self.communication.neighbors.
    """

    def __init__(self, idx: int, rng: random.Random, world: SimulationWorld, seed: int):
        self.idx = idx
        self.world = world
        # Generatore privato: le scelte del drone non consumano il generatore degli incendi,
        # quindi modificare il comportamento dei droni non cambia la sequenza degli incendi.
        self.rng = random.Random(f"drone-{seed}-{idx}")
        self.position = np.array([rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float)
        self.velocity = np.zeros(2, dtype=float)
        self.acceleration = np.zeros(2, dtype=float)
        self.original_target = np.array([rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float)
        self.anchor_target = self.original_target.copy()
        self.target = self.original_target.copy()
        self.target_velocity = np.zeros(2, dtype=float)
        self.known_fires: Dict[FirePos, int] = {}
        self.extinguished_fires: Dict[FirePos, int] = {}
        self.saturated_fires: Dict[FirePos, int] = {}  # incendio -> step residui di esclusione
        self.fire_target: Optional[FirePos] = None
        # Compito interrotto dal rifornimento: intenzione PRIVATA, non comunicata e senza aging.
        # Non è conoscenza sul mondo (quella sta in known_fires, con la sua età), è solo
        # "dove stavo lavorando prima di andare a fare acqua".
        self.interrupted_fire: Optional[FirePos] = None
        self.interrupted_target: Optional[np.ndarray] = None
        self.idle_steps = 0
        self.desired_velocity = np.zeros(2, dtype=float)
        self.last_applied_force = np.zeros(2, dtype=float)
        self.last_avoidance_force = np.zeros(2, dtype=float)
        self.water = DRONE_WATER_CAPACITY
        self.reloading = False
        self.water_station_idx: Optional[int] = None
        self.refuel_claim_age = 0
        self.station_slot: Optional[int] = None
        self.communication = CommunicationModule(self)
        self.pid = PIDController(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT, PID_MAX_OUTPUT_ACCEL, MAX_JERK)

    @property
    def _neighbor_messages(self) -> Dict[int, DroneMessage]:
        return self.communication.neighbors

    # --------------------------------------------------------
    # Comunicazione
    # --------------------------------------------------------

    def _build_message(self) -> DroneMessage:
        return DroneMessage(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            target=self.target.copy(),
            reloading=self.reloading,
            water_station_idx=self.water_station_idx,
            refuel_claim_age=self.refuel_claim_age,
            station_slot=self.station_slot,
            extinguishing=self.is_extinguishing_fire(),
            fire_target=self.fire_target,
            known_fires=dict(self.known_fires),
            extinguished_fires=dict(self.extinguished_fires),
        )

    def pre_step(self) -> None:
        """Calcola il messaggio del drone e lo trasmette ai vicini (via canale radio del World)."""
        self.world.broadcast(self.idx, self._build_message())

    def merge_neighbor_knowledge(self) -> None:
        self.communication.merge_fire_knowledge()

    # --------------------------------------------------------
    # Percezione
    # --------------------------------------------------------

    def sense_environment(self) -> None:
        # Aging memoria incendi attivi
        for fire_pos in list(self.known_fires):
            self.known_fires[fire_pos] += 1
            if self.known_fires[fire_pos] > FIRE_MEMORY_TTL_STEPS:
                del self.known_fires[fire_pos]

        # Aging memoria incendi spenti
        for fire_pos in list(self.extinguished_fires):
            self.extinguished_fires[fire_pos] += 1
            if self.extinguished_fires[fire_pos] > EXTINGUISHED_FIRE_MEMORY_TTL_STEPS:
                del self.extinguished_fires[fire_pos]

        # Scadenza dell'esclusione degli incendi saturi
        for fire_pos in list(self.saturated_fires):
            self.saturated_fires[fire_pos] -= 1
            if self.saturated_fires[fire_pos] <= 0:
                del self.saturated_fires[fire_pos]

        sensed = {vec_to_tuple(fire.pos) for fire in self.world.sense_fires(self.position)}

        # Verifica locale: se sono abbastanza vicino da vedere un incendio che ricordo, ma il sensore
        # non lo vede, allora è spento. Lo registro così la notizia si propaga agli altri.
        for fire_pos in list(self.known_fires):
            if fire_pos in sensed:
                continue
            if np.linalg.norm(self.position - np.array(fire_pos)) <= FIRE_DETECTION_RADIUS:
                del self.known_fires[fire_pos]
                self.extinguished_fires[fire_pos] = 0
                if self.fire_target == fire_pos:
                    self.fire_target = None

        # Rilevamento diretto: l'osservazione fisica corrente ha priorità su qualunque voce ricevuta
        for pos_key in sensed:
            self.known_fires[pos_key] = 0
            self.extinguished_fires.pop(pos_key, None)

        if self.fire_target is not None and self.fire_target not in self.known_fires:
            self.fire_target = None

    # --------------------------------------------------------
    # Fire task allocation (decentralizzata)
    # --------------------------------------------------------

    def _fire_rank(self, fire_pos: FirePos) -> int:
        """Quanti droni vicini hanno priorità su di me per questo incendio.

        Non esiste un contatore globale: MAX_DRONES_ON_FIRE è confrontato con una stima locale.
        Priorità: prima chi sta già spegnendo questo incendio (non si scalza chi lavora),
        poi chi è più vicino, a parità l'indice minore. Tutti i droni applicano la stessa regola
        sugli stessi dati, quindi convergono sulla stessa decisione senza coordinatore.
        """
        fire_arr = np.array(fire_pos)
        my_dist = float(np.linalg.norm(self.position - fire_arr))
        my_key = (0 if (self.water > 0.0 and my_dist <= FIRE_EXTINGUISH_RADIUS) else 1, my_dist, self.idx)
        rank = 0
        for other_idx, msg in self._neighbor_messages.items():
            dist = float(np.linalg.norm(msg.position - fire_arr))
            working_here = msg.extinguishing and dist <= FIRE_EXTINGUISH_RADIUS
            if msg.fire_target != fire_pos and not working_here:
                continue
            if (0 if working_here else 1, dist, other_idx) < my_key:
                rank += 1
        return rank

    def _select_fire(self) -> Tuple[Optional[FirePos], Optional[FirePos]]:
        """Sceglie l'incendio da ingaggiare.

        Restituisce (incendio scelto, incendio saturo vicino da cui allontanarsi).
        Il target corrente ha la precedenza (impegno: niente oscillazioni tra incendi equidistanti);
        gli incendi saturi vengono esclusi per FIRE_SATURATION_MEMORY_S e si passa al successivo.
        """
        candidates = sorted(
            (p for p in self.known_fires if p not in self.saturated_fires),
            key=lambda p: np.linalg.norm(np.array(p) - self.position),
        )
        if self.fire_target in candidates:
            candidates.remove(self.fire_target)
            candidates.insert(0, self.fire_target)

        bounce_from: Optional[FirePos] = None
        for fire_pos in candidates:
            if self._fire_rank(fire_pos) < MAX_DRONES_ON_FIRE:
                return fire_pos, None
            self.saturated_fires[fire_pos] = FIRE_SATURATION_MEMORY_STEPS
            if bounce_from is None and np.linalg.norm(self.position - np.array(fire_pos)) <= FIRE_DETECTION_RADIUS:
                bounce_from = fire_pos
        return None, bounce_from

    def _engage_fire(self, fire_pos: FirePos) -> None:
        """Ingaggio: il target è il punto dell'anello di lavoro nella direzione da cui arrivo.

        Il target vincola solo la distanza dal fuoco; la posizione lungo l'anello la decide
        l'avoidance fisica, che distribuisce da sola i droni attorno all'incendio.
        """
        self.fire_target = fire_pos
        fire_arr = np.array(fire_pos, dtype=float)
        bearing = normalize(self.position - fire_arr) # vettore radiale dal fuoco al drone, ma potrebbe essere nullo se il drone è sopra il fuoco
        if np.linalg.norm(bearing) < 1e-6:
            bearing = unit_from_angle(self.idx * GOLDEN_ANGLE)
        self.target = fire_arr + bearing * FIRE_WORK_RADIUS
        self.target[0] = np.clip(self.target[0], 0.0, self.world.area_width)
        self.target[1] = np.clip(self.target[1], 0.0, self.world.area_height)
        self.original_target = fire_arr.copy()
        self.anchor_target = self.target.copy()  # al rilascio del compito il drone riparte da qui
        self.target_velocity[:] = 0.0

    def _bounce_away_from(self, fire_pos: FirePos) -> None:
        """Rimbalzo radiale da un incendio saturo."""
        fire_arr = np.array(fire_pos, dtype=float)
        bounce_dir = normalize(self.position - fire_arr)
        if np.linalg.norm(bounce_dir) < 1e-6:
            bounce_dir = normalize(self.velocity) if np.linalg.norm(self.velocity) > 1e-6 else unit_from_angle(self.idx * GOLDEN_ANGLE)

        # Applica lo spostamento usando la variabile di configurazione
        self.target = self.position + bounce_dir * FIRE_SATURATION_BOUNCE_DISTANCE
        self.target[0] = np.clip(self.target[0], 0.0, self.world.area_width)
        self.target[1] = np.clip(self.target[1], 0.0, self.world.area_height)
        self.original_target = self.target.copy()
        self.anchor_target = self.target.copy()
        self.target_velocity[:] = 0.0

    # --------------------------------------------------------
    # Campo di forze sul target (solo in esplorazione)
    # --------------------------------------------------------

    def compute_repulsion_between_targets(self) -> np.ndarray:
        """Separazione dei target: usa i target comunicati dai vicini, non gli oggetti Drone."""
        force = np.zeros(2, dtype=float)
        for msg in self._neighbor_messages.values():
            delta = self.target - msg.target
            dist = np.linalg.norm(delta)
            if dist >= TARGET_SEPARATION:
                continue
            if dist < 1e-6:
                pos_delta = self.position - msg.position
                direction = normalize(pos_delta) if np.linalg.norm(pos_delta) > 1e-6 else unit_from_angle(self.idx * GOLDEN_ANGLE)
                force += K_REPULSION_BETWEEN_TARGETS * TARGET_SEPARATION * direction
            else:
                force += K_REPULSION_BETWEEN_TARGETS * (TARGET_SEPARATION - dist) * delta / dist
        return clamp_magnitude(force, MAX_FORCE_ON_TARGET)

    def compute_boundary_force(self) -> np.ndarray:
        force = np.zeros(2, dtype=float)
        margin = MARGIN_REPULSION_BOUNDARY
        if self.target[0] < margin:
            force[0] += K_BOUNDARY_REPULSION * (margin - self.target[0]) / max(margin, 1e-6)
        elif self.target[0] > self.world.area_width - margin:
            force[0] -= K_BOUNDARY_REPULSION * (self.target[0] - (self.world.area_width - margin)) / max(margin, 1e-6)
        if self.target[1] < margin:
            force[1] += K_BOUNDARY_REPULSION * (margin - self.target[1]) / max(margin, 1e-6)
        elif self.target[1] > self.world.area_height - margin:
            force[1] -= K_BOUNDARY_REPULSION * (self.target[1] - (self.world.area_height - margin)) / max(margin, 1e-6)
        return force

    def compute_anchor_force(self) -> np.ndarray:
        return K_ANCHOR_DRAGGING * (self.anchor_target - self.target)

    def compute_total_force(self) -> np.ndarray:
        total = self.compute_repulsion_between_targets() + self.compute_anchor_force() + self.compute_boundary_force()
        return clamp_magnitude(total, MAX_FORCE_ON_TARGET)

    # --------------------------------------------------------
    # Dinamica del target
    # --------------------------------------------------------

    def _sample_exploration_target(self) -> np.ndarray:
        return np.array([self.rng.uniform(0.0, self.world.area_width), self.rng.uniform(0.0, self.world.area_height)], dtype=float)

    def _refresh_anchor(self) -> None:
        delta = self.target - self.anchor_target
        if np.linalg.norm(delta) > 1e-6:
            self.anchor_target += delta * (1.0 - math.exp(-ANCHOR_TO_TARGET_INTENSITY * SIM_TIME_STEP))

    def has_reached_target(self) -> bool:
        return bool(np.linalg.norm(self.position - self.target) < TARGET_REACHED_DISTANCE)

    def _update_target_position(self) -> None:
        force = self.compute_total_force()
        self.last_applied_force = force.copy()
        decay = math.exp(-TARGET_VEL_DAMPING_RATE * SIM_TIME_STEP)
        self.target_velocity = decay * self.target_velocity + force * SIM_TIME_STEP
        self.target_velocity = clamp_magnitude(self.target_velocity, MAX_TARGET_SPEED)
        self.target += self.target_velocity * SIM_TIME_STEP
        self.target[0] = np.clip(self.target[0], 0.0, self.world.area_width)
        self.target[1] = np.clip(self.target[1], 0.0, self.world.area_height)
        self._refresh_anchor()

    # --------------------------------------------------------
    # Collision avoidance (sempre attiva, anche durante lo spegnimento)
    # --------------------------------------------------------

    def _compute_collision_avoidance(self) -> Tuple[np.ndarray, bool, np.ndarray]:
        """Evitamento collisioni tra droni: campo potenziale predittivo + livello di emergenza.

        Livello 1 (predittivo): per ogni vicino calcola il punto di massimo avvicinamento (CPA)
            nell'intervallo [0, AVOID_LOOKAHEAD]:
                t_ca = clip(-(rel·v_rel) / |v_rel|², 0, T)      closest = rel + v_rel * t_ca
            Se |closest| < margine di sicurezza -> repulsione lungo closest (allarga la distanza di
            passaggio, cioè spinge di lato) + smorzamento della velocità di avvicinamento.
        Livello 2 (emergenza): distanza attuale < EMERGENCY_AVOID_DISTANCE -> la separazione
            prevale quasi del tutto sul compito.
        Livello 3 (collisione fisica, < DRONE_IMPACT_RADIUS) è solo rilevato dal simulatore.

        Restituisce (correzione predittiva, emergenza attiva, vettore di emergenza).
        """
        predictive = np.zeros(2, dtype=float)
        emergency_vector = np.zeros(2, dtype=float)
        emergency = False
        safety_margin = AVOID_MIN_DISTANCE + SAFE_DISTANCE_K_VEL * np.linalg.norm(self.velocity) # più veloce il drone -> maggiore il margine necessario ad evitare collisioni

        # Per ogni vicino, calcola il punto di massimo avvicinamento e la distanza attuale.
        for other_idx, state in self._neighbor_messages.items():
            rel = self.position - state.position
            distance = np.linalg.norm(rel)
            v_rel = self.velocity - state.velocity
            if distance > 1e-9:
                unit_now = rel / distance
            else:
                # Sovrapposizione esatta: direzioni opposte per i due droni
                unit_now = np.array([1.0, 0.0]) if self.idx < other_idx else np.array([-1.0, 0.0])

            # Tempo e distanza di massimo avvicinamento
            speed_sq = float(np.dot(v_rel, v_rel))
            t_ca = float(np.clip(-np.dot(rel, v_rel) / speed_sq, 0.0, AVOID_LOOKAHEAD)) if speed_sq > 1e-12 else 0.0 # tempo di massimo avvicinamento predetto, ma se v_rel è nullo allora t_ca = 0
            closest = rel + v_rel * t_ca
            closest_distance = np.linalg.norm(closest)
            if closest_distance > 1e-6:
                direction = closest / closest_distance
            elif speed_sq > 1e-12:
                # Frontale perfetto: scarto laterale. Per l'altro drone v_rel è opposta,
                # quindi la perpendicolare è opposta e i due si spostano da lati diversi.
                v_dir = v_rel / math.sqrt(speed_sq)
                direction = np.array([-v_dir[1], v_dir[0]])
            else:
                direction = unit_now

            closing_speed = max(0.0, -float(np.dot(v_rel, unit_now)))

            if closest_distance < safety_margin:
                penetration = (safety_margin - closest_distance) / safety_margin
                predictive += direction * (K_AVOID_REPULSION * penetration + K_AVOID_DAMPING * closing_speed)

            if distance < EMERGENCY_AVOID_DISTANCE:
                emergency = True
                emergency_vector += unit_now * (EMERGENCY_AVOID_DISTANCE - distance) * EMERGENCY_AVOID_GAIN - v_rel * EMERGENCY_DAMPING

        return clamp_magnitude(predictive, AVOID_MAX_CORRECTION), emergency, emergency_vector

    def _compute_desired_velocity_with_avoidance(self) -> np.ndarray:
        """Navigazione verso il target + avoidance -> velocità desiderata limitata.

        La navigazione viene limitata a MAX_DRONE_SPEED PRIMA di sommare l'avoidance: altrimenti,
        con un target lontano (es. 0.7 * 15 m = 10.5 m/s), la correzione verrebbe diluita dal clamp finale.
        """
        navigation = clamp_magnitude(K_DESIRED_VEL_TO_TARGET * (self.target - self.position), MAX_DRONE_SPEED)
        predictive, emergency, emergency_vector = self._compute_collision_avoidance()
        if emergency:
            self.last_avoidance_force = emergency_vector
            desired_velocity = emergency_vector + EMERGENCY_TARGET_WEIGHT * navigation
        else:
            self.last_avoidance_force = predictive
            desired_velocity = navigation + predictive
        return clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)

    # --------------------------------------------------------
    # Moto
    # --------------------------------------------------------

    def _integrate_motion(self, desired_velocity: np.ndarray) -> None:
        self.desired_velocity = clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)
        self.acceleration = self.pid.step(self.desired_velocity, self.velocity, self.acceleration, SIM_TIME_STEP)
        self.velocity = clamp_magnitude(self.velocity + self.acceleration * SIM_TIME_STEP, MAX_DRONE_SPEED)
        self.position += self.velocity * SIM_TIME_STEP
        self.position[0] = np.clip(self.position[0], 0.0, self.world.area_width)
        self.position[1] = np.clip(self.position[1], 0.0, self.world.area_height)

    def _maybe_resume_exploration(self) -> None:
        """Nessun compito attivo e target raggiunto -> dopo MAX_IDLE_STEPS nuovo target casuale."""
        if not self.reloading and self.fire_target is None and self.has_reached_target():
            self.idle_steps += 1
            if self.idle_steps > MAX_IDLE_STEPS:
                self.original_target = self._sample_exploration_target()
                self.anchor_target = self.original_target.copy()
                self.target = self.original_target.copy()
                self.target_velocity[:] = 0.0
                self.idle_steps = 0
        else:
            self.idle_steps = 0

    # --------------------------------------------------------
    # Stazioni idriche (coda FIFO decentralizzata)
    # --------------------------------------------------------

    def _is_in_station_service_area(self, position: np.ndarray, station_idx: int) -> bool:
        return bool(np.linalg.norm(position - self.world.water_stations[station_idx]) <= WATER_STATION_SERVICE_RADIUS)

    def _station_priority(self, station_slot: Optional[int], position: np.ndarray, claim_age: int, idx: int) -> Tuple[int, int, int]:
        """Chiave di priorità (minore = prima). Chi è già in servizio non viene scalzato;
        poi FIFO: chi ha richiesto il rifornimento da più tempo (claim_age maggiore) passa prima.
        """
        in_service = station_slot is not None and self._is_in_station_service_area(position, self.water_station_idx)
        return (0 if in_service else 1, -claim_age, idx)

    def _station_competitors(self) -> List[Tuple[Tuple[int, int, int], int, Optional[int]]]:
        """(priorità, idx, slot) di tutti i droni noti, me compreso, diretti alla mia stazione, ordinati."""
        competitors = [(self._station_priority(self.station_slot, self.position, self.refuel_claim_age, self.idx),
                        self.idx, self.station_slot)]
        for other_idx, msg in self._neighbor_messages.items():
            if msg.reloading and msg.water_station_idx == self.water_station_idx:
                competitors.append((self._station_priority(msg.station_slot, msg.position, msg.refuel_claim_age, other_idx),
                                    other_idx, msg.station_slot))
        competitors.sort()
        return competitors

    def _estimate_station_load(self, station_idx: int) -> int:
        """Droni noti (vicini) che stanno usando o aspettando questa stazione."""
        return sum(1 for msg in self._neighbor_messages.values()
                   if msg.reloading and msg.water_station_idx == station_idx)

    def _select_water_station(self) -> int:
        # Priorità: stazioni non sature (stima locale), poi la più vicina, poi la meno carica
        candidates = []
        for i, pos in enumerate(self.world.water_stations):
            load = self._estimate_station_load(i)
            candidates.append((load >= WATER_STATION_CAPACITY, np.linalg.norm(self.position - pos), load, i))
        return min(candidates)[3]

    def _station_slot_position(self, station_idx: int, slot: int) -> np.ndarray:
        station_pos = self.world.water_stations[station_idx]
        if WATER_STATION_CAPACITY <= 1:
            return station_pos.copy()
        angle = station_idx * math.pi / 3.0 + 2.0 * math.pi * slot / WATER_STATION_CAPACITY
        return station_pos + unit_from_angle(angle) * WATER_STATION_SLOT_RADIUS

    def _update_station_slot(self) -> None:
        """Ammissione (primi WATER_STATION_CAPACITY in priorità) e scelta di uno slot libero.

        Lo slot è "appiccicoso": una volta preso lo si tiene fino a fine rifornimento. In caso di
        conflitto (due droni scelgono lo stesso slot con dati vecchi di uno step) cede chi ha priorità minore.
        """
        competitors = self._station_competitors()
        my_rank = next(i for i, (_, idx, _) in enumerate(competitors) if idx == self.idx)
        if my_rank >= WATER_STATION_CAPACITY:
            self.station_slot = None
            return
        my_key = competitors[my_rank][0]
        others = [(key, slot) for key, idx, slot in competitors if idx != self.idx and slot is not None]
        if self.station_slot is not None and any(slot == self.station_slot and key < my_key for key, slot in others):
            self.station_slot = None
        if self.station_slot is None:
            taken = {slot for _, slot in others}
            free = [s for s in range(WATER_STATION_CAPACITY) if s not in taken]
            if free:
                self.station_slot = free[0]

    def _maybe_start_reload(self) -> None:
        """Se il drone ha poca acqua e non sta già rifornendo, sceglie una stazione e inizia a dirigersi verso di essa."""
        if self.reloading or self.is_extinguishing_fire() or self.water > LOW_WATER_THRESHOLD:
            return
        self.reloading = True
        self.water_station_idx = self._select_water_station()
        self.refuel_claim_age = 0
        self.station_slot = None
        # Salva il compito interrotto prima di sovrascrivere il target con la stazione.
        self.interrupted_fire = self.fire_target
        self.interrupted_target = self.target.copy()
        self.fire_target = None
        self.original_target = self.world.water_stations[self.water_station_idx].copy()
        self.target = self.original_target.copy()
        self.anchor_target = self.target.copy()
        self.target_velocity[:] = 0.0
        self.pid.reset()

    def _move_toward_station(self) -> None:
        if self.water_station_idx is None:
            self.water_station_idx = self._select_water_station()
        self.refuel_claim_age += 1
        self._update_station_slot()
        station_pos = self.world.water_stations[self.water_station_idx]

        if self.station_slot is not None:
            # Ammesso: vado nel mio slot di servizio
            target_pos = self._station_slot_position(self.water_station_idx, self.station_slot)
        else:
            # In coda: attendo sull'anello, nella direzione da cui arrivo. Come per gli incendi,
            # la spaziatura lungo l'anello la fa l'avoidance.
            bearing = normalize(self.position - station_pos)
            if np.linalg.norm(bearing) < 1e-6:
                bearing = unit_from_angle(self.idx * GOLDEN_ANGLE)
            target_pos = station_pos + bearing * WATER_STATION_WAIT_RADIUS
            target_pos[0] = np.clip(target_pos[0], 0.0, self.world.area_width)
            target_pos[1] = np.clip(target_pos[1], 0.0, self.world.area_height)

        self.target = target_pos
        self.anchor_target = target_pos.copy()
        self.target_velocity[:] = 0.0
        self._integrate_motion(self._compute_desired_velocity_with_avoidance())

    def try_reload(self) -> None:
        # Si rifornisce solo chi è stato ammesso (ha uno slot) ed è nello slot, dentro l'area di servizio.
        if not self.reloading or self.station_slot is None or not self.has_reached_target():
            return
        if not self._is_in_station_service_area(self.position, self.water_station_idx):
            return
        self.water = min(DRONE_WATER_CAPACITY, self.water + WATER_STATION_REFILL_RATE * SIM_TIME_STEP)
        if self.water >= DRONE_WATER_CAPACITY - 1e-6:
            self.water = DRONE_WATER_CAPACITY
            self.reloading = False
            self.water_station_idx = None
            self.refuel_claim_age = 0
            self.station_slot = None
            self.target_velocity[:] = 0.0
            self.fire_target = None
            # Riprende il compito interrotto invece di ripartire da un punto casuale.
            self.target = self._resume_point_after_reload()
            self.original_target = self.target.copy()
            self.anchor_target = self.target.copy()
            self.interrupted_fire = None
            self.interrupted_target = None
            self.pid.reset()

    def _resume_point_after_reload(self) -> np.ndarray:
        """Da dove riparte il drone dopo il pieno.

        Priorità:
            1. l'incendio che stava gestendo, se non è arrivata una smentita;
            2. il punto in cui si trovava il suo target quando è scattato il rifornimento;
            3. un nuovo target di esplorazione casuale.

        L'incendio NON viene reinserito in known_fires: il drone non ha osservato niente di nuovo,
        ha solo l'intenzione di tornare a controllare. Il viaggio di ritorno può durare più di
        FIRE_MEMORY_TTL_S, quindi senza questo l'informazione andrebbe persa proprio mentre il
        drone è fermo in coda. Arrivato entro FIRE_DETECTION_RADIUS, sense_environment() conferma
        (known_fires[p] = 0) oppure falsifica (extinguished_fires[p] = 0).
        """
        if self.interrupted_fire is not None and self.interrupted_fire not in self.extinguished_fires:
            return np.array(self.interrupted_fire, dtype=float)
        if self.interrupted_target is not None:
            return self.interrupted_target.copy()
        return self._sample_exploration_target()

    # --------------------------------------------------------
    # Interazione con gli incendi
    # --------------------------------------------------------

    def is_extinguishing_fire(self) -> bool:
        return self.water > 0.0 and self.world.has_active_fire_near(self.position, FIRE_EXTINGUISH_RADIUS)

    def try_extinguish(self) -> None:
        if self.water <= 0.0:
            return
        used, extinguished = self.world.request_extinguish(self.position, self.water, DRONE_WATER_FLOW_RATE)
        self.water -= used
        if self.water < 1e-9:
            self.water = 0.0
        # Il drone ha visto spegnersi l'incendio: lo registra come spento, così la notizia si propaga.
        # (Prima cancellava l'incendio dalla memoria a OGNI getto d'acqua, anche se era ancora acceso,
        # e non lo segnava mai come spento.)
        for fire_pos in extinguished:
            self.known_fires.pop(fire_pos, None)
            self.saturated_fires.pop(fire_pos, None)
            self.extinguished_fires[fire_pos] = 0
            if self.fire_target == fire_pos:
                self.fire_target = None

    # --------------------------------------------------------
    # Decisione
    # --------------------------------------------------------

    def decide_and_move(self) -> None:
        # Priorità dei compiti: rifornimento > incendio > esplorazione
        self._maybe_start_reload()
        if self.reloading:
            self._move_toward_station()
            return

        # 1. Rilevamento incendio locale/scoperto (con stima locale della saturazione)
        selected_fire, saturated_nearby = self._select_fire()

        if selected_fire is not None:
            # 3. INCENDIO DISPONIBILE: Ingaggio (target sull'anello di lavoro)
            self._engage_fire(selected_fire)
        else:
            self.fire_target = None
            # 2. SE L'INCENDIO È SATURO: Rimbalzo radiale
            if saturated_nearby is not None:
                self._bounce_away_from(saturated_nearby)
            # Esplorazione: il target si muove nel campo di forze (ancora, separazione, bordi)
            self._update_target_position()

        # 4. Integrazione del movimento standard tramite PID (con collision avoidance)
        self._integrate_motion(self._compute_desired_velocity_with_avoidance())
        self._maybe_resume_exploration()

    def step(self) -> None:
        """Avanzamento di un passo di simulazione."""
        self.communication.begin_round()   # 1. commit dei messaggi ricevuti
        self.sense_environment()           # 2. percezione locale
        self.merge_neighbor_knowledge()    # 3. fusione conoscenza distribuita
        self.decide_and_move()             # 4. decisione autonoma + movimento
        self.try_extinguish()              # 5. azione fisica
        self.try_reload()                  # 6. rifornimento


# ------------------------------------------------------------
# SwarmSimulation (orchestrazione centrale, nessuna decisione operativa)
# ------------------------------------------------------------

class SwarmSimulation:
    @staticmethod
    def _default_fires() -> List[Fire]:
        positions = [
            (AREA_WIDTH * 0.20, AREA_HEIGHT * 0.75),
            (AREA_WIDTH * 0.50, AREA_HEIGHT * 0.25),
            (AREA_WIDTH * 0.80, AREA_HEIGHT * 0.75),
        ]
        return [Fire(np.array(pos, dtype=float)) for pos in positions]

    def _random_fires(self, count: int) -> List[Fire]:
        margin = FIRE_GENERATION_MARGIN
        return [Fire(np.array([self.rng.uniform(margin, AREA_WIDTH - margin),
                               self.rng.uniform(margin, AREA_HEIGHT - margin)], dtype=float))
                for _ in range(count)]

    @staticmethod
    def _default_water_stations() -> List[np.ndarray]:
        positions = [
            (AREA_WIDTH * 0.20, AREA_HEIGHT * 0.25),
            (AREA_WIDTH * 0.50, AREA_HEIGHT * 0.75),
            (AREA_WIDTH * 0.80, AREA_HEIGHT * 0.25),
        ]
        return [np.array(pos, dtype=float) for pos in positions[:NUM_WATER_STATIONS]]

    def _random_water_stations(self, count: int) -> List[np.ndarray]:
        margin = WATER_STATION_GENERATION_MARGIN
        stations: List[np.ndarray] = []
        for _ in range(count * 100):
            if len(stations) >= count:
                break
            pos = np.array([self.rng.uniform(margin, AREA_WIDTH - margin),
                            self.rng.uniform(margin, AREA_HEIGHT - margin)], dtype=float)
            if all(np.linalg.norm(pos - other) >= WATER_STATION_MIN_SEPARATION for other in stations):
                stations.append(pos)
        return stations if len(stations) == count else self._default_water_stations()[:count]

    def __init__(self, seed: Optional[int] = None, random_fires: bool = False, random_stations: bool = False,
                 log_collisions: bool = False):
        self.seed = RANDOM_SEED if seed is None else seed
        self.log_collisions = log_collisions

        self.rng = random.Random(self.seed)
        self.fires = self._random_fires(NUM_FIRES) if random_fires else self._default_fires()
        self.water_stations = self._random_water_stations(NUM_WATER_STATIONS) if random_stations else self._default_water_stations()

        self.drones: List[Drone] = []
        self.world = SimulationWorld(self.drones, self.fires, AREA_WIDTH, AREA_HEIGHT, self.water_stations)
        self.drones.extend(Drone(i, self.rng, self.world, self.seed) for i in range(NUM_DRONES))

        self.step_count = 0
        self.step_collisions = 0        # coppie in contatto in questo step
        self.total_collisions = 0       # eventi di collisione (una coppia che ENTRA in contatto conta 1)
        self.contact_steps = 0          # step-coppia trascorsi in contatto (durata complessiva)
        self._pairs_in_contact: Set[Tuple[int, int]] = set()
        self.last_neighbors: Dict[int, List[Drone]] = {}

    @property
    def sim_time(self) -> float:
        return self.step_count * SIM_TIME_STEP

    def step(self) -> None:
        # 1. Topologia radio all'inizio del round
        self.last_neighbors = self.world.refresh_neighbors()
        # 2. Tutti i droni pubblicano lo stesso stato logico
        for drone in self.drones:
            drone.pre_step()
        # 3. Tutti i droni elaborano il proprio step
        for drone in self.drones:
            drone.step()
        # 4. Aggiornamento ambiente fisico
        self.world.update_fires(self.rng)
        # 5. Rilevamento collisioni (statistica: non le risolve)
        self._detect_collisions()
        self.step_count += 1

    def _detect_collisions(self) -> None:
        """Rileva le collisioni fisiche per fini statistici.

        Non applica una risposta centrale: è l'avoidance locale che deve prevenirle.
        Conta gli EVENTI (ingresso in contatto) e non gli step: una sovrapposizione che dura
        50 step è una collisione, non 50.
        """
        self.step_collisions = 0
        n = len(self.drones)
        for i in range(n):
            for j in range(i + 1, n):
                a = self.drones[i]
                b = self.drones[j]
                dist = np.linalg.norm(a.position - b.position)
                pair = (a.idx, b.idx)
                if dist < DRONE_IMPACT_RADIUS:
                    self.step_collisions += 1
                    self.contact_steps += 1
                    if pair not in self._pairs_in_contact:
                        self._pairs_in_contact.add(pair)
                        self.total_collisions += 1
                        if self.log_collisions:
                            print(f"[collision] step={self.step_count} t={self.sim_time:.2f}s ids={pair} dist={dist:.4f}")
                else:
                    self._pairs_in_contact.discard(pair)

    def summary(self) -> str:
        return (f"t={self.sim_time:.1f}s  incendi attivi={len(self.world.fires)}  spenti={self.world.extinguished_count}  "
                f"collisioni={self.total_collisions}  step-contatto={self.contact_steps}")


# ------------------------------------------------------------
# Renderer (Pygame)
# ------------------------------------------------------------

def world_to_screen(pos: np.ndarray) -> Tuple[int, int]:
    """Converte coordinate mondo in pixel schermo con asse Y invertito per Pygame."""
    return int((pos[0] / AREA_WIDTH * WINDOW_WIDTH)), int((1.0 - pos[1] / AREA_HEIGHT) * WINDOW_HEIGHT)

def world_length_to_screen(length: float) -> int:
    """Converte una lunghezza in metri in pixel (scala orizzontale)."""
    return max(1, int(length / AREA_WIDTH * WINDOW_WIDTH))


class Renderer:
    def __init__(self, sim: SwarmSimulation, show_force_vectors: bool, show_communication: bool):
        import pygame  # import locale: la modalità --headless non richiede Pygame
        self.pg = pygame
        self.sim = sim
        self.show_force_vectors = show_force_vectors
        self.show_communication = show_communication
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.line_overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        pygame.display.set_caption("X-Coverage — Swarm Simulation 2D (Pygame)")
        self.font = pygame.font.SysFont("monospace", 15)

    def close(self) -> None:
        self.pg.quit()

    def _draw_transparent_circle(self, color, center, radius) -> None:
        pg = self.pg
        radius = max(1, int(radius))
        target_rect = pg.Rect(center[0] - radius, center[1] - radius, radius * 2, radius * 2)
        shape_surface = pg.Surface(target_rect.size, pg.SRCALPHA)
        pg.draw.circle(shape_surface, color, (radius, radius), radius)
        self.screen.blit(shape_surface, target_rect)

    def _draw_dashed_line(self, surface, color, p0, p1, dash_len: int, gap_len: int) -> None:
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        length = max(1.0, float(np.hypot(dx, dy)))
        steps = int(length // (dash_len + gap_len))
        for i in range(steps):
            start_t = i * (dash_len + gap_len) / length
            end_t = min(1.0, (i * (dash_len + gap_len) + dash_len) / length)
            self.pg.draw.line(surface, color,
                              (p0[0] + dx * start_t, p0[1] + dy * start_t),
                              (p0[0] + dx * end_t, p0[1] + dy * end_t), 1)

    def _draw_vector(self, start_pos: np.ndarray, vec: np.ndarray, color: Tuple[int, int, int], scale: float, weight: int = 1) -> None:
        if np.linalg.norm(vec) < 1e-8:
            return
        end_pos = start_pos + clamp_magnitude(vec, MAX_FORCE_ON_TARGET) * scale
        self.pg.draw.line(self.screen, color, world_to_screen(start_pos), world_to_screen(end_pos), weight)

    def draw_scene(self, paused: bool = False) -> None:
        pg = self.pg
        sim = self.sim
        self.screen.fill((20, 20, 25))
        self.line_overlay.fill((0, 0, 0, 0))

        # 1. Stazioni idriche (cerchio = area di servizio, tratteggio = anello di attesa)
        for station_idx, station_pos in enumerate(sim.world.water_stations):
            st_x, st_y = world_to_screen(station_pos)
            pg.draw.circle(self.screen, (30, 100, 200), (st_x, st_y), world_length_to_screen(WATER_STATION_SERVICE_RADIUS), 2)
            pg.draw.circle(self.line_overlay, (30, 100, 200, 70), (st_x, st_y), world_length_to_screen(WATER_STATION_WAIT_RADIUS), 1)
            txt = self.font.render(f"WATER {station_idx}", True, (100, 180, 255))
            self.screen.blit(txt, (st_x - 28, st_y - world_length_to_screen(WATER_STATION_SERVICE_RADIUS) - 18))

        # 2. Incendi
        for fire in sim.world.fires:
            fx, fy = world_to_screen(fire.pos)
            det_r = world_length_to_screen(FIRE_DETECTION_RADIUS)
            ext_r = world_length_to_screen(FIRE_EXTINGUISH_RADIUS)

            fire_txt = self.font.render(f"{fire.health:.0f}", True, (255, 200, 50))
            self.screen.blit(fire_txt, (fx - 10, fy - 30))

            pg.draw.circle(self.screen, (255, 140, 40), (fx, fy), det_r, 1)
            pg.draw.circle(self.screen, (200, 80, 0), (fx, fy), ext_r, 1)

            ratio = max(0.0, min(fire.health, FIRE_HEALTH) / FIRE_HEALTH)
            f_color = (int(100 + 155 * ratio), int(200 - 120 * ratio), int(100 - 100 * ratio), 50)
            self._draw_transparent_circle(f_color, (fx, fy), ext_r)

        # 3. Connessioni di comunicazione (sull'overlay: l'alpha funziona solo su superfici SRCALPHA)
        if self.show_communication:
            for drone in sim.drones:
                for other in sim.last_neighbors.get(drone.idx, []):
                    if other.idx > drone.idx:
                        pg.draw.line(self.line_overlay, COMMUNICATION_LINE_COLOR,
                                     world_to_screen(drone.position), world_to_screen(other.position), 1)

        # 4. Linee tratteggiate drone -> target e drone -> ancora
        for drone in sim.drones:
            p = world_to_screen(drone.position)
            self._draw_dashed_line(self.line_overlay, DASHED_LINE_TARGET_COLOR, p, world_to_screen(drone.target), 6, 5)
            self._draw_dashed_line(self.line_overlay, DASHED_LINE_ANCHOR_COLOR, p, world_to_screen(drone.anchor_target), 4, 5)

        self.screen.blit(self.line_overlay, (0, 0))

        # 5. Droni, target e vettori
        for drone in sim.drones:
            px, py = world_to_screen(drone.position)
            tx, ty = world_to_screen(drone.target)
            ox, oy = world_to_screen(drone.original_target)
            ax, ay = world_to_screen(drone.anchor_target)

            pg.draw.circle(self.screen, (50, 220, 80), (ox, oy), 3)
            pg.draw.circle(self.screen, (160, 160, 255), (ax, ay), 3, 1)
            pg.draw.circle(self.screen, (230, 50, 50), (tx, ty), 4)

            if self.show_force_vectors:
                self._draw_vector(drone.target, drone.compute_repulsion_between_targets(), (255, 200, 50), 0.2)
                self._draw_vector(drone.target, drone.compute_anchor_force(), (50, 255, 200), 0.2)
                self._draw_vector(drone.target, drone.compute_boundary_force(), (50, 50, 230), 0.2)
                self._draw_vector(drone.target, drone.last_applied_force, (255, 255, 255), 0.2)
                # Correzione di velocità dovuta al collision avoidance (applicata al drone, non al target)
                self._draw_vector(drone.position, drone.last_avoidance_force, (255, 80, 255), 0.5)

            if drone.is_extinguishing_fire():
                pg.draw.circle(self.screen, (255, 230, 20), (px, py), 12, 2)

            # Drone ammesso alla stazione (ha uno slot di servizio)
            if drone.reloading and drone.station_slot is not None:
                pg.draw.circle(self.screen, (20, 200, 255), (px, py), 14, 2)

            w_ratio = drone.water / DRONE_WATER_CAPACITY
            d_color = (int(255 * (1.0 - w_ratio)), int(180 * w_ratio + 50), int(255 * w_ratio))
            pg.draw.circle(self.screen, d_color, (px, py), world_length_to_screen(DRONE_IMPACT_RADIUS))

        # 6. Overlay statistiche
        status = sim.summary() + ("   [PAUSA]" if paused else "")
        self.screen.blit(self.font.render(status, True, (255, 255, 0)), (10, 10))
        help_txt = "P pausa  V vettori  C comunicazione  Q esci"
        self.screen.blit(self.font.render(help_txt, True, (150, 150, 150)), (10, 28))

        pg.display.flip()


# ------------------------------------------------------------
# CLI
# ------------------------------------------------------------

def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulazione 2D di swarm decentralizzato con Pygame")
    parser.add_argument("--show-vectors", action=argparse.BooleanOptionalAction, default=SHOW_FORCE_VECTORS,
                        help="Abilita il disegno dei vettori di forza e di collision avoidance")
    parser.add_argument("--comm", action=argparse.BooleanOptionalAction, default=SHOW_DRONES_COMMUNICATION,
                        help="Mostra le connessioni di comunicazione tra droni")
    parser.add_argument("--random-fires", action="store_true", help=f"Genera {NUM_FIRES} incendi casuali")
    parser.add_argument("--random-stations", action="store_true", help=f"Genera {NUM_WATER_STATIONS} stazioni idriche casuali")
    parser.add_argument("--log", action="store_true", help="Abilita il logging delle collisioni")
    parser.add_argument("--seed", type=int, default=RANDOM_SEED, help="Seme casuale della simulazione")
    parser.add_argument("--headless", action="store_true", help="Esegue senza finestra e stampa le statistiche")
    parser.add_argument("--steps", type=int, default=30000, help="Numero di step in modalità --headless")
    return parser


def run_headless(sim: SwarmSimulation, steps: int) -> None:
    report_every = int(round(10.0 / SIM_TIME_STEP))
    for _ in range(steps):
        sim.step()
        if sim.step_count % report_every == 0:
            print(sim.summary(), flush=True)
    print("FINE:", sim.summary())


def run_simulation(show_force_vectors: bool = SHOW_FORCE_VECTORS, show_communication: bool = SHOW_DRONES_COMMUNICATION,
                   random_fires: bool = False, random_stations: bool = False, log_collisions: bool = False,
                   seed: int = RANDOM_SEED) -> None:
    sim = SwarmSimulation(seed=seed, random_fires=random_fires, random_stations=random_stations, log_collisions=log_collisions)
    renderer = Renderer(sim, show_force_vectors, show_communication)
    pg = renderer.pg
    clock = pg.time.Clock()
    try:
        paused = False
        running = True
        while running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False
                elif event.type == pg.KEYDOWN:
                    if event.key == pg.K_q:
                        running = False
                    elif event.key == pg.K_p:
                        paused = not paused
                    elif event.key == pg.K_v:
                        renderer.show_force_vectors = not renderer.show_force_vectors
                    elif event.key == pg.K_c:
                        renderer.show_communication = not renderer.show_communication

            if not paused:
                sim.step()

            renderer.draw_scene(paused)
            clock.tick(60)
    finally:
        renderer.close()


if __name__ == "__main__":
    for problem in validate_config():
        print(f"[config] ATTENZIONE: {problem}", file=sys.stderr)

    args = build_argument_parser().parse_args()
    if args.headless:
        run_headless(SwarmSimulation(seed=args.seed, random_fires=args.random_fires,
                                     random_stations=args.random_stations, log_collisions=args.log), args.steps)
    else:
        run_simulation(
            show_force_vectors=args.show_vectors,
            show_communication=args.comm,
            random_fires=args.random_fires,
            random_stations=args.random_stations,
            log_collisions=args.log,
            seed=args.seed,
        )
