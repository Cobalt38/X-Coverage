import argparse
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import pygame

# ============================================================
# Configurazione
# ============================================================

# SIM
NUM_DRONES = 12
AREA_WIDTH = 20.0
AREA_HEIGHT = 12.0
SIM_TIME_STEP = 0.01
RANDOM_SEED = 42

WINDOW_WIDTH = 1536
WINDOW_HEIGHT = 921

SHOW_FORCE_VECTORS = False
SHOW_DRONES_COMMUNICATION = True

DASHED_LINE_TARGET_COLOR = (255, 110, 255, 100)
DASHED_LINE_ANCHOR_COLOR = (110, 250, 110, 100)

CAMERA_DISTANCE_FACTOR = 0.65

FIRE_GROWTH_RATE = 0.5


# ============================================================
# DRONI
# ============================================================

COMMUNICATION_RADIUS = 2.5

TARGET_SEPARATION = 2.0
TARGET_REACHED_DISTANCE = 0.2

MAX_TARGET_SPEED = 20.0
MAX_FORCE_ON_TARGET = 10.0

TARGET_VEL_AGING_FACTOR = 0.1

MAX_DRONE_SPEED = 1.0

ANCHOR_TO_TARGET_INTENSITY = 0.0075
K_DESIRED_VEL_TO_TARGET = 0.70

K_REPULSION_BETWEEN_TARGETS = 1.0
K_ANCHOR_DRAGGING = 0.2
K_FIRE_DRAGGING = 4.0
K_BOUNDARY_REPULSION = 1.0

FIRE_DETECTION_RADIUS = 2.0
FIRE_GENERATION_MARGIN = 1.5

WATER_STATION_GENERATION_MARGIN = 1.5
MARGIN_REPULSION_BOUNDARY = 1.5

MAX_IDLE_STEPS = 30


# ============================================================
# Collision avoidance / Safety
# ============================================================

# Collisione fisica reale.
DRONE_IMPACT_RADIUS = 0.1

# Distanza minima operativa desiderata.
AVOID_MIN_DISTANCE = 1.0

# Distanza di sicurezza dinamica:
# safety_margin = AVOID_MIN_DISTANCE + K * speed
SAFE_DISTANCE_BASE = 1.0
SAFE_DISTANCE_K_VEL = 0.6

# Campo di forza predittivo.
K_DAMPING_REPULSION = 7.2

# Smorzamento rispetto alla velocità relativa di chiusura.
K_DAMPING_DAMP = 2.2

# Limite di variazione dell'accelerazione.
MAX_JERK = 8.0

# Orizzonte temporale predittivo.
AVOID_LOOKAHEAD = 1.0

# Forza massima del campo di avoidance.
AVOID_FORCE = 2.0

# Soglia locale di emergenza.
EMERGENCY_AVOID_DISTANCE = 0.8

# Guadagno specifico della risposta di emergenza.
EMERGENCY_AVOID_GAIN = 7.0

# Evitamento fisico sempre attivo anche tra droni che
# stanno lavorando sullo stesso incendio.
FIRE_TASK_AVOIDANCE_SCALE = 0.30


# ============================================================
# Fire extinguishing & saturation
# ============================================================

FIRE_HEALTH = 200.0

FIRE_SPAWN_THRESHOLD = 1.20
FIRE_SPAWN_PROB_PER_STEP = 0.003
FIRE_SPAWN_OFFSET_MAX = 3.0
FIRE_SPAWN_INITIAL_HEALTH = 0.15

DRONE_WATER_CAPACITY = 20.0
DRONE_WATER_FLOW_RATE = 5.0

FIRE_EXTINGUISH_RADIUS = 1.2

MAX_DRONES_ON_FIRE = 3

# Questo parametro rappresenta una DISTANZA, non una forza.
FIRE_SATURATION_BOUNCE_DISTANCE = 3.5


# ============================================================
# Stazione idrica
# ============================================================

LOW_WATER_THRESHOLD = DRONE_WATER_CAPACITY * 0.1

WATER_STATION_REFILL_RATE = 10.0

NUM_WATER_STATIONS = 3
WATER_STATION_CAPACITY = 2

WATER_STATION_SERVICE_RADIUS = 0.65
WATER_STATION_WAIT_RADIUS = 2.4
WATER_STATION_MIN_SEPARATION = 2.0


# ============================================================
# Target dynamics
# ============================================================

TAU = 0.3  # Serve nel calcolo della velocità target in base alla forza totale.

# ============================================================
# PID velocity control
# ============================================================

PID_KP = 3.4
PID_KI = 0.6
PID_KD = 0.25

PID_INTEGRAL_LIMIT = 3.0
PID_MAX_OUTPUT_ACCEL = 4.0

# ============================================================
# TTL memoria incendi
# ============================================================

FIRE_MEMORY_TTL_STEPS = 100
EXTINGUISHED_FIRE_MEMORY_TTL_STEPS = FIRE_MEMORY_TTL_STEPS * 3

# Numero incendi generati con --random-fires
NUM_FIRES = 3

# ============================================================
# Utils
# ============================================================
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
    return float(v[0]), float(v[1])


def world_to_screen(pos: np.ndarray) -> Tuple[int, int]:
    """
    Converte coordinate mondo in pixel Pygame.
    L'asse Y del mondo è invertito rispetto allo schermo.
    """
    return (
        int(pos[0] / AREA_WIDTH * WINDOW_WIDTH),
        int((1.0 - pos[1] / AREA_HEIGHT) * WINDOW_HEIGHT),
    )


def world_length_to_screen(length: float) -> int:
    return max(1, int(length / AREA_WIDTH * WINDOW_WIDTH))


def draw_transparent_circle(surface, color, center, radius):
    radius = max(1, int(radius))
    target_rect = pygame.Rect(
        center[0] - radius,
        center[1] - radius,
        radius * 2,
        radius * 2,
    )
    shape_surface = pygame.Surface(
        target_rect.size,
        pygame.SRCALPHA,
    )
    pygame.draw.circle(
        shape_surface,
        color,
        (radius, radius),
        radius,
    )
    surface.blit(shape_surface, target_rect)

# ============================================================
# PID Controller
# ============================================================
class PDController:
    """
    Controllore PID 2D sulla velocità.
    L'output è un'accelerazione limitata.
    Il jerk limita la variazione di accelerazione tra due step.
    """
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
        max_output: float,
        max_jerk: float,
    ):
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

    def step(
        self,
        desired_velocity: np.ndarray,
        current_velocity: np.ndarray,
        current_acceleration: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        error = desired_velocity - current_velocity
        deriv = (error - self.last_error) / max(dt, 1e-12)
        self.integral += error * dt
        self.integral = np.clip(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )
        pid_output = self.kp * error + self.ki * self.integral + self.kd * deriv
        target_accel = clamp_magnitude(
            pid_output,
            self.max_output,
        )
        accel_error = target_accel - current_acceleration
        max_accel_change = self.max_jerk * dt
        accel_change = clamp_magnitude(
            accel_error,
            max_accel_change,
        )
        new_acceleration = current_acceleration + accel_change
        new_acceleration = clamp_magnitude(
            new_acceleration,
            self.max_output,
        )
        self.last_error = error.copy()
        return new_acceleration


# ============================================================
# Fire
# ============================================================


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
        applied = min(
            max(water, 0.0),
            self.health,
        )
        self.health -= applied
        return applied


# ============================================================
# Comunicazione
# ============================================================


@dataclass
class DroneMessage:
    position: np.ndarray
    velocity: np.ndarray
    target: np.ndarray
    reloading: bool
    water_station_idx: Optional[int]
    refuel_claim_age: int
    extinguishing: bool
    fire_target: Optional[Tuple[float, float]]

    known_fires: Dict[Tuple[float, float], int]
    extinguished_fires: Dict[Tuple[float, float], int]


class CommunicationModule:
    """
    Comunicazione locale tra agenti.

    Il doppio buffer garantisce che tutti i droni utilizzino,
    durante un round, informazioni appartenenti allo stesso
    istante logico.

    neighbors:
        messaggi già committati e utilizzabili.

    _incoming:
        messaggi ricevuti durante la fase precedente e destinati
        al prossimo round.
    """

    def __init__(self, drone: "Drone"):
        self.drone = drone
        self.neighbors: Dict[int, DroneMessage] = {}
        self._incoming: Dict[int, DroneMessage] = {}

    def begin_round(self) -> None:
        self.neighbors = self._incoming
        self._incoming = {}

    def deliver(
        self,
        sender_idx: int,
        msg: DroneMessage,
    ) -> None:
        self._incoming[sender_idx] = msg

    def merge_fire_knowledge(self) -> None:
        drone = self.drone

        # ----------------------------------------------------
        # 1. Informazioni sugli incendi confermati spenti.
        #    Hanno priorità sulle informazioni di incendio attivo.
        # ----------------------------------------------------
        for message in self.neighbors.values():
            for fire_pos, age in message.extinguished_fires.items():
                if (
                    fire_pos not in drone.extinguished_fires
                    or age < drone.extinguished_fires[fire_pos]
                ):
                    drone.extinguished_fires[fire_pos] = age

        # ----------------------------------------------------
        # 2. Propagazione degli incendi attivi.
        # ----------------------------------------------------
        for message in self.neighbors.values():
            for fire_pos, age in message.known_fires.items():
                if fire_pos in drone.extinguished_fires:
                    continue
                if (
                    fire_pos not in drone.known_fires
                    or age < drone.known_fires[fire_pos]
                ):
                    drone.known_fires[fire_pos] = age

        # ----------------------------------------------------
        # 3. Pulizia delle informazioni attive contraddette
        #    da una conferma di spegnimento.
        # ----------------------------------------------------
        for fire_pos in list(drone.known_fires):
            if fire_pos in drone.extinguished_fires:
                del drone.known_fires[fire_pos]


# ============================================================
# SimulationWorld
# ============================================================


class SimulationWorld:
    """
    Ambiente della simulazione.
    Il World mantiene lo stato fisico globale perché è il simulatore.
    Non prende decisioni operative per conto dei droni.
    """

    def __init__(
        self,
        drones: List["Drone"],
        fires: List[Fire],
        area_width: float,
        area_height: float,
        water_stations: List[np.ndarray],
    ):
        self._drones = drones
        self._fires = fires

        self.area_width = area_width
        self.area_height = area_height

        self.water_stations = water_stations

        self._neighbor_cache: Optional[Dict[int, List["Drone"]]] = None

        self.step_counter = 0

    @property
    def fires(self) -> List[Fire]:
        return self._fires

    def refresh_neighbors(self) -> Dict[int, List["Drone"]]:
        """
        Costruisce la topologia di comunicazione geometrica.
        Questo è un servizio del simulatore: il singolo drone non
        riceve la lista globale, ma solo i messaggi dei vicini.
        """
        neighbor_map: Dict[int, List["Drone"]] = {
            drone.idx: [] for drone in self._drones
        }
        for i, drone1 in enumerate(self._drones):
            for drone2 in self._drones[i + 1 :]:
                distance = np.linalg.norm(drone1.position - drone2.position)
                if distance <= COMMUNICATION_RADIUS:
                    neighbor_map[drone1.idx].append(drone2)
                    neighbor_map[drone2.idx].append(drone1)
        self._neighbor_cache = neighbor_map
        return neighbor_map

    def get_neighbors(
        self,
        drone: "Drone",
    ) -> List["Drone"]:
        if self._neighbor_cache is None:
            self.refresh_neighbors()
        return self._neighbor_cache.get(drone.idx, []) if self._neighbor_cache else []

    def sense_fires(
        self,
        position: np.ndarray,
    ) -> List[Fire]:
        return [
            fire
            for fire in self._fires
            if (
                fire.active
                and np.linalg.norm(position - fire.pos) <= FIRE_DETECTION_RADIUS
            )
        ]

    def has_active_fire_near(
        self,
        position: np.ndarray,
        radius: float,
    ) -> bool:
        return any(
            fire.active and np.linalg.norm(position - fire.pos) <= radius
            for fire in self._fires
        )

    def request_extinguish(
        self,
        position: np.ndarray,
        water_available: float,
        flow_rate: float,
    ) -> float:
        remaining = min(
            max(water_available, 0.0),
            flow_rate * SIM_TIME_STEP,
        )
        total_used = 0.0
        for fire in list(self._fires):
            if not fire.active:
                continue
            if np.linalg.norm(position - fire.pos) > FIRE_EXTINGUISH_RADIUS:
                continue
            used = fire.extinguish(remaining)
            total_used += used
            remaining -= used
            if remaining <= 1e-12:
                break
        self._remove_dead_fires()
        return total_used

    def update_fires(
        self,
        rng: random.Random,
    ) -> None:
        self.step_counter += 1
        spawn_queue: List[Fire] = []
        for fire in self._fires:
            fire.grow(SIM_TIME_STEP)
            if fire.health <= FIRE_HEALTH * FIRE_SPAWN_THRESHOLD:
                continue
            if rng.random() >= FIRE_SPAWN_PROB_PER_STEP:
                continue
            angle = rng.uniform(
                0.0,
                2.0 * np.pi,
            )
            distance = rng.uniform(
                0.5,
                FIRE_SPAWN_OFFSET_MAX,
            )
            new_pos = fire.pos + np.array(
                [
                    distance * np.cos(angle),
                    distance * np.sin(angle),
                ],
                dtype=float,
            )
            new_pos[0] = float(
                np.clip(
                    new_pos[0],
                    FIRE_GENERATION_MARGIN,
                    self.area_width - FIRE_GENERATION_MARGIN,
                )
            )
            new_pos[1] = float(
                np.clip(
                    new_pos[1],
                    FIRE_GENERATION_MARGIN,
                    self.area_height - FIRE_GENERATION_MARGIN,
                )
            )
            spawn_queue.append(
                Fire(
                    pos=new_pos,
                    health=(FIRE_HEALTH * FIRE_SPAWN_INITIAL_HEALTH),
                )
            )
        self._fires.extend(spawn_queue)
        self._remove_dead_fires()

    def _remove_dead_fires(self) -> None:
        self._fires[:] = [fire for fire in self._fires if fire.active]


# ============================================================
# Drone
# ============================================================


class Drone:
    """
    Agente autonomo.
    Il drone:
        1. riceve messaggi locali;
        2. aggiorna la memoria;
        3. percepisce l'ambiente;
        4. seleziona un compito;
        5. calcola navigazione;
        6. applica collision avoidance;
        7. controlla la velocità tramite PID;
        8. esegue l'azione fisica.
    """

    def __init__(
        self,
        idx: int,
        rng: random.Random,
        world: SimulationWorld,
    ):
        self.idx = idx
        self.world = world
        self.rng = rng
        self.position = np.array(
            [
                rng.uniform(0.0, world.area_width),
                rng.uniform(0.0, world.area_height),
            ],
            dtype=float,
        )
        self.velocity = np.zeros(2, dtype=float)
        self.acceleration = np.zeros(2, dtype=float)
        self.original_target = np.array(
            [
                rng.uniform(0.0, world.area_width),
                rng.uniform(0.0, world.area_height),
            ],
            dtype=float,
        )
        self.anchor_target = self.original_target.copy()
        self.target = self.original_target.copy()
        self.target_velocity = np.zeros(2, dtype=float)
        self.known_fires: Dict[Tuple[float, float], int] = {}
        self.extinguished_fires: Dict[Tuple[float, float], int] = {}
        self.fire_target: Optional[Tuple[float, float]] = None
        self.idle_steps = 0
        self.desired_velocity = np.zeros(
            2,
            dtype=float,
        )
        self.last_applied_force = np.zeros(
            2,
            dtype=float,
        )
        self.last_avoidance_force = np.zeros(
            2,
            dtype=float,
        )
        self.water = DRONE_WATER_CAPACITY
        self.reloading = False
        self.water_station_idx: Optional[int] = None
        self.refuel_claim_age = 0
        self.communication = CommunicationModule(self)
        self.pid = PDController(
            PID_KP,
            PID_KI,
            PID_KD,
            PID_INTEGRAL_LIMIT,
            PID_MAX_OUTPUT_ACCEL,
            MAX_JERK,
        )

    @property
    def _neighbor_messages(
        self,
    ) -> Dict[int, DroneMessage]:
        return self.communication.neighbors

    # ========================================================
    # Comunicazione
    # ========================================================

    def _build_message(self) -> DroneMessage:
        return DroneMessage(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            target=self.target.copy(),
            reloading=self.reloading,
            water_station_idx=self.water_station_idx,
            refuel_claim_age=self.refuel_claim_age,
            extinguishing=self.is_extinguishing_fire(),
            fire_target=self.fire_target,
            known_fires=dict(self.known_fires),
            extinguished_fires=dict(self.extinguished_fires),
        )

    def _deliver(self, sender_idx: int, msg: DroneMessage) -> None:
        self.communication.deliver(
            sender_idx,
            msg,
        )

    def merge_neighbor_knowledge(self) -> None:
        self.communication.merge_fire_knowledge()

    # ========================================================
    # Percezione
    # ========================================================

    def sense_environment(self) -> None:
        # ----------------------------------------------------
        # Aging memoria incendi attivi.
        # ----------------------------------------------------
        for fire_pos in list(self.known_fires):
            self.known_fires[fire_pos] += 1
            if self.known_fires[fire_pos] > FIRE_MEMORY_TTL_STEPS:
                del self.known_fires[fire_pos]

        # ----------------------------------------------------
        # Aging memoria incendi spenti.
        # ----------------------------------------------------
        for fire_pos in list(self.extinguished_fires):
            self.extinguished_fires[fire_pos] += 1
            if self.extinguished_fires[fire_pos] > EXTINGUISHED_FIRE_MEMORY_TTL_STEPS:
                del self.extinguished_fires[fire_pos]

        # ----------------------------------------------------
        # Verifica locale degli incendi precedentemente noti.
        # ----------------------------------------------------
        for fire_pos in list(self.known_fires):
            fire_arr = np.array(
                fire_pos,
                dtype=float,
            )
            if np.linalg.norm(self.position - fire_arr) <= FIRE_DETECTION_RADIUS:
                if not self.world.has_active_fire_near(
                    fire_arr,
                    0.5,
                ):
                    del self.known_fires[fire_pos]

                    self.extinguished_fires[fire_pos] = 0

                    if self.fire_target == fire_pos:
                        self.fire_target = None

        # ----------------------------------------------------
        # Rilevamento diretto.
        # ----------------------------------------------------
        for fire in self.world.sense_fires(self.position):
            pos_key = vec_to_tuple(fire.pos)

            # Una rilevazione fisica corrente ha priorità.
            self.known_fires[pos_key] = 0

            # Se un incendio torna ad essere rilevato come attivo,
            # la vecchia memoria "spento" viene rimossa.
            if pos_key in self.extinguished_fires:
                del self.extinguished_fires[pos_key]

        # ----------------------------------------------------
        # Il target incendio deve sempre esistere nella memoria.
        # ----------------------------------------------------
        if self.fire_target is not None and self.fire_target not in self.known_fires:
            self.fire_target = None

    # ========================================================
    # Fire task allocation
    # ========================================================

    def _select_fire(
        self,
    ) -> Optional[Tuple[float, float]]:

        if not self.known_fires:
            return None

        return min(
            self.known_fires,
            key=lambda p: np.linalg.norm(np.array(p) - self.position),
        )

    def _count_drones_on_fire(
        self,
        fire_pos: Tuple[float, float],
    ) -> int:
        """
        Conta localmente i droni che risultano impegnati sullo
        stesso incendio.

        Non esiste un contatore globale del task.
        """

        count = 0

        fire_arr = np.array(
            fire_pos,
            dtype=float,
        )

        for msg in self._neighbor_messages.values():

            if msg.fire_target == fire_pos:
                count += 1
                continue

            if (
                msg.extinguishing
                and np.linalg.norm(msg.position - fire_arr) <= FIRE_DETECTION_RADIUS
            ):
                count += 1

        return count

    # ========================================================
    # Target force field
    # ========================================================

    def compute_repulsion_between_targets(
        self,
        neighbors: List["Drone"],
    ) -> np.ndarray:

        force = np.zeros(2, dtype=float)

        for other in neighbors:

            delta = self.target - other.target
            distance = np.linalg.norm(delta)

            if distance >= TARGET_SEPARATION:
                continue

            if distance < 1e-6:

                position_delta = self.position - other.position

                direction = (
                    normalize(position_delta)
                    if np.linalg.norm(position_delta) > 1e-6
                    else np.array([1.0, 0.0])
                )

                force += K_REPULSION_BETWEEN_TARGETS * TARGET_SEPARATION * direction

            else:

                force += (
                    K_REPULSION_BETWEEN_TARGETS
                    * (TARGET_SEPARATION - distance)
                    * delta
                    / distance
                )

        return clamp_magnitude(
            force,
            MAX_FORCE_ON_TARGET,
        )

    def compute_boundary_force(self) -> np.ndarray:

        force = np.zeros(2, dtype=float)

        margin = MARGIN_REPULSION_BOUNDARY

        if self.target[0] < margin:
            force[0] += (
                K_BOUNDARY_REPULSION * (margin - self.target[0]) / max(margin, 1e-6)
            )

        elif self.target[0] > self.world.area_width - margin:
            force[0] -= (
                K_BOUNDARY_REPULSION
                * (self.target[0] - (self.world.area_width - margin))
                / max(margin, 1e-6)
            )

        if self.target[1] < margin:
            force[1] += (
                K_BOUNDARY_REPULSION * (margin - self.target[1]) / max(margin, 1e-6)
            )

        elif self.target[1] > self.world.area_height - margin:
            force[1] -= (
                K_BOUNDARY_REPULSION
                * (self.target[1] - (self.world.area_height - margin))
                / max(margin, 1e-6)
            )

        return force

    def compute_fire_force(self) -> np.ndarray:

        if self.fire_target is None:
            return np.zeros(2, dtype=float)

        fire = np.array(
            self.fire_target,
            dtype=float,
        )

        delta = fire - self.target
        distance = np.linalg.norm(delta)

        return K_FIRE_DRAGGING * distance * normalize(delta)

    def compute_total_force(
        self,
        neighbors: List["Drone"],
    ) -> np.ndarray:

        repulsion = self.compute_repulsion_between_targets(neighbors)

        if self.reloading:
            return repulsion

        total = (
            repulsion
            + K_ANCHOR_DRAGGING * (self.anchor_target - self.target)
            + self.compute_fire_force()
            + self.compute_boundary_force()
        )

        return clamp_magnitude(
            total,
            MAX_FORCE_ON_TARGET,
        )

    # ========================================================
    # Target dynamics
    # ========================================================

    def _sample_exploration_target(self) -> np.ndarray:
        return np.array(
            [
                self.rng.uniform(
                    0.0,
                    self.world.area_width,
                ),
                self.rng.uniform(
                    0.0,
                    self.world.area_height,
                ),
            ],
            dtype=float,
        )

    def _refresh_anchor(self) -> None:

        delta = self.target - self.anchor_target

        distance = np.linalg.norm(delta)

        if distance <= 1e-6:
            return

        alpha = min(
            1.0,
            ANCHOR_TO_TARGET_INTENSITY * SIM_TIME_STEP,
        )

        self.anchor_target += delta * alpha

    def has_reached_target(self) -> bool:
        return bool(
            np.linalg.norm(self.position - self.target) < TARGET_REACHED_DISTANCE
        )

    def _update_target_position(
        self,
        neighbors: List["Drone"],
    ) -> None:

        force = self.compute_total_force(neighbors)

        self.last_applied_force = force.copy()

        # Damping esponenziale fisicamente interpretabile.
        decay = np.exp(-TARGET_VEL_AGING_FACTOR * SIM_TIME_STEP)

        self.target_velocity = decay * self.target_velocity + force * SIM_TIME_STEP

        self.target_velocity = clamp_magnitude(
            self.target_velocity,
            MAX_TARGET_SPEED,
        )

        self.target += self.target_velocity * SIM_TIME_STEP

        self.target[0] = np.clip(
            self.target[0],
            0.0,
            self.world.area_width,
        )

        self.target[1] = np.clip(
            self.target[1],
            0.0,
            self.world.area_height,
        )

        self._refresh_anchor()

    # ========================================================
    # Collision avoidance
    # ========================================================

    def _compute_collision_avoidance_force(
        self,
    ) -> Tuple[np.ndarray, bool]:
        """
        Campo di forza predittivo decentralizzato.

        Per ogni vicino:

            relative position
                    +
            relative velocity
                    ↓
            time of closest approach
                    ↓
            predicted closest point
                    ↓
            predicted minimum distance
                    ↓
            repulsive force + damping

        Il controllo non guarda più la distanza solamente a
        t = LOOKAHEAD, ma il punto di massimo avvicinamento
        nell'intervallo [0, LOOKAHEAD].
        """

        total_force = np.zeros(
            2,
            dtype=float,
        )

        emergency_vector = np.zeros(
            2,
            dtype=float,
        )

        emergency = False

        current_speed = np.linalg.norm(self.velocity)

        safety_margin = (
            max(
                AVOID_MIN_DISTANCE,
                SAFE_DISTANCE_BASE,
            )
            + SAFE_DISTANCE_K_VEL * current_speed
        )

        for state in self._neighbor_messages.values():

            relative_position = self.position - state.position

            distance = np.linalg.norm(relative_position)

            if distance <= 1e-12:
                continue

            unit_now = normalize(relative_position)

            relative_velocity = self.velocity - state.velocity

            relative_speed_sq = np.dot(
                relative_velocity,
                relative_velocity,
            )

            # ------------------------------------------------
            # Tempo di massimo avvicinamento.
            # ------------------------------------------------
            if relative_speed_sq > 1e-12:

                time_to_closest = (
                    -np.dot(
                        relative_position,
                        relative_velocity,
                    )
                    / relative_speed_sq
                )

                time_to_closest = float(
                    np.clip(
                        time_to_closest,
                        0.0,
                        AVOID_LOOKAHEAD,
                    )
                )

            else:
                time_to_closest = 0.0

            closest_relative_position = (
                relative_position + relative_velocity * time_to_closest
            )

            closest_distance = np.linalg.norm(closest_relative_position)

            # ------------------------------------------------
            # Direzione della separazione futura.
            # ------------------------------------------------
            if closest_distance > 1e-8:
                closest_direction = normalize(closest_relative_position)
            else:
                closest_direction = unit_now

            # ------------------------------------------------
            # Velocità relativa di chiusura.
            #
            # positive = i droni si stanno avvicinando.
            # ------------------------------------------------
            closing_speed = max(
                0.0,
                -np.dot(
                    relative_velocity,
                    closest_direction,
                ),
            )

            # ------------------------------------------------
            # Intensità del campo di forza.
            #
            # Il termine principale cresce quando la distanza
            # prevista scende sotto il margine.
            # ------------------------------------------------
            if closest_distance < safety_margin:

                penetration = safety_margin - closest_distance

                normalized_penetration = penetration / max(safety_margin, 1e-6)

                repulsive_strength = K_DAMPING_REPULSION * normalized_penetration

                damping_strength = K_DAMPING_DAMP * closing_speed

                avoidance_force = closest_direction * (
                    repulsive_strength + damping_strength
                )

                # Se entrambi stanno lavorando sullo stesso
                # incendio, rilassiamo soltanto il task-level
                # avoidance, MA NON eliminiamo mai la protezione
                # fisica.
                if state.extinguishing and self.is_extinguishing_fire():
                    avoidance_force *= FIRE_TASK_AVOIDANCE_SCALE

                total_force += avoidance_force

            # ------------------------------------------------
            # Emergency layer.
            #
            # Questo livello NON viene disabilitato durante
            # l'estinzione.
            # ------------------------------------------------
            if distance < EMERGENCY_AVOID_DISTANCE:

                emergency = True

                emergency_vector += (
                    unit_now
                    * (EMERGENCY_AVOID_DISTANCE - distance)
                    * EMERGENCY_AVOID_GAIN
                )

                # Damping diretto contro la velocità di chiusura.
                local_closing_speed = max(
                    0.0,
                    -np.dot(
                        relative_velocity,
                        unit_now,
                    ),
                )

                emergency_vector += unit_now * local_closing_speed * K_DAMPING_DAMP

        total_force = clamp_magnitude(
            total_force,
            MAX_DRONE_SPEED * 3.0,
        )

        return total_force, emergency

    def _compute_desired_velocity_with_avoidance(
        self,
    ) -> np.ndarray:
        """
        Combina:

            target tracking
                  +
            predictive force field
                  +
            emergency avoidance

        e restituisce una velocità desiderata limitata.
        """

        target_velocity = K_DESIRED_VEL_TO_TARGET * (self.target - self.position)

        avoidance_force, emergency = self._compute_collision_avoidance_force()

        self.last_avoidance_force = avoidance_force.copy()

        if emergency:

            emergency_vector = np.zeros(
                2,
                dtype=float,
            )

            for state in self._neighbor_messages.values():

                relative_position = self.position - state.position

                distance = np.linalg.norm(relative_position)

                if distance <= 1e-12 or distance >= EMERGENCY_AVOID_DISTANCE:
                    continue

                unit = normalize(relative_position)

                relative_velocity = self.velocity - state.velocity

                emergency_vector += (
                    unit * (EMERGENCY_AVOID_DISTANCE - distance) * EMERGENCY_AVOID_GAIN
                )

                emergency_vector -= relative_velocity * 0.8

            desired_velocity = emergency_vector + 0.15 * (self.target - self.position)

        else:

            desired_velocity = target_velocity + avoidance_force

        return clamp_magnitude(
            desired_velocity,
            MAX_DRONE_SPEED,
        )

    # ========================================================
    # Motion integration
    # ========================================================

    def _integrate_motion(
        self,
        desired_velocity: np.ndarray,
    ) -> None:

        self.desired_velocity = clamp_magnitude(
            desired_velocity,
            MAX_DRONE_SPEED,
        )

        self.acceleration = self.pid.step(
            self.desired_velocity,
            self.velocity,
            self.acceleration,
            SIM_TIME_STEP,
        )

        self.velocity = clamp_magnitude(
            self.velocity + self.acceleration * SIM_TIME_STEP,
            MAX_DRONE_SPEED,
        )

        self.position += self.velocity * SIM_TIME_STEP

        self.position[0] = np.clip(
            self.position[0],
            0.0,
            self.world.area_width,
        )

        self.position[1] = np.clip(
            self.position[1],
            0.0,
            self.world.area_height,
        )

    # ========================================================
    # Exploration
    # ========================================================

    def _maybe_resume_exploration(self) -> None:

        if (
            not self.reloading
            and self.fire_target is None
            and not self.known_fires
            and self.has_reached_target()
        ):
            self.idle_steps += 1

            if self.idle_steps > MAX_IDLE_STEPS:

                self.original_target = self._sample_exploration_target()

                self.anchor_target = self.original_target.copy()

                self.target = self.original_target.copy()

                self.target_velocity[:] = 0.0

                self.idle_steps = 0

        else:
            self.idle_steps = 0

    # ========================================================
    # Water stations
    # ========================================================

    def _estimate_station_load(
        self,
        station_idx: int,
    ) -> int:

        load = 0

        station_pos = self.world.water_stations[station_idx]

        for state in self._neighbor_messages.values():

            if not state.reloading:
                continue

            if state.water_station_idx != station_idx:
                continue

            if (
                np.linalg.norm(state.position - station_pos)
                <= WATER_STATION_SERVICE_RADIUS
            ):
                load += 1

        if (
            self.reloading
            and self.water_station_idx == station_idx
            and self._is_in_station_service_area(station_idx)
        ):
            load += 1

        return load

    def _is_in_station_service_area(
        self,
        station_idx: int,
    ) -> bool:

        return bool(
            np.linalg.norm(self.position - self.world.water_stations[station_idx])
            <= WATER_STATION_SERVICE_RADIUS
        )

    def _select_water_station(self) -> int:
        """
        Selezione locale della stazione.

        Priorità:
            1. stazioni non sature;
            2. minor carico stimato;
            3. minor distanza.
        """

        candidates = []

        for idx, pos in enumerate(self.world.water_stations):
            load = self._estimate_station_load(idx)

            saturated = int(load >= WATER_STATION_CAPACITY)

            distance = np.linalg.norm(self.position - pos)

            candidates.append(
                (
                    saturated,
                    load,
                    distance,
                    idx,
                )
            )

        return min(candidates)[3]

    def _station_service_priority(
        self,
    ) -> Tuple[int, int]:

        candidates = [
            (
                self.refuel_claim_age,
                self.idx,
            )
        ]

        if self.water_station_idx is None:
            return candidates[0]

        station_pos = self.world.water_stations[self.water_station_idx]

        for other_idx, message in self._neighbor_messages.items():

            if not message.reloading:
                continue

            if message.water_station_idx != self.water_station_idx:
                continue

            if (
                np.linalg.norm(message.position - station_pos)
                <= WATER_STATION_WAIT_RADIUS
            ):
                candidates.append(
                    (
                        message.refuel_claim_age,
                        other_idx,
                    )
                )

        return min(candidates)

    def _can_enter_station_service_area(
        self,
    ) -> bool:

        if self.water_station_idx is None:
            return False

        station_pos = self.world.water_stations[self.water_station_idx]

        candidates = [
            (
                self.refuel_claim_age,
                self.idx,
            )
        ]

        for other_idx, message in self._neighbor_messages.items():

            if not message.reloading:
                continue

            if message.water_station_idx != self.water_station_idx:
                continue

            if (
                np.linalg.norm(message.position - station_pos)
                <= WATER_STATION_WAIT_RADIUS
            ):
                candidates.append(
                    (
                        message.refuel_claim_age,
                        other_idx,
                    )
                )

        candidates.sort()

        allowed = [idx for _, idx in candidates[:WATER_STATION_CAPACITY]]

        return self.idx in allowed

    def _maybe_start_reload(self) -> None:

        if self.reloading:
            return

        if self.is_extinguishing_fire():
            return

        if self.water > LOW_WATER_THRESHOLD:
            return

        self.reloading = True

        self.water_station_idx = self._select_water_station()

        self.refuel_claim_age = 0

        self.fire_target = None

        self.original_target = self.world.water_stations[self.water_station_idx].copy()

        self.target = self.original_target.copy()

        self.target_velocity[:] = 0.0

        self.pid.reset()

    def _move_toward_station(self) -> None:

        if self.water_station_idx is None:
            self.water_station_idx = self._select_water_station()

        self.refuel_claim_age += 1

        station_pos = self.world.water_stations[self.water_station_idx]

        priority = self._station_service_priority()

        if self._can_enter_station_service_area() and priority[1] == self.idx:
            target_pos = station_pos.copy()

        else:

            claims = [
                (
                    message.refuel_claim_age,
                    other_idx,
                )
                for other_idx, message in self._neighbor_messages.items()
                if (
                    message.reloading
                    and message.water_station_idx == self.water_station_idx
                )
            ]

            claims.append(
                (
                    self.refuel_claim_age,
                    self.idx,
                )
            )

            claims.sort()

            queue_index = next(
                i for i, (_, idx) in enumerate(claims) if idx == self.idx
            )

            angle = queue_index * 2.399963 + self.water_station_idx * 1.047197551

            target_pos = station_pos + np.array(
                [
                    np.cos(angle),
                    np.sin(angle),
                ],
                dtype=float,
            ) * (WATER_STATION_WAIT_RADIUS + 0.35 * queue_index)

            target_pos[0] = np.clip(
                target_pos[0],
                0.0,
                self.world.area_width,
            )

            target_pos[1] = np.clip(
                target_pos[1],
                0.0,
                self.world.area_height,
            )

        self.target = target_pos
        self.target_velocity[:] = 0.0

        if (
            self._can_enter_station_service_area()
            and priority[1] == self.idx
            and self.has_reached_target()
        ):
            self.velocity[:] = 0.0
            self.acceleration[:] = 0.0
            self.desired_velocity[:] = 0.0
            self.pid.reset()
            return

        self._integrate_motion(self._compute_desired_velocity_with_avoidance())

    # ========================================================
    # Fire interaction
    # ========================================================

    def is_extinguishing_fire(self) -> bool:
        return bool(
            self.water > 0.0
            and self.world.has_active_fire_near(
                self.position,
                FIRE_EXTINGUISH_RADIUS,
            )
        )

    def try_extinguish(self) -> None:

        if self.water <= 0.0:
            return

        # Memorizziamo prima gli incendi potenzialmente interessati.
        nearby_known_fires = [
            fire_pos
            for fire_pos in self.known_fires
            if (
                np.linalg.norm(
                    self.position
                    - np.array(
                        fire_pos,
                        dtype=float,
                    )
                )
                <= FIRE_EXTINGUISH_RADIUS * 1.25
            )
        ]

        used = self.world.request_extinguish(
            self.position,
            self.water,
            DRONE_WATER_FLOW_RATE,
        )

        self.water -= used

        if used <= 1e-9:
            return

        # ----------------------------------------------------
        # Importante:
        # il drone che ha effettivamente osservato l'azione
        # registra localmente l'incendio come spento.
        # Questo permette alla conoscenza "extinguished" di
        # propagarsi decentralmente agli altri agenti.
        # ----------------------------------------------------
        for fire_pos in nearby_known_fires:

            fire_arr = np.array(
                fire_pos,
                dtype=float,
            )

            if not self.world.has_active_fire_near(
                fire_arr,
                0.5,
            ):
                self.known_fires.pop(
                    fire_pos,
                    None,
                )

                self.extinguished_fires[fire_pos] = 0

                if self.fire_target == fire_pos:
                    self.fire_target = None

    def try_reload(self) -> None:

        if not self.reloading or not self.has_reached_target():
            return

        self.water = min(
            DRONE_WATER_CAPACITY,
            self.water + WATER_STATION_REFILL_RATE * SIM_TIME_STEP,
        )

        if self.water >= DRONE_WATER_CAPACITY - 1e-6:
            self.water = DRONE_WATER_CAPACITY

            self.reloading = False
            self.water_station_idx = None
            self.refuel_claim_age = 0

            self.target_velocity[:] = 0.0
            self.fire_target = None

            self.target = self._sample_exploration_target()

            self.original_target = self.target.copy()

            self.anchor_target = self.target.copy()

            self.pid.reset()

    # ========================================================
    # Decision making
    # ========================================================

    def decide_and_move(
        self,
        neighbors: List["Drone"],
    ) -> None:

        self._maybe_start_reload()

        if self.reloading:
            self._move_toward_station()
            return

        # ----------------------------------------------------
        # 1. Selezione incendio conosciuto.
        # ----------------------------------------------------
        selected_fire = self._select_fire()

        if selected_fire is not None:

            active_drones = self._count_drones_on_fire(selected_fire)

            fire_arr = np.array(
                selected_fire,
                dtype=float,
            )

            distance_to_fire = np.linalg.norm(self.position - fire_arr)

            # ------------------------------------------------
            # 2. Incendio saturo.
            #
            # Il bounce viene usato solo quando il drone
            # NON fa già parte del gruppo che lavora sul fuoco.
            # ------------------------------------------------
            if (
                active_drones >= MAX_DRONES_ON_FIRE
                and self.fire_target != selected_fire
            ):

                if distance_to_fire <= FIRE_DETECTION_RADIUS:

                    bounce_dir = normalize(self.position - fire_arr)

                    if np.linalg.norm(bounce_dir) < 1e-6:

                        bounce_dir = (
                            normalize(self.velocity)
                            if np.linalg.norm(self.velocity) > 1e-6
                            else np.array([1.0, 0.0])
                        )

                    self.target = (
                        self.position + bounce_dir * FIRE_SATURATION_BOUNCE_DISTANCE
                    )

                    self.target[0] = np.clip(
                        self.target[0],
                        0.0,
                        self.world.area_width,
                    )

                    self.target[1] = np.clip(
                        self.target[1],
                        0.0,
                        self.world.area_height,
                    )

                    self.anchor_target = self.target.copy()

                    self.target_velocity[:] = 0.0
                    self.fire_target = None

            else:

                # --------------------------------------------
                # 3. Incendio disponibile.
                # --------------------------------------------
                self.fire_target = selected_fire

                if distance_to_fire <= FIRE_EXTINGUISH_RADIUS:
                    self.target = self.position.copy()

                    self.anchor_target = self.position.copy()

                    self.target_velocity[:] = 0.0

                else:
                    self.target = fire_arr.copy()
                    self.original_target = fire_arr.copy()

        # ----------------------------------------------------
        # 4. Target dynamics.
        # ----------------------------------------------------
        self._update_target_position(neighbors)

        # ----------------------------------------------------
        # 5. Collision avoidance + PID + movimento.
        # ----------------------------------------------------
        self._integrate_motion(self._compute_desired_velocity_with_avoidance())

        # ----------------------------------------------------
        # 6. Se abbiamo completato un task di esplorazione,
        #    scegliamo un nuovo target.
        # ----------------------------------------------------
        self._maybe_resume_exploration()

    # ========================================================
    # Simulation step
    # ========================================================

    def pre_step(self) -> None:
        """
        Costruisce il messaggio corrente e lo invia ai vicini.

        Tutti i messaggi vengono ricevuti nel buffer del round
        successivo.
        """

        msg = self._build_message()

        for neighbor in self.world.get_neighbors(self):
            neighbor._deliver(
                self.idx,
                msg,
            )

    def step(self) -> None:

        # ----------------------------------------------------
        # 1. Commit dei messaggi ricevuti.
        # ----------------------------------------------------
        self.communication.begin_round()

        # ----------------------------------------------------
        # 2. Percezione locale.
        # ----------------------------------------------------
        self.sense_environment()

        # ----------------------------------------------------
        # 3. Fusione conoscenza distribuita.
        # ----------------------------------------------------
        self.merge_neighbor_knowledge()

        # ----------------------------------------------------
        # 4. Decisione autonoma + movimento.
        # ----------------------------------------------------
        neighbors = self.world.get_neighbors(self)

        self.decide_and_move(neighbors)

        # ----------------------------------------------------
        # 5. Azione fisica.
        # ----------------------------------------------------
        self.try_extinguish()

        # ----------------------------------------------------
        # 6. Rifornimento.
        # ----------------------------------------------------
        self.try_reload()


# ============================================================
# SwarmSimulation
# ============================================================


class SwarmSimulation:

    @staticmethod
    def _default_fires() -> List[Fire]:

        positions = [
            (
                AREA_WIDTH * 0.20,
                AREA_HEIGHT * 0.75,
            ),
            (
                AREA_WIDTH * 0.50,
                AREA_HEIGHT * 0.25,
            ),
            (
                AREA_WIDTH * 0.80,
                AREA_HEIGHT * 0.75,
            ),
        ]

        return [
            Fire(
                np.array(
                    pos,
                    dtype=float,
                )
            )
            for pos in positions
        ]

    def _random_fires(
        self,
        count: int,
    ) -> List[Fire]:

        margin = FIRE_GENERATION_MARGIN

        return [
            Fire(
                np.array(
                    [
                        self.rng.uniform(
                            margin,
                            AREA_WIDTH - margin,
                        ),
                        self.rng.uniform(
                            margin,
                            AREA_HEIGHT - margin,
                        ),
                    ],
                    dtype=float,
                )
            )
            for _ in range(count)
        ]

    @staticmethod
    def _default_water_stations() -> List[np.ndarray]:

        positions = [
            (
                AREA_WIDTH * 0.20,
                AREA_HEIGHT * 0.25,
            ),
            (
                AREA_WIDTH * 0.50,
                AREA_HEIGHT * 0.75,
            ),
            (
                AREA_WIDTH * 0.80,
                AREA_HEIGHT * 0.25,
            ),
        ]

        return [
            np.array(
                pos,
                dtype=float,
            )
            for pos in positions[:NUM_WATER_STATIONS]
        ]

    def _random_water_stations(
        self,
        count: int,
    ) -> List[np.ndarray]:

        margin = WATER_STATION_GENERATION_MARGIN

        stations: List[np.ndarray] = []

        for _ in range(count * 100):

            if len(stations) >= count:
                break

            pos = np.array(
                [
                    self.rng.uniform(
                        margin,
                        AREA_WIDTH - margin,
                    ),
                    self.rng.uniform(
                        margin,
                        AREA_HEIGHT - margin,
                    ),
                ],
                dtype=float,
            )

            if all(
                np.linalg.norm(pos - other) >= WATER_STATION_MIN_SEPARATION
                for other in stations
            ):
                stations.append(pos)

        if len(stations) == count:
            return stations

        return self._default_water_stations()[:count]

    def __init__(
        self,
        show_force_vectors: bool = False,
        show_communication: bool = True,
        random_fires: bool = False,
        random_stations: bool = False,
        log_collisions: bool = False,
    ):

        self.show_force_vectors = show_force_vectors

        self.show_communication = show_communication

        self.log_collisions = log_collisions

        self.rng = random.Random(RANDOM_SEED)

        self.fires = (
            self._random_fires(NUM_FIRES) if random_fires else self._default_fires()
        )

        self.water_stations = (
            self._random_water_stations(NUM_WATER_STATIONS)
            if random_stations
            else self._default_water_stations()
        )

        self.drones: List[Drone] = []

        self.world = SimulationWorld(
            self.drones,
            self.fires,
            AREA_WIDTH,
            AREA_HEIGHT,
            self.water_stations,
        )

        self.drones.extend(
            Drone(
                i,
                self.rng,
                self.world,
            )
            for i in range(NUM_DRONES)
        )

        self.total_collisions = 0
        self.step_collisions = 0

        self._last_neighbors: Dict[
            int,
            List[Drone],
        ] = {}

        pygame.init()
        pygame.font.init()

        self.screen = pygame.display.set_mode(
            (
                WINDOW_WIDTH,
                WINDOW_HEIGHT,
            )
        )

        self.line_overlay = pygame.Surface(
            (
                WINDOW_WIDTH,
                WINDOW_HEIGHT,
            ),
            pygame.SRCALPHA,
        )

        pygame.display.set_caption("Swarm Simulation 2D (Pygame)")

        self.font = pygame.font.SysFont(
            "monospace",
            15,
        )

    def close(self):
        pygame.quit()

    def step(
        self,
        step_index: Optional[int] = None,
    ) -> None:

        # ----------------------------------------------------
        # 1. Topologia di comunicazione all'inizio del round.
        # ----------------------------------------------------
        self._last_neighbors = self.world.refresh_neighbors()

        # ----------------------------------------------------
        # 2. Tutti i droni pubblicano lo stesso stato logico.
        # ----------------------------------------------------
        for drone in self.drones:
            drone.pre_step()

        # ----------------------------------------------------
        # 3. Tutti i droni elaborano il proprio step.
        # ----------------------------------------------------
        for drone in self.drones:
            drone.step()

        # ----------------------------------------------------
        # 4. Aggiornamento ambiente fisico.
        # ----------------------------------------------------
        self.world.update_fires(self.rng)

        # ----------------------------------------------------
        # 5. Rilevamento collisioni.
        #
        # Questo NON risolve la collisione.
        # È esclusivamente un detector/statistica.
        # ----------------------------------------------------
        self.step_collisions = 0

        self._detect_collisions(step_index)

    def _detect_collisions(
        self,
        step_index: Optional[int] = None,
    ) -> None:
        """
        Rileva collisioni fisiche.

        Non applica una risposta centrale.
        L'avoidance deve prevenire l'evento prima che avvenga.
        """

        n = len(self.drones)

        for i in range(n):

            for j in range(i + 1, n):

                a = self.drones[i]
                b = self.drones[j]

                distance = np.linalg.norm(a.position - b.position)

                if distance < DRONE_IMPACT_RADIUS:

                    self.step_collisions += 1
                    self.total_collisions += 1

                    if self.log_collisions:
                        print(
                            f"[collision] "
                            f"step={step_index} "
                            f"ids=({a.idx},{b.idx}) "
                            f"dist={distance:.4f}"
                        )

    # ========================================================
    # Rendering
    # ========================================================

    def draw_scene(self) -> None:

        self.screen.fill((20, 20, 25))

        # ----------------------------------------------------
        # 1. Stazioni idriche
        # ----------------------------------------------------
        for station_idx, station_pos in enumerate(self.world.water_stations):

            st_x, st_y = world_to_screen(station_pos)

            pygame.draw.circle(
                self.screen,
                (30, 100, 200),
                (st_x, st_y),
                18,
                2,
            )

            txt = self.font.render(
                f"WATER {station_idx}",
                True,
                (100, 180, 255),
            )

            self.screen.blit(
                txt,
                (
                    st_x - 28,
                    st_y - 30,
                ),
            )

        # ----------------------------------------------------
        # 2. Incendi
        # ----------------------------------------------------
        for fire in self.fires:

            fx, fy = world_to_screen(fire.pos)

            det_r = int(FIRE_DETECTION_RADIUS / AREA_WIDTH * WINDOW_WIDTH)

            ext_r = int(FIRE_EXTINGUISH_RADIUS / AREA_WIDTH * WINDOW_WIDTH)

            fire_txt = self.font.render(
                f"{fire.health:.0f}",
                True,
                (255, 200, 50),
            )

            self.screen.blit(
                fire_txt,
                (
                    fx - 10,
                    fy - 30,
                ),
            )

            pygame.draw.circle(
                self.screen,
                (255, 140, 40),
                (fx, fy),
                det_r,
                1,
            )

            pygame.draw.circle(
                self.screen,
                (200, 80, 0),
                (fx, fy),
                ext_r,
                1,
            )

            ratio = max(
                0.0,
                min(
                    fire.health,
                    FIRE_HEALTH,
                )
                / FIRE_HEALTH,
            )

            fire_color = (
                int(100 + 155 * ratio),
                int(200 - 120 * ratio),
                int(100 - 100 * ratio),
                50,
            )

            draw_transparent_circle(
                self.screen,
                fire_color,
                (fx, fy),
                ext_r,
            )

        # ----------------------------------------------------
        # 3. Connessioni di comunicazione
        # ----------------------------------------------------
        if self.show_communication:

            for drone in self.drones:

                for other in self._last_neighbors.get(
                    drone.idx,
                    [],
                ):

                    if other.idx <= drone.idx:
                        continue

                    p0 = world_to_screen(drone.position)

                    p1 = world_to_screen(other.position)

                    pygame.draw.line(
                        self.screen,
                        (100, 160, 220, 50),
                        p0,
                        p1,
                        1,
                    )

        # ----------------------------------------------------
        # 4. Linee tratteggiate target / anchor
        # ----------------------------------------------------
        self.line_overlay.fill((0, 0, 0, 0))

        for drone in self.drones:

            px, py = world_to_screen(drone.position)

            tx, ty = world_to_screen(drone.target)

            ax, ay = world_to_screen(drone.anchor_target)

            # Target
            dx = tx - px
            dy = ty - py

            length = max(
                1.0,
                np.hypot(dx, dy),
            )

            dash_len = 6
            gap_len = 5

            steps = int(length // (dash_len + gap_len))

            for i in range(steps):

                start_t = i * (dash_len + gap_len) / length

                end_t = min(
                    1.0,
                    (i * (dash_len + gap_len) + dash_len) / length,
                )

                x0 = px + dx * start_t
                y0 = py + dy * start_t

                x1 = px + dx * end_t
                y1 = py + dy * end_t

                pygame.draw.line(
                    self.line_overlay,
                    DASHED_LINE_TARGET_COLOR,
                    (x0, y0),
                    (x1, y1),
                    1,
                )

            # Anchor
            dx = ax - px
            dy = ay - py

            length = max(
                1.0,
                np.hypot(dx, dy),
            )

            dash_len = 4
            gap_len = 5

            steps = int(length // (dash_len + gap_len))

            for i in range(steps):

                start_t = i * (dash_len + gap_len) / length

                end_t = min(
                    1.0,
                    (i * (dash_len + gap_len) + dash_len) / length,
                )

                x0 = px + dx * start_t
                y0 = py + dy * start_t

                x1 = px + dx * end_t
                y1 = py + dy * end_t

                pygame.draw.line(
                    self.line_overlay,
                    DASHED_LINE_ANCHOR_COLOR,
                    (x0, y0),
                    (x1, y1),
                    1,
                )

        self.screen.blit(
            self.line_overlay,
            (0, 0),
        )

        # ----------------------------------------------------
        # 5. Droni, target e vettori
        # ----------------------------------------------------
        for drone in self.drones:

            px, py = world_to_screen(drone.position)

            tx, ty = world_to_screen(drone.target)

            ox, oy = world_to_screen(drone.original_target)

            ax, ay = world_to_screen(drone.anchor_target)

            pygame.draw.circle(
                self.screen,
                (50, 220, 80),
                (ox, oy),
                3,
            )

            pygame.draw.circle(
                self.screen,
                (160, 160, 255),
                (ax, ay),
                3,
                1,
            )

            pygame.draw.circle(
                self.screen,
                (230, 50, 50),
                (tx, ty),
                4,
            )

            if self.show_force_vectors:

                neighs = self._last_neighbors.get(
                    drone.idx,
                    [],
                )

                self._draw_vector(
                    drone.target,
                    drone.compute_repulsion_between_targets(neighs),
                    (255, 200, 50),
                    0.2,
                )

                self._draw_vector(
                    drone.target,
                    K_ANCHOR_DRAGGING * (drone.anchor_target - drone.target),
                    (50, 255, 200),
                    0.2,
                )

                self._draw_vector(
                    drone.target,
                    drone.compute_fire_force(),
                    (255, 128, 0),
                    0.2,
                )

                self._draw_vector(
                    drone.target,
                    drone.compute_boundary_force(),
                    (50, 50, 230),
                    0.2,
                )

                # Campo di forza collision avoidance.
                self._draw_vector(
                    drone.position,
                    drone.last_avoidance_force,
                    (255, 80, 255),
                    0.5,
                )

                self._draw_vector(
                    drone.target,
                    drone.last_applied_force,
                    (255, 255, 255),
                    0.2,
                )

            # ------------------------------------------------
            # Drone impegnato nello spegnimento.
            # ------------------------------------------------
            if drone.is_extinguishing_fire():

                pygame.draw.circle(
                    self.screen,
                    (255, 230, 20),
                    (px, py),
                    12,
                    2,
                )

            # ------------------------------------------------
            # Drone con accesso alla stazione.
            # ------------------------------------------------
            if (
                drone.reloading
                and drone.water_station_idx is not None
                and drone._can_enter_station_service_area()
                and drone._station_service_priority()[1] == drone.idx
            ):

                pygame.draw.circle(
                    self.screen,
                    (20, 200, 255),
                    (px, py),
                    14,
                    2,
                )

            # ------------------------------------------------
            # Colore drone in funzione dell'acqua.
            # ------------------------------------------------
            water_ratio = drone.water / DRONE_WATER_CAPACITY

            drone_color = (
                int(255 * (1.0 - water_ratio)),
                int(180 * water_ratio + 50),
                int(255 * water_ratio),
            )

            drone_visual_radius = world_length_to_screen(DRONE_IMPACT_RADIUS)

            pygame.draw.circle(
                self.screen,
                drone_color,
                (px, py),
                drone_visual_radius,
            )

        # ----------------------------------------------------
        # 6. Overlay collisioni
        # ----------------------------------------------------
        collision_text = (
            f"collisions_step="
            f"{self.step_collisions} "
            f"total="
            f"{self.total_collisions}"
        )

        rendered = self.font.render(
            collision_text,
            True,
            (255, 255, 0),
        )

        self.screen.blit(
            rendered,
            (10, 10),
        )

        pygame.display.flip()

    def _draw_vector(
        self,
        start_pos: np.ndarray,
        vec: np.ndarray,
        color: Tuple[int, int, int],
        scale: float,
        weight: int = 1,
    ) -> None:

        if np.linalg.norm(vec) < 1e-8:
            return

        end_pos = (
            start_pos
            + clamp_magnitude(
                vec,
                MAX_FORCE_ON_TARGET,
            )
            * scale
        )

        p0 = world_to_screen(start_pos)

        p1 = world_to_screen(end_pos)

        pygame.draw.line(
            self.screen,
            color,
            p0,
            p1,
            weight,
        )


# ============================================================
# CLI
# ============================================================
def build_argument_parser() -> argparse.ArgumentParser:

    parser = argparse.ArgumentParser(
        description=("Simulazione 2D di swarm " "decentralizzato con Pygame")
    )
    parser.add_argument(
        "--show-vectors",
        action="store_true",
        help=("Abilita il disegno dei vettori " "di forza e collision avoidance"),
    )
    parser.add_argument(
        "--comm",
        action="store_true",
        help=("Mostra le connessioni di " "comunicazione tra droni"),
    )
    parser.add_argument(
        "--random-fires",
        action="store_true",
        help=(f"Genera {NUM_FIRES} incendi casuali"),
    )
    parser.add_argument(
        "--random-stations",
        action="store_true",
        help=(f"Genera {NUM_WATER_STATIONS} " "stazioni idriche casuali"),
    )
    parser.add_argument(
        "--log",
        action="store_true",
        help=("Abilita il logging delle collisioni"),
    )
    return parser


# ============================================================
# Run simulation
# ============================================================


def run_simulation(
    show_force_vectors: bool = False,
    show_communication: bool = True,
    random_fires: bool = False,
    random_stations: bool = False,
    log_collisions: bool = False,
) -> None:

    sim = SwarmSimulation(
        show_force_vectors=show_force_vectors,
        show_communication=show_communication,
        random_fires=random_fires,
        random_stations=random_stations,
        log_collisions=log_collisions,
    )

    clock = pygame.time.Clock()

    try:

        step_count = 0
        paused = False
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        running = False
                    elif event.key == pygame.K_p:
                        paused = not paused
                    elif event.key == pygame.K_v:
                        sim.show_force_vectors = not sim.show_force_vectors
            if not paused:
                sim.step(step_count)
                step_count += 1
            sim.draw_scene()
            clock.tick(60)

    finally:
        sim.close()


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":

    parser = build_argument_parser()

    args = parser.parse_args()

    run_simulation(
        show_force_vectors=args.show_vectors,
        show_communication=args.comm,
        random_fires=args.random_fires,
        random_stations=args.random_stations,
        log_collisions=args.log,
    )
