import argparse
import os
import random
import sys
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import pygame

# ------------------------------------------------------------
# Configurazione 
# ------------------------------------------------------------

# SIM
NUM_DRONES = 12
AREA_WIDTH = 20.0
AREA_HEIGHT = 12.0
SIM_TIME_STEP = 0.01
RANDOM_SEED = 42

WINDOW_WIDTH = 1536 # finestra Pygame
WINDOW_HEIGHT = 921  # aspect ratio 20:12

SHOW_FORCE_VECTORS = False
SHOW_DRONES_COMMUNICATION = True 
# Stile linee tratteggiate (RGBA con canale alpha per l'effetto sfumato)
DASHED_LINE_TARGET_COLOR = (255, 110, 255, 100)
DASHED_LINE_ANCHOR_COLOR = (110, 250, 110, 100)
# Stability check: if the average target movement over `STABILITY_WINDOW` steps
# falls below `STABILITY_THRESHOLD`, stop early.
STABILITY_WINDOW = 60
STABILITY_THRESHOLD = 1e-3
CAMERA_DISTANCE_FACTOR = 0.65 # Smaller -> more zoom

FIRE_GROWTH_RATE = 0.5  # Health points per second, for active fires

#DRONI
COMMUNICATION_RADIUS = 2.5
TARGET_SEPARATION = 2.0
TARGET_REACHED_DISTANCE = 0.2
MAX_TARGET_SPEED = 20.0
MAX_FORCE_ON_TARGET = 10.0
TARGET_VEL_AGING_FACTOR = 0.1
MAX_DRONE_SPEED = 1.0
ANCHOR_TO_TARGET_INTENSITY = 0.0075  # l'ancora segue lentamente il target

K_DESIRED_VEL_TO_TARGET = 0.70
K_REPULSION_BETWEEN_TARGETS = 1.0 
K_ANCHOR_DRAGGING = 0.2
K_FIRE_DRAGGING = 4.0
K_BOUNDARY_REPULSION = 1.0

FIRE_DETECTION_RADIUS = 2.0
FIRE_GENERATION_MARGIN = 1.5
WATER_STATION_GENERATION_MARGIN = 1.5
MARGIN_REPULSION_BOUNDARY = 1.5 # distanza minima dal bordo della mappa a cui il target inizia a essere respinto verso l'interno.

MAX_IDLE_STEPS = 30  # Se il drone è fermo da più di questo numero di passi e non ha fuochi noti, riprende l'esplorazione.

# Collision avoidance & Safety parameters
DRONE_IMPACT_RADIUS = 0.1       # Soglia reale sotto la quale due droni collidono fisicamente
SAFE_DISTANCE_BASE = 2.4        # Distanza minima di sicurezza tra droni (ellisse di sicurezza)
SAFE_DISTANCE_K_VEL = 0.6       # Moltiplicatore per allungare l'ellisse di sicurezza lungo la velocità
K_DAMPING_REPULSION = 7.2       # Guadagno repulsivo del campo potenziale
K_DAMPING_DAMP = 2.2            # Smorzamento della velocità relativa nel campo potenziale
MAX_JERK = 8.0                  # Massima variazione dell'accelerazione nel tempo (m/s^3)

AVOID_LOOKAHEAD = 2.0           # Seconds to look ahead when predicting collisions
AVOID_MIN_DISTANCE = 1.0        # Desired minimum separation (meters)
AVOID_FORCE = 2.0               # Scaling of avoidance steering
EMERGENCY_AVOID_DISTANCE = 1.0  # Hard local threshold for evasive override

# Fire extinguishing 
FIRE_HEALTH = 100.0
DRONE_WATER_CAPACITY = 20.0
DRONE_WATER_FLOW_RATE = 5.0 
FIRE_EXTINGUISH_RADIUS = 1.2
MAX_DRONES_ON_FIRE = 3          # Numero massimo di droni che possono presidiare/estinguere lo stesso incendio contemporaneamente
WAIT_DISTANCE_FROM_FIRE = 1.5   # Distanza a cui i droni in eccesso attendono il proprio turno attorno a un incendio già presidiato

# Stazione idrica / rifornimento
WATER_STATION_POS = np.array([AREA_WIDTH * 0.5, AREA_HEIGHT * 0.5], dtype=float)
LOW_WATER_THRESHOLD = DRONE_WATER_CAPACITY * 0.1
WATER_STATION_REFILL_RATE = 10.0
NUM_WATER_STATIONS = 3
WATER_STATION_CAPACITY = 2
WATER_STATION_SERVICE_RADIUS = 0.65
WATER_STATION_WAIT_RADIUS = 2.4
WATER_STATION_MIN_SEPARATION = 2.0

TAU = 0.3 # per calcolare la velocità desiderata in base alla distanza dal target, prima di applicare il PID
# PID, velocity control
PID_KP = 3.4
PID_KI = 0.6
PID_KD = 0.25
PID_INTEGRAL_LIMIT = 3.0
PID_MAX_OUTPUT_ACCEL = 4.0

# Un fuoco non riconfermato da osservazione o comunicazione per più di FIRE_MEMORY_TTL_STEPS passi viene dimenticato.
FIRE_MEMORY_TTL_STEPS = 100

# Un fuoco spento resta "ricordato come spento" (per impedire che una voce non aggiornata di un vicino lo
# faccia risultare di nuovo attivo) per questo numero di passi.
EXTINGUISHED_FIRE_MEMORY_TTL_STEPS = FIRE_MEMORY_TTL_STEPS * 3

# Numero di incendi generati quando --random-fires è attivo.
NUM_FIRES = 3

# Condizioni di terminazione
STABILITY_MIN_STEPS_BEFORE_STOP = STABILITY_WINDOW  # non valutare la stabilità prima di questo numero di passi

# ------------------------------------------------------------
# Utils
# ------------------------------------------------------------

def clamp_magnitude(vector: np.ndarray, limit: float) -> np.ndarray:
    # Clamps the magnitude of a vector to a maximum limit, preserving its direction.
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

def world_to_screen(pos: np.ndarray) -> Tuple[int, int]:
    """Converte le coordinate del mondo (0..AREA_WIDTH, 0..AREA_HEIGHT) in coordinate pixel dello schermo (con asse Y invertito per Pygame)."""
    return int((pos[0] / AREA_WIDTH * WINDOW_WIDTH)), int((1.0 - pos[1] / AREA_HEIGHT) * WINDOW_HEIGHT)

def world_length_to_screen(length: float) -> int:
    """Converte una lunghezza espressa in metri (mondo) in pixel, usando la stessa scala orizzontale di world_to_screen."""
    return max(1, int(length / AREA_WIDTH * WINDOW_WIDTH))

def draw_transparent_circle(surface, color, center, radius):
    target_rect = pygame.Rect(center[0] - radius, center[1] - radius, radius * 2, radius * 2)
    shape_surface = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    
    pygame.draw.circle(shape_surface, color, (radius, radius), radius)
    
    surface.blit(shape_surface, target_rect)


class PDController:
    """Controllore PID generico su vettori 2D, con limitazione di integrale, output e jerk."""

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
        """Calcola la nuova accelerazione a partire dall'errore di velocità, con limitazione del jerk."""
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


@dataclass
class DroneMessage:
    position: np.ndarray
    velocity: np.ndarray
    target: np.ndarray
    reloading: bool
    water_station_idx: Optional[int]
    refuel_claim_age: int
    fire_claim: Optional[Tuple[float, float]]
    known_fires: Dict[Tuple[float, float], int]
    extinguished_fires: Dict[Tuple[float, float], int]


class CommunicationModule:
    """Memoria locale dei messaggi ricevuti dai droni vicini."""

    def __init__(self, drone: 'Drone'):
        self.drone = drone
        self.neighbors: Dict[int, DroneMessage] = {}

    def communicate(self, neighbors: List['Drone']) -> None:
        self.neighbors = {
            other.idx: DroneMessage(
                position=other.position.copy(),
                velocity=other.velocity.copy(),
                target=other.target.copy(),
                reloading=other.reloading,
                water_station_idx=other.water_station_idx,
                refuel_claim_age=other.refuel_claim_age,
                fire_claim=other.fire_claim,
                known_fires=dict(other.known_fires),
                extinguished_fires=dict(other.extinguished_fires),
            )
            for other in neighbors
        }

    def merge_fire_knowledge(self) -> None:
        drone = self.drone

        # 1. Le informazioni "incendio spento" hanno sempre priorità: si propagano come un'informazione
        #    a sé stante (stesso schema a "età minima" usato per gli incendi attivi) così che un drone che
        #    non ha mai visto di persona lo spegnimento possa comunque saperlo da un vicino.
        for message in self.neighbors.values():
            for fire_pos, age in message.extinguished_fires.items():
                if fire_pos not in drone.extinguished_fires or age < drone.extinguished_fires[fire_pos]:
                    drone.extinguished_fires[fire_pos] = age

        # 2. Le informazioni sugli incendi attivi si propagano normalmente, ma solo se quella posizione
        #    non risulta già spenta: questo impedisce a una voce "vecchia" di un vicino di far tornare
        #    in vita un incendio che è già stato confermato estinto altrove nello sciame.
        for message in self.neighbors.values():
            for fire_pos, age in message.known_fires.items():
                if fire_pos in drone.extinguished_fires:
                    continue
                if fire_pos not in drone.known_fires or age < drone.known_fires[fire_pos]:
                    drone.known_fires[fire_pos] = age

        # 3. Ripulisci comunque la memoria "attivo" da tutto ciò che ora risulta spento (anche se era già
        #    presente da prima di ricevere la conferma di spegnimento).
        for fire_pos in list(drone.known_fires):
            if fire_pos in drone.extinguished_fires:
                del drone.known_fires[fire_pos]
                if drone.fire_claim == fire_pos:
                    drone.fire_claim = None


class SimulationWorld:
    """Ambiente: geometria, sensori simulati e azioni fisiche richieste dai droni."""

    def __init__(self, drones: List['Drone'], fires: List[Fire], area_width: float, area_height: float,
                 water_stations: List[np.ndarray]):
        self._drones = drones
        self._fires = fires
        self.area_width = area_width
        self.area_height = area_height
        self.water_stations = water_stations
        self._neighbor_cache: Optional[Dict[int, List['Drone']]] = None

    @property
    def fires(self) -> List[Fire]:
        return self._fires

    def refresh_neighbors(self) -> Dict[int, List['Drone']]:
        neighbor_map: Dict[int, List['Drone']] = {d.idx: [] for d in self._drones}
        for i, drone1 in enumerate(self._drones):
            for drone2 in self._drones[i + 1:]:
                if np.linalg.norm(drone1.position - drone2.position) <= COMMUNICATION_RADIUS:
                    neighbor_map[drone1.idx].append(drone2)
                    neighbor_map[drone2.idx].append(drone1)
        self._neighbor_cache = neighbor_map
        return neighbor_map

    def get_neighbors(self, drone: 'Drone') -> List['Drone']:
        if self._neighbor_cache is None:
            self.refresh_neighbors()
        return self._neighbor_cache.get(drone.idx, []) if self._neighbor_cache else []

    def sense_fires(self, position: np.ndarray) -> List[Fire]:
        return [
            fire for fire in self._fires
            if fire.active and np.linalg.norm(position - fire.pos) <= FIRE_DETECTION_RADIUS
        ]

    def has_active_fire_near(self, position: np.ndarray, radius: float) -> bool:
        return any(fire.active and np.linalg.norm(position - fire.pos) <= radius for fire in self._fires)

    def request_extinguish(self, position: np.ndarray, water_available: float, flow_rate: float) -> float:
        remaining = min(max(water_available, 0.0), flow_rate * SIM_TIME_STEP)
        total_used = 0.0
        for fire in list(self._fires):
            if not fire.active or np.linalg.norm(position - fire.pos) > FIRE_EXTINGUISH_RADIUS:
                continue
            used = fire.extinguish(remaining)
            total_used += used
            remaining -= used
            if not remaining:
                break
        self._remove_dead_fires()
        return total_used

    def update_fires(self) -> None:
        for fire in self._fires:
            fire.grow(SIM_TIME_STEP)
        self._remove_dead_fires()

    def _remove_dead_fires(self) -> None:
        self._fires[:] = [fire for fire in self._fires if fire.active]



# ------------------------------------------------------------
# Drone
# ------------------------------------------------------------
# Logica decisionale del singolo drone.
# Swarm decentralizzato: percezione, comunicazione, pianificazione, movimento, azione sul mondo.

class Drone:
    """Logica autonoma: percezione locale, comunicazione, decisione e attuazione."""

    def __init__(self, idx: int, rng: random.Random, world: SimulationWorld):
        self.idx = idx
        self.world = world
        self.rng = rng
        self.position = np.array([rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float)
        self.velocity = np.zeros(2, dtype=float)
        self.acceleration = np.zeros(2, dtype=float)
        self.original_target = np.array([rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float)
        self.anchor_target = self.original_target.copy()
        self.target = self.original_target.copy()
        self.target_velocity = np.zeros(2, dtype=float)
        self.known_fires: Dict[Tuple[float, float], int] = {}
        self.extinguished_fires: Dict[Tuple[float, float], int] = {}
        self.fire_claim: Optional[Tuple[float, float]] = None
        self.idle_steps = 0
        self.desired_velocity = np.zeros(2, dtype=float)
        self.last_applied_force = np.zeros(2, dtype=float)
        self.water = DRONE_WATER_CAPACITY
        self.reloading = False
        self.water_station_idx: Optional[int] = None
        self.refuel_claim_age = 0
        self.communication = CommunicationModule(self)
        self.pid = PDController(PID_KP, PID_KI, PID_KD, PID_INTEGRAL_LIMIT, PID_MAX_OUTPUT_ACCEL, MAX_JERK)

    @property
    def neighbor_messages(self) -> Dict[int, DroneMessage]:
        return self.communication.neighbors

    def sense_environment(self) -> None:
        for fire_pos in list(self.known_fires):
            self.known_fires[fire_pos] += 1
            if self.known_fires[fire_pos] > FIRE_MEMORY_TTL_STEPS:
                del self.known_fires[fire_pos]

        for fire_pos in list(self.extinguished_fires):
            self.extinguished_fires[fire_pos] += 1
            if self.extinguished_fires[fire_pos] > EXTINGUISHED_FIRE_MEMORY_TTL_STEPS:
                del self.extinguished_fires[fire_pos]

        for fire_pos in list(self.known_fires):
            if np.linalg.norm(self.position - np.array(fire_pos)) <= FIRE_DETECTION_RADIUS:
                if not self.world.has_active_fire_near(np.array(fire_pos), 0.5):
                    del self.known_fires[fire_pos]
                    # Conferma diretta: registriamo lo spegnimento così che si propaghi ai vicini
                    # e non venga "resuscitato" da una loro voce non aggiornata (vedi merge_fire_knowledge).
                    self.extinguished_fires[fire_pos] = 0
                    if self.fire_claim == fire_pos:
                        self.fire_claim = None

        for fire in self.world.sense_fires(self.position):
            self.known_fires[vec_to_tuple(fire.pos)] = 0

        if self.fire_claim is not None and self.fire_claim not in self.known_fires:
            self.fire_claim = None

    def communicate(self, neighbors: List['Drone']) -> None:
        self.communication.communicate(neighbors)
        self.communication.merge_fire_knowledge()

    def _fire_priority_rank(self, fire_pos: Tuple[float, float]) -> int:
        """Posizione (0 = massima priorità) di questo drone nella coda locale di droni interessati
        a `fire_pos`, calcolata solo da informazioni note localmente (proprio stato + messaggi dei
        vicini): più un drone conosce da tempo/con certezza il fuoco (età minore), più alta è la sua
        priorità; a parità di età l'idx più basso vince, per rompere il pareggio in modo deterministico."""
        candidates: Dict[int, int] = {}
        self_age = self.known_fires.get(fire_pos)
        if self_age is not None:
            candidates[self.idx] = self_age
        elif self.fire_claim == fire_pos:
            candidates[self.idx] = 0
        for other_idx, message in self.neighbor_messages.items():
            age = message.known_fires.get(fire_pos)
            if age is None and message.fire_claim == fire_pos:
                age = 0
            if age is not None:
                candidates[other_idx] = min(age, candidates.get(other_idx, age))
        ranking = sorted(candidates.items(), key=lambda item: (item[1], item[0]))
        for rank, (idx, _age) in enumerate(ranking):
            if idx == self.idx:
                return rank
        return len(ranking)  # non dovrebbe mai accadere: self è sempre incluso tra i candidati

    def _fire_target_offset(self, fire_pos: np.ndarray, rank: int) -> np.ndarray:
        """Posizione di lavoro attorno al fuoco assegnata a un drone che lo sta attivamente estinguendo.
        `rank` (0 .. MAX_DRONES_ON_FIRE-1) distingue i droni assegnati contemporaneamente allo stesso
        fuoco, in modo che non convergano tutti sullo stesso punto."""
        angle = (rank * 2.399963) % (2.0 * np.pi)
        return fire_pos + FIRE_EXTINGUISH_RADIUS * 0.55 * np.array([np.cos(angle), np.sin(angle)])

    def _fire_wait_offset(self, fire_pos: np.ndarray, rank: int) -> np.ndarray:
        """Posizione di attesa attorno al fuoco per i droni in eccesso rispetto a MAX_DRONES_ON_FIRE,
        analoga alla coda usata per le stazioni idriche: ogni `rank` ottiene un angolo distinto così
        i droni in coda si distribuiscono invece di ammassarsi."""
        angle = (rank * 2.399963) % (2.0 * np.pi)
        wait_pos = fire_pos + WAIT_DISTANCE_FROM_FIRE * np.array([np.cos(angle), np.sin(angle)])
        wait_pos[0] = np.clip(wait_pos[0], 0.0, self.world.area_width)
        wait_pos[1] = np.clip(wait_pos[1], 0.0, self.world.area_height)
        return wait_pos

    def _select_fire(self) -> Optional[Tuple[float, float]]:
        if not self.known_fires:
            return None
        return min(self.known_fires, key=lambda p: np.linalg.norm(np.array(p) - self.position))

    def _claim_fire_if_needed(self) -> None:
        fire_pos = self._select_fire()
        if fire_pos is None:
            self.fire_claim = None
            return
        rank = self._fire_priority_rank(fire_pos)
        if rank < MAX_DRONES_ON_FIRE:
            # Tra i primi MAX_DRONES_ON_FIRE per questo fuoco: lo reclama come target attivo.
            self.fire_claim = fire_pos
            self.target = self._fire_target_offset(np.array(fire_pos, dtype=float), rank)
            self.original_target = self.target.copy()
            self.target_velocity[:] = 0.0
        else:
            # Troppi droni già assegnati: si posiziona in attesa vicino al fuoco, senza reclamarlo,
            # finché non salirà di priorità (es. i droni davanti finiscono l'acqua o il fuoco si sposta).
            self.fire_claim = None
            self.target = self._fire_wait_offset(np.array(fire_pos, dtype=float), rank)
            self.original_target = self.target.copy()
            self.target_velocity[:] = 0.0

    def compute_repulsion_between_targets(self, neighbors: List['Drone']) -> np.ndarray:
        force = np.zeros(2, dtype=float)
        for other in neighbors:
            delta = self.target - other.target
            dist = np.linalg.norm(delta)
            if dist >= TARGET_SEPARATION:
                continue
            if dist < 1e-6:
                pos_delta = self.position - other.position
                if np.linalg.norm(pos_delta) > 1e-6:
                    direction = normalize(pos_delta)
                else:
                    angle = (self.idx + 1) * 2.399963
                    direction = np.array([np.cos(angle), np.sin(angle)])
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

    def compute_fire_force(self) -> np.ndarray:
        if self.fire_claim is None:
            return np.zeros(2, dtype=float)
        fire = np.array(self.fire_claim, dtype=float)
        delta = fire - self.target
        dist = np.linalg.norm(delta)
        return K_FIRE_DRAGGING * max(dist - FIRE_EXTINGUISH_RADIUS * 0.33, 0.0) * normalize(delta)

    def _sample_exploration_target(self) -> np.ndarray:
        return np.array([self.rng.uniform(0.0, self.world.area_width), self.rng.uniform(0.0, self.world.area_height)], dtype=float)

    def _refresh_anchor(self) -> None:
        delta = self.target - self.anchor_target
        if np.linalg.norm(delta) > 1e-6:
            self.anchor_target += delta * min(0.1, ANCHOR_TO_TARGET_INTENSITY * SIM_TIME_STEP)

    def has_reached_target(self) -> bool:
        return bool(np.linalg.norm(self.position - self.target) < TARGET_REACHED_DISTANCE)

    def compute_total_force(self, neighbors: List['Drone']) -> np.ndarray:
        repulsion = self.compute_repulsion_between_targets(neighbors)
        if self.reloading:
            return repulsion
        total = repulsion + K_ANCHOR_DRAGGING * (self.anchor_target - self.target) + self.compute_fire_force() + self.compute_boundary_force()
        return clamp_magnitude(total, MAX_FORCE_ON_TARGET)

    def decide_and_move(self, neighbors: List['Drone']) -> None:
        self._maybe_start_reload()
        if self.reloading:
            self._move_toward_station()
            return
        self._claim_fire_if_needed()
        self._update_target_position(neighbors)
        self._integrate_motion(self._compute_desired_velocity_with_avoidance())
        self._handle_target_reached()
        self._maybe_resume_exploration()

    def _estimate_station_load(self, station_idx: int) -> int:
        return sum(1 for state in self.neighbor_messages.values()
                   if state.reloading and state.water_station_idx == station_idx
                   and np.linalg.norm(state.position - self.world.water_stations[station_idx]) <= WATER_STATION_SERVICE_RADIUS) + int(self.reloading and self.water_station_idx == station_idx and self._is_in_station_service_area(station_idx))

    def _is_in_station_service_area(self, station_idx: int) -> bool:
        return bool(np.linalg.norm(self.position - self.world.water_stations[station_idx]) <= WATER_STATION_SERVICE_RADIUS)

    def _select_water_station(self) -> int:
        return min((self._estimate_station_load(i) >= WATER_STATION_CAPACITY, self._estimate_station_load(i),
                    np.linalg.norm(self.position - pos), i) for i, pos in enumerate(self.world.water_stations))[3]

    def _station_service_priority(self) -> Tuple[int, int]:
        candidates = [(self.refuel_claim_age, self.idx)]
        if self.water_station_idx is None:
            return candidates[0]
        station_pos = self.world.water_stations[self.water_station_idx]
        candidates.extend((message.refuel_claim_age, other_idx)
                           for other_idx, message in self.neighbor_messages.items()
                           if message.reloading and message.water_station_idx == self.water_station_idx
                           and np.linalg.norm(message.position - station_pos) <= WATER_STATION_WAIT_RADIUS)
        return min(candidates)

    def _can_enter_station_service_area(self) -> bool:
        if self.water_station_idx is None:
            return False
        station_pos = self.world.water_stations[self.water_station_idx]
        candidates = [(self.refuel_claim_age, self.idx)]
        candidates.extend((message.refuel_claim_age, other_idx)
                           for other_idx, message in self.neighbor_messages.items()
                           if message.reloading and message.water_station_idx == self.water_station_idx
                           and np.linalg.norm(message.position - station_pos) <= WATER_STATION_WAIT_RADIUS)
        candidates.sort()
        return self.idx in [idx for _, idx in candidates[:WATER_STATION_CAPACITY]]

    def _maybe_start_reload(self) -> None:
        if self.reloading or self.is_extinguishing_fire() or self.water > LOW_WATER_THRESHOLD:
            return
        self.reloading = True
        self.water_station_idx = self._select_water_station()
        self.refuel_claim_age = 0
        self.fire_claim = None
        self.original_target = self.world.water_stations[self.water_station_idx].copy()
        self.target = self.original_target.copy()
        self.target_velocity[:] = 0.0

    def _move_toward_station(self) -> None:
        if self.water_station_idx is None:
            self.water_station_idx = self._select_water_station()
        self.refuel_claim_age += 1
        station_pos = self.world.water_stations[self.water_station_idx]
        priority = self._station_service_priority()
        if self._can_enter_station_service_area() and priority[1] == self.idx:
            target_pos = station_pos.copy()
        else:
            claims = [(message.refuel_claim_age, other_idx) for other_idx, message in self.neighbor_messages.items()
                      if message.reloading and message.water_station_idx == self.water_station_idx]
            claims.append((self.refuel_claim_age, self.idx))
            claims.sort()
            queue_index = next(i for i, (_, idx) in enumerate(claims) if idx == self.idx)
            angle = queue_index * 2.399963 + self.water_station_idx * 1.047197551
            target_pos = station_pos + np.array([np.cos(angle), np.sin(angle)]) * (WATER_STATION_WAIT_RADIUS + 0.35 * queue_index)
            target_pos[0] = np.clip(target_pos[0], 0.0, self.world.area_width)
            target_pos[1] = np.clip(target_pos[1], 0.0, self.world.area_height)
        self.target = target_pos
        self.target_velocity[:] = 0.0
        if self._can_enter_station_service_area() and priority[1] == self.idx and self.has_reached_target():
            self.velocity[:] = 0.0
            self.acceleration[:] = 0.0
            self.desired_velocity[:] = 0.0
            self.pid.reset()
            return
        self._integrate_motion(self._compute_desired_velocity_with_avoidance())

    def _update_target_position(self, neighbors: List['Drone']) -> None:
        force = self.compute_total_force(neighbors)
        self.last_applied_force = force.copy()
        self.target_velocity = (1.0 - TARGET_VEL_AGING_FACTOR) * self.target_velocity + force * SIM_TIME_STEP
        self.target_velocity = clamp_magnitude(self.target_velocity, MAX_TARGET_SPEED)
        self.target += self.target_velocity * SIM_TIME_STEP
        self.target[0] = np.clip(self.target[0], 0.0, self.world.area_width)
        self.target[1] = np.clip(self.target[1], 0.0, self.world.area_height)
        self._refresh_anchor()

    def _compute_desired_velocity_with_avoidance(self) -> np.ndarray:
        desired_velocity = K_DESIRED_VEL_TO_TARGET * (self.target - self.position)
        speed = np.linalg.norm(self.velocity)
        safety_margin = AVOID_MIN_DISTANCE + SAFE_DISTANCE_K_VEL * speed
        emergency = False
        emergency_vector = np.zeros(2, dtype=float)
        for state in self.neighbor_messages.values():
            rel = self.position - state.position
            distance = np.linalg.norm(rel)
            if distance < 1e-6:
                continue
            unit = rel / distance
            v_rel = self.velocity - state.velocity
            if np.dot(v_rel, unit) >= 0.0:
                continue
            margin = safety_margin
            if self.reloading and state.reloading and self.water_station_idx == state.water_station_idx:
                margin = min(margin, 0.65)
            predicted = rel + v_rel * AVOID_LOOKAHEAD
            predicted_distance = np.linalg.norm(predicted)
            if predicted_distance < margin:
                desired_velocity += normalize(predicted) * AVOID_FORCE * (margin - predicted_distance) / max(margin, 1e-6)
            if distance < EMERGENCY_AVOID_DISTANCE:
                emergency = True
                emergency_vector += unit * (EMERGENCY_AVOID_DISTANCE - distance) * 6.0 - v_rel * 0.8
        if emergency:
            desired_velocity = emergency_vector + 0.25 * (self.target - self.position)
        return clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)

    def _integrate_motion(self, desired_velocity: np.ndarray) -> None:
        self.desired_velocity = clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)
        self.acceleration = self.pid.step(self.desired_velocity, self.velocity, self.acceleration, SIM_TIME_STEP)
        self.velocity = clamp_magnitude(self.velocity + self.acceleration * SIM_TIME_STEP, MAX_DRONE_SPEED)
        self.position += self.velocity * SIM_TIME_STEP
        self.position[0] = np.clip(self.position[0], 0.0, self.world.area_width)
        self.position[1] = np.clip(self.position[1], 0.0, self.world.area_height)

    def _handle_target_reached(self) -> None:
        if self.has_reached_target():
            self.idle_steps += 1
            if self.is_extinguishing_fire():
                self.target_velocity[:] = 0.0
                self.target = self.position.copy()
                self.velocity *= 0.65
                self.acceleration[:] = 0.0
                self.desired_velocity[:] = 0.0
        else:
            self.idle_steps = 0

        # se sia il drone che il target sono sullo stesso fuoco, il drone comincia subito a spegnere l'incendio e il target viene portato sul drone:
        if self.fire_claim is not None and np.linalg.norm(self.position - np.array(self.fire_claim)) <= FIRE_EXTINGUISH_RADIUS * 1.25:
            self.target = self.position.copy()
            self.target_velocity[:] = 0.0

    def _maybe_resume_exploration(self) -> None:
        if not self.reloading and self.fire_claim is None and not self.known_fires and self.has_reached_target() and self.idle_steps > MAX_IDLE_STEPS:
            self.original_target = self._sample_exploration_target()
            self.anchor_target = self.original_target.copy()
            self.target = self.original_target.copy()
            self.target_velocity[:] = 0.0
            self.idle_steps = 0

    def is_extinguishing_fire(self) -> bool:
        return self.has_reached_target() and self.water > 0.0 and self.world.has_active_fire_near(self.position, FIRE_EXTINGUISH_RADIUS)

    def try_extinguish(self) -> None:
        if not self.has_reached_target() or self.water <= 0.0:
            return
        used = self.world.request_extinguish(self.position, self.water, DRONE_WATER_FLOW_RATE)
        self.water -= used
        if used > 1e-9:
            for fire_pos in list(self.known_fires):
                if np.linalg.norm(self.position - np.array(fire_pos)) <= FIRE_EXTINGUISH_RADIUS * 1.25:
                    del self.known_fires[fire_pos]
            self.fire_claim = None

    def try_reload(self) -> None:
        if not self.reloading or not self.has_reached_target():
            return
        self.water = min(DRONE_WATER_CAPACITY, self.water + WATER_STATION_REFILL_RATE * SIM_TIME_STEP)
        if self.water >= DRONE_WATER_CAPACITY - 1e-6:
            self.water = DRONE_WATER_CAPACITY
            self.reloading = False
            self.water_station_idx = None
            self.refuel_claim_age = 0
            self.target_velocity[:] = 0.0
            nearest = min(self.known_fires, key=lambda p: np.linalg.norm(np.array(p) - self.position), default=None)
            # Assegnazione "di partenza": il prossimo _claim_fire_if_needed() (già al prossimo passo)
            # ricalcolerà rank e target reali, mettendo il drone in coda se ormai il fuoco è già presidiato
            # da MAX_DRONES_ON_FIRE droni.
            if nearest is not None:
                rank = self._fire_priority_rank(nearest)
                self.fire_claim = nearest if rank < MAX_DRONES_ON_FIRE else None
                fire_offset = self._fire_target_offset if rank < MAX_DRONES_ON_FIRE else self._fire_wait_offset
                self.target = fire_offset(np.array(nearest, dtype=float), rank)
            else:
                self.fire_claim = None
                self.target = self._sample_exploration_target()
            self.original_target = self.target.copy()

    def run_step(self) -> None:
        self.sense_environment()
        neighbors = self.world.get_neighbors(self)
        self.communicate(neighbors)
        self.decide_and_move(neighbors)
        self.try_extinguish()
        self.try_reload()


# ------------------------------------------------------------
# SwarmSimulation (orchestrazione centrale e rendering)
# ------------------------------------------------------------
# Questa classe gestisce:
# - la creazione dell'ambiente
# - la logica di collisione globale
# - il rendering del mondo in una finestra Pygame

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


    def __init__(self, headless: bool = False, show_force_vectors: bool = False, show_communication: bool = True,
                 random_fires: bool = False, random_stations: bool = False, log_collisions: bool = False):
        self.headless = headless
        self.show_force_vectors = show_force_vectors
        self.show_communication = show_communication
        self.log_collisions = log_collisions

        self.rng = random.Random(RANDOM_SEED)
        self.fires = self._random_fires(NUM_FIRES) if random_fires else self._default_fires()
        self.water_stations = self._random_water_stations(NUM_WATER_STATIONS) if random_stations else self._default_water_stations()

        self.drones: List[Drone] = []
        self.world = SimulationWorld(self.drones, self.fires, AREA_WIDTH, AREA_HEIGHT, self.water_stations)
        self.drones.extend(Drone(i, self.rng, self.world) for i in range(NUM_DRONES))

        self.total_collisions = 0
        self.step_collisions = 0
        self._last_neighbors: Dict[int, List[Drone]] = {}

        # Inizializzazione Display / Pygame
        if self.headless:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.line_overlay = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        pygame.display.set_caption("Swarm Simulation 2D (Pygame)")
        self.font = pygame.font.SysFont("monospace", 15)

    def close(self):
        pygame.quit()

    def step(self, step_index: Optional[int] = None):
        self._last_neighbors = self.world.refresh_neighbors()

        for drone in self.drones:
            drone.run_step()

        self.world.update_fires()
        self.step_collisions = 0
        self._resolve_collisions(step_index)

    def _resolve_collisions(self, step_index: Optional[int] = None):
        """Rileva le collisioni fisiche per scopi statistici."""
        n = len(self.drones)
        for i in range(n):
            for j in range(i + 1, n):
                a = self.drones[i]
                b = self.drones[j]
                dist = np.linalg.norm(a.position - b.position)
                if dist < DRONE_IMPACT_RADIUS:  # il raggio visivo del drone in draw_scene è derivato da questa stessa costante
                    self.step_collisions += 1
                    self.total_collisions += 1
                    if not self.log_collisions:
                        continue  # evita tutto il calcolo di forze/vettori sottostante quando il logging è disattivato
                    neigh_a = self._last_neighbors.get(a.idx, [])
                    neigh_b = self._last_neighbors.get(b.idx, [])
                    force_a = a.compute_total_force(neigh_a)
                    force_b = b.compute_total_force(neigh_b)
                    rep_a = a.compute_repulsion_between_targets(neigh_a)
                    rep_b = b.compute_repulsion_between_targets(neigh_b)
                    fire_a = a.compute_fire_force()
                    fire_b = b.compute_fire_force()
                    boundary_a = a.compute_boundary_force()
                    boundary_b = b.compute_boundary_force()
                    print(
                        f"[collision] step={step_index} ids=({a.idx},{b.idx}) "
                        f"dist={dist:.4f} "
                        f"pos_a={tuple(np.round(a.position, 4))} pos_b={tuple(np.round(b.position, 4))} "
                        f"vel_a={tuple(np.round(a.velocity, 4))} vel_b={tuple(np.round(b.velocity, 4))} "
                        f"target_a={tuple(np.round(a.target, 4))} target_b={tuple(np.round(b.target, 4))} "
                        f"desired_a={tuple(np.round(a.desired_velocity, 4))} desired_b={tuple(np.round(b.desired_velocity, 4))} "
                        f"force_a={tuple(np.round(force_a, 4))} force_b={tuple(np.round(force_b, 4))} "
                        f"rep_a={tuple(np.round(rep_a, 4))} rep_b={tuple(np.round(rep_b, 4))} "
                        f"fire_a={tuple(np.round(fire_a, 4))} fire_b={tuple(np.round(fire_b, 4))} "
                        f"boundary_a={tuple(np.round(boundary_a, 4))} boundary_b={tuple(np.round(boundary_b, 4))} "
                        f"water_a={a.water:.2f} water_b={b.water:.2f} reloading_a={a.reloading} reloading_b={b.reloading}"
                    )

    def draw_scene(self):
        self.screen.fill((20, 20, 25))

        # 1. Stazione Idrica
        for station_idx, station_pos in enumerate(self.world.water_stations):
            st_x, st_y = world_to_screen(station_pos)
            pygame.draw.circle(self.screen, (30, 100, 200), (st_x, st_y), 18, 2)
            txt = self.font.render(f"WATER {station_idx}", True, (100, 180, 255))
            self.screen.blit(txt, (st_x - 28, st_y - 30))

        # 2. Incendi
        for fire in self.fires:
            fx, fy = world_to_screen(fire.pos)
            det_r = int((FIRE_DETECTION_RADIUS / AREA_WIDTH) * WINDOW_WIDTH)
            ext_r = int((FIRE_EXTINGUISH_RADIUS / AREA_WIDTH) * WINDOW_WIDTH)

            # Salute residua dell'incendio
            fire_txt = self.font.render(f"{fire.health:.0f}", True, (255, 200, 50))
            self.screen.blit(fire_txt, (fx - 10, fy - 30))
            
            # Cerchio Rilevamento (Arancione)
            pygame.draw.circle(self.screen, (255, 140, 40), (fx, fy), det_r, 1)
            # Cerchio Estinzione
            pygame.draw.circle(self.screen, (200, 80, 0), (fx, fy), ext_r, 1)
            
            # Corpo Fuoco (colore dinamico basato sulla salute residua)
            ratio = max(0.0, min(fire.health, FIRE_HEALTH) / FIRE_HEALTH)
            f_color = (int(100 + 155 * ratio), int(200 - 120 * ratio), int(100 - 100 * ratio), 50)
            #pygame.draw.circle(self.screen, f_color, (fx, fy), 6)
            draw_transparent_circle(self.screen, f_color, (fx, fy), ext_r)

        # 3. Connessioni di Comunicazione
        if self.show_communication:
            for drone in self.drones:
                for other in self._last_neighbors.get(drone.idx, []):
                    if other.idx > drone.idx:
                        p0 = world_to_screen(drone.position)
                        p1 = world_to_screen(other.position)
                        pygame.draw.line(self.screen, (100, 160, 220, 50), p0, p1, 1)

        # 4. Linee tratteggiate (disegnate su overlay trasparente per un effetto sfumato)
        self.line_overlay.fill((0, 0, 0, 0))
        for drone in self.drones:
            px, py = world_to_screen(drone.position)
            tx, ty = world_to_screen(drone.target)
            ax, ay = world_to_screen(drone.anchor_target)

            # Linea dal drone al target corrente, in stile tratteggiato.
            dx = tx - px
            dy = ty - py
            length = max(1.0, np.hypot(dx, dy))
            dash_len = 6
            gap_len = 5
            steps = int(length // (dash_len + gap_len))
            for i in range(steps):
                start_t = i * (dash_len + gap_len) / length
                end_t = min(1.0, (i * (dash_len + gap_len) + dash_len) / length)
                x0 = px + dx * start_t
                y0 = py + dy * start_t
                x1 = px + dx * end_t
                y1 = py + dy * end_t
                pygame.draw.line(self.line_overlay, DASHED_LINE_TARGET_COLOR, (x0, y0), (x1, y1), 1)

            # Linea dal drone all'ancora attuale, in stile tratteggiato.
            dx = ax - px
            dy = ay - py
            length = max(1.0, np.hypot(dx, dy))
            dash_len = 4
            gap_len = 5
            steps = int(length // (dash_len + gap_len))
            for i in range(steps):
                start_t = i * (dash_len + gap_len) / length
                end_t = min(1.0, (i * (dash_len + gap_len) + dash_len) / length)
                x0 = px + dx * start_t
                y0 = py + dy * start_t
                x1 = px + dx * end_t
                y1 = py + dy * end_t
                pygame.draw.line(self.line_overlay, DASHED_LINE_ANCHOR_COLOR, (x0, y0), (x1, y1), 1)

        self.screen.blit(self.line_overlay, (0, 0))

        # 5. Droni, Target e Vettori
        for drone in self.drones:
            px, py = world_to_screen(drone.position)
            tx, ty = world_to_screen(drone.target)
            ox, oy = world_to_screen(drone.original_target)
            ax, ay = world_to_screen(drone.anchor_target)

            # Target Originale, Ancora e Target Corrente
            pygame.draw.circle(self.screen, (50, 220, 80), (ox, oy), 3)
            pygame.draw.circle(self.screen, (160, 160, 255), (ax, ay), 3, 1)
            pygame.draw.circle(self.screen, (230, 50, 50), (tx, ty), 4)

            # Vettori di forza
            # repulsione con target: giallo, 
            # attrazione verso target originale: ciano, 
            # attrazione verso fuoco: arancione, 
            # repulsione dai bordi: blu, 
            # forza totale: bianco
            if self.show_force_vectors:
                neighs = self._last_neighbors.get(drone.idx, [])
                self._draw_vector(drone.target, drone.compute_repulsion_between_targets(neighs), (255, 200, 50), 0.2)
                self._draw_vector(drone.target, K_ANCHOR_DRAGGING * (drone.anchor_target - drone.target), (50, 255, 200), 0.2)
                self._draw_vector(drone.target, drone.compute_fire_force(), (255, 128, 0), 0.2)
                self._draw_vector(drone.target, drone.compute_boundary_force(), (50, 50, 230), 0.2)
                self._draw_vector(drone.target, drone.last_applied_force, (255, 255, 255), 0.2)

            # Indicatore Anello Estinzione
            if drone.is_extinguishing_fire():
                pygame.draw.circle(self.screen, (255, 230, 20), (px, py), 12, 2)

            # Indicatore Anello Primo in coda di rifornimento
            if drone.reloading and drone.water_station_idx is not None and drone._can_enter_station_service_area() and drone._station_service_priority()[1] == drone.idx:
                pygame.draw.circle(self.screen, (20, 200, 255), (px, py), 14, 2)

            # Corpo Drone (Colore basato sull'acqua residua, con sfumatura)
            w_ratio = drone.water / DRONE_WATER_CAPACITY
            d_color = (int(255 * (1.0 - w_ratio)), int(180 * w_ratio + 50), int(255 * w_ratio))
            drone_visual_radius = world_length_to_screen(DRONE_IMPACT_RADIUS)
            pygame.draw.circle(self.screen, d_color, (px, py), drone_visual_radius)

        # 6. Overlay Testo / Collisioni
        col_text = f"collisions_step={self.step_collisions} total={self.total_collisions}"
        ren = self.font.render(col_text, True, (255, 255, 0))
        self.screen.blit(ren, (10, 10))

        if not self.headless:
            pygame.display.flip()

    def _draw_vector(self, start_pos: np.ndarray, vec: np.ndarray, color: Tuple[int, int, int], scale: float, weight: int = 1): 
        if np.linalg.norm(vec) < 1e-8:
            return
        end_pos = start_pos + clamp_magnitude(vec, MAX_FORCE_ON_TARGET) * scale
        p0 = world_to_screen(start_pos)
        p1 = world_to_screen(end_pos)
        pygame.draw.line(self.screen, color, p0, p1, weight)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulazione 2D di swarm decentralizzato con Pygame")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Esegue la simulazione in modalità headless (senza finestra visibile)",
    )
    parser.add_argument(
        "--show-vectors",
        action="store_true",
        help="Abilita il disegno dei vettori di forza",
    )
    parser.add_argument(
        "--comm",
        action="store_true",
        help="Mostra le connessioni di comunicazione tra droni",
    )
    parser.add_argument(
        "--random-fires",
        action="store_true",
        help=f"Genera {NUM_FIRES} incendi in posizioni casuali invece della configurazione fissa di default",
    )
    parser.add_argument(
        "--random-stations",
        action="store_true",
        help=f"Genera {NUM_WATER_STATIONS} stazioni di rifornimento in posizioni casuali invece della configurazione fissa di default",
    )
    parser.add_argument(
        "--log",
        action="store_true",
        help="Abilita il logging dettagliato delle collisioni (forze, velocità, ecc.); disattivato di default per performance",
    )
    parser.add_argument(
        "--screenshot",
        nargs="?",
        const="screenshot.png",
        help="Salva uno screenshot della simulazione",
    )
    parser.add_argument(
        "--screenshot-step",
        type=int,
        default=0,
        help="Passo specifico a cui salvare lo screenshot (0 per salvare alla fine)",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Numero massimo di passi prima della terminazione automatica (default: nessun limite, simulazione infinita)",
    )
    parser.add_argument(
        "--stability-threshold",
        type=float,
        default=None,
        help="Soglia di spostamento medio dei target sotto la quale la simulazione si considera stabile e si ferma (default: disattivata)",
    )
    parser.add_argument(
        "--no-auto-stop",
        action="store_true",
        help="Ignora --max-steps e --stability-threshold: la simulazione continua finché non viene chiusa manualmente (Q o chiusura finestra)",
    )
    return parser


def save_image(surface: pygame.Surface, out_path: str) -> None:
    pygame.image.save(surface, out_path)


def run_simulation(
    headless: bool,
    show_force_vectors: bool = False,
    show_communication: bool = True,
    screenshot_path: Optional[str] = None,
    screenshot_step: int = 0,
    random_fires: bool = False,
    random_stations: bool = False,
    log_collisions: bool = False,
    max_steps: Optional[int] = None,
    stability_threshold: Optional[float] = None,
    auto_stop: bool = True,
) -> None:
    sim = SwarmSimulation(
        headless=headless,
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
        recent_moves = deque(maxlen=STABILITY_WINDOW)
        prev_targets = [drone.target.copy() for drone in sim.drones]

        running = True
        while running:
            # Gestione Eventi Tastiera/Finestra
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

            sim.draw_scene()

            total_move = 0.0
            for i, drone in enumerate(sim.drones):
                move = np.linalg.norm(drone.target - prev_targets[i])
                total_move += move
                prev_targets[i][:] = drone.target
            avg_move = total_move / max(1, len(sim.drones))
            recent_moves.append(avg_move)

            step_count += 1

            if screenshot_path is not None and screenshot_step > 0 and step_count == screenshot_step:
                save_image(sim.screen, screenshot_path)

            if auto_stop:
                if max_steps is not None and step_count >= max_steps:
                    running = False
                elif stability_threshold is not None and len(recent_moves) >= STABILITY_MIN_STEPS_BEFORE_STOP and (sum(recent_moves) / len(recent_moves)) < stability_threshold:
                    running = False

            if not headless:
                clock.tick(60)

        if screenshot_path is not None and (screenshot_step <= 0 or step_count < screenshot_step):
            save_image(sim.screen, screenshot_path)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = build_argument_parser()
    args = parser.parse_args()

    display_available = os.environ.get("DISPLAY") is not None or os.environ.get("WAYLAND_DISPLAY") is not None
    effective_headless = args.headless or not display_available

    run_simulation(
        headless=effective_headless,
        show_force_vectors=args.show_vectors,
        show_communication=args.comm,
        screenshot_path=args.screenshot,
        screenshot_step=args.screenshot_step,
        random_fires=args.random_fires,
        random_stations=args.random_stations,
        log_collisions=args.log,
        max_steps=args.max_steps,
        stability_threshold=args.stability_threshold,
        auto_stop=not args.no_auto_stop,
    )