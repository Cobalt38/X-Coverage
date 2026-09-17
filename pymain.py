import argparse
import os
import random
import sys
from collections import deque
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
SHOW_DRONES_TRAILS = False 
TRAIL_MAX_SEGMENTS = 60
TRAIL_MIN_MOVE = 0.12 # minimum distance moved before adding a new trail segment (meters)

MAX_SIMULATION_STEPS = 20000
LOG_INTERVAL = 100
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
ANCHOR_TO_TARGET_INTENSITY = 0.005  # l'ancora segue lentamente il target

K_DESIRED_VEL_TO_TARGET = 0.72
K_REPULSION_BETWEEN_TARGETS = 1.5 
K_ANCHOR_DRAGGING = 0.15
K_FIRE_DRAGGING = 4.0
K_BOUNDARY_REPULSION = 3.0

FIRE_DETECTION_RADIUS = 2.0
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

# Memoria dei fuochi: un fuoco non riconfermato da osservazione o comunicazione
# per più di FIRE_MEMORY_TTL_STEPS passi viene dimenticato (evita memoria infinita
# di incendi ormai spenti/irraggiungibili).
FIRE_MEMORY_TTL_STEPS = 100

# Numero di incendi generati quando --random-fires è attivo.
NUM_FIRES = 3

# Condizioni di terminazione della simulazione (usate sia in modalità headless che con finestra)
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
    # 1. Crea una superficie quadrata della dimensione del cerchio con supporto Alpha
    target_rect = pygame.Rect(center[0] - radius, center[1] - radius, radius * 2, radius * 2)
    shape_surface = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    
    # 2. Disegna il cerchio al centro della superficie temporanea
    pygame.draw.circle(shape_surface, color, (radius, radius), radius)
    
    # 3. Disegna la superficie temporanea sulla superficie di destinazione
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

class SimulationWorld:
    # espone solo:
    # - estensione della mappa
    # - vicinato radio
    # - sensori degli incendi
    # - azione coordinata di estinzione dell'incendio
    def __init__(self, drones: List['Drone'], fires: List[dict], area_width: float, area_height: float):
        self._drones = drones
        self._fires = fires
        self.area_width = area_width
        self.area_height = area_height
        self._neighbor_cache: Optional[Dict[int, List['Drone']]] = None
        self.water_stations = []

    def refresh_neighbors(self) -> Dict[int, List['Drone']]:
        neighbor_map: Dict[int, List['Drone']] = {d.idx: [] for d in self._drones}
        n = len(self._drones)
        for i in range(n):
            drone1 = self._drones[i]
            for j in range(i + 1, n):
                drone2 = self._drones[j]
                if np.linalg.norm(drone1.position - drone2.position) <= COMMUNICATION_RADIUS:
                    neighbor_map[drone1.idx].append(drone2)
                    neighbor_map[drone2.idx].append(drone1)
        self._neighbor_cache = neighbor_map
        return neighbor_map

    def get_neighbors(self, drone: 'Drone') -> List['Drone']:
        if self._neighbor_cache is None:
            self.refresh_neighbors()
        return self._neighbor_cache.get(drone.idx, []) if self._neighbor_cache else []

    def request_extinguish(self, position: np.ndarray, water_available: float, flow_rate: float) -> float:
        """Interfaccia con cui un drone richiede di applicare acqua agli incendi vicini."""
        total_used = 0.0
        for fire in self._fires:
            if fire["health"] <= 0.0:
                continue
            if np.linalg.norm(position - fire["pos"]) > FIRE_EXTINGUISH_RADIUS:
                continue
            remaining_water = water_available - total_used
            if remaining_water <= 0.0:
                break
            applied = min(remaining_water, flow_rate * SIM_TIME_STEP)
            applied = min(applied, fire["health"])
            fire["health"] -= applied
            total_used += applied
        return total_used


# ------------------------------------------------------------
# Drone
# ------------------------------------------------------------
# Logica decisionale del singolo drone.
# Swarm decentralizzato: percezione, comunicazione, pianificazione, movimento, azione sul mondo.

class Drone:
    def __init__(self, idx: int, rng: random.Random, world: SimulationWorld):
        self.idx = idx
        self.world = world
        self.position = np.array(
            [rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float
        )
        self.velocity = np.zeros(2, dtype=float)
        self.acceleration = np.zeros(2, dtype=float)
        self.original_target = np.array(
            [rng.uniform(0.0, world.area_width), rng.uniform(0.0, world.area_height)], dtype=float
        )
        self.anchor_target = self.original_target.copy()
        self.target = self.original_target.copy()
        self.target_velocity = np.zeros(2, dtype=float)
        # known_fires: posizione -> età in passi dall'ultima conferma (osservazione diretta
        # o comunicazione da un vicino). Un'età troppo alta fa "dimenticare" il fuoco.
        self.known_fires: Dict[Tuple[float, float], int] = {}
        self.idle_steps = 0

        self.trail_positions: deque = deque(maxlen=TRAIL_MAX_SEGMENTS)
        self.last_trail_pos = self.position.copy()
        self.history: deque = deque(maxlen=TRAIL_MAX_SEGMENTS + 1)
        self.history.append(self.position.copy())
        
        self.rng = rng
        self.desired_velocity = np.zeros(2, dtype=float)
        self.last_applied_force = np.zeros(2, dtype=float)
        self.water = DRONE_WATER_CAPACITY
        self.reloading = False
        self.water_station_idx: Optional[int] = None
        self.refuel_claim_age = 0

        # Ultimo stato pubblico noto dei vicini, ricevuto via radio durante communicate().
        self.neighbor_states: Dict[int, dict] = {}

        # Un unico PDController riutilizzato sia per il movimento normale verso il
        # target sia per il movimento verso la stazione idrica durante il reload.
        self.pid = PDController(
            kp=PID_KP, ki=PID_KI, kd=PID_KD,
            integral_limit=PID_INTEGRAL_LIMIT,
            max_output=PID_MAX_OUTPUT_ACCEL,
            max_jerk=MAX_JERK,
        )


    def sense_environment(self) -> None:
        """Fase 1: il drone aggiorna solo la propria memoria interna. I fuochi sono conosciuti solo tramite osservazioni locali e informazione ricevuta via radio."""
        # Invecchia tutta la memoria di un passo; le conferme (sotto, e in communicate()) la azzerano.
        for fire_pos in list(self.known_fires.keys()):
            self.known_fires[fire_pos] += 1
            if self.known_fires[fire_pos] > FIRE_MEMORY_TTL_STEPS:
                del self.known_fires[fire_pos]  # dimentica un fuoco non riconfermato da troppo tempo

        for fire_pos in list(self.known_fires.keys()):
            if np.linalg.norm(self.position - np.array(fire_pos, dtype=float)) <= FIRE_DETECTION_RADIUS:
                active_found = any(
                    f["health"] > 0.0 and np.linalg.norm(np.array(fire_pos) - f["pos"]) < 0.5
                    for f in self.world._fires
                )
                if not active_found:
                    del self.known_fires[fire_pos]

        for fire in self.world._fires: # looks like cheating because the drone interrogates the world directly, but it only updates its own memory of known fires.
            if fire["health"] <= 0.0:
                continue
            if np.linalg.norm(self.position - fire["pos"]) <= FIRE_DETECTION_RADIUS:
                self.known_fires[vec_to_tuple(fire["pos"])] = 0  # osservazione diretta: età azzerata

    def communicate(self, neighbors: List['Drone']) -> None:
        """Fase 2: scambio diretto (drone-to-drone) della conoscenza accumulata.
        Nessuna mediazione della simulazione centrale: ogni drone legge
        direttamente lo stato pubblico dei vicini che la radio gli ha indicato."""
        for other in neighbors:
            for fire_pos, age in other.known_fires.items():
                # Teniamo l'età minima: se un vicino ha una conferma più recente, la adottiamo.
                if fire_pos not in self.known_fires or age < self.known_fires[fire_pos]:
                    self.known_fires[fire_pos] = age

        # Il drone comunica ai vicini il proprio stato pubblico (posizione, velocità, target)
        self.neighbor_states = {
            other.idx: {
                "position": other.position.copy(),
                "velocity": other.velocity.copy(),
                "target": other.target.copy(),
                "reloading": other.reloading,
                "water_station_idx": other.water_station_idx,
                "refuel_claim_age": other.refuel_claim_age,
            }
            for other in neighbors
        } if neighbors and len(neighbors) > 0 else {}

    def compute_repulsion_between_targets(self, neighbors: List['Drone']) -> np.ndarray:
        F_repulsion = np.zeros(2, dtype=float)
        for other in neighbors:
            delta = self.target - other.target
            dist = np.linalg.norm(delta)
            if dist >= TARGET_SEPARATION:
                continue
            if dist < 1e-6:
                # Se i due target sono praticamente coincidenti, usiamo invece la
                # differenza tra le posizioni dei droni per trovare una direzione di repulsione
                # stabile ed evitare problemi numerici o allineamenti degeneri.
                pos_delta = self.position - other.position
                pos_dist = np.linalg.norm(pos_delta)
                if pos_dist > 1e-6:
                    direction = pos_delta / pos_dist
                else:
                    angle = (self.idx + 1) * 2.399963
                    direction = np.array([np.cos(angle), np.sin(angle)], dtype=float)
                F_repulsion += K_REPULSION_BETWEEN_TARGETS * TARGET_SEPARATION * direction
            else:
                direction = delta / dist
                F_repulsion += K_REPULSION_BETWEEN_TARGETS * (TARGET_SEPARATION - dist) * direction
        return clamp_magnitude(F_repulsion, MAX_FORCE_ON_TARGET)

    def compute_boundary_force(self) -> np.ndarray:
        force = np.zeros(2, dtype=float)
        margin = MARGIN_REPULSION_BOUNDARY
        area_width = self.world.area_width
        area_height = self.world.area_height
        if self.target[0] < margin:
            force[0] += K_BOUNDARY_REPULSION * (margin - self.target[0]) / max(margin, 1e-6)
        if self.target[0] > area_width - margin:
            force[0] -= K_BOUNDARY_REPULSION * (self.target[0] - (area_width - margin)) / max(margin, 1e-6)
        if self.target[1] < margin:
            force[1] += K_BOUNDARY_REPULSION * (margin - self.target[1]) / max(margin, 1e-6)
        if self.target[1] > area_height - margin:
            force[1] -= K_BOUNDARY_REPULSION * (self.target[1] - (area_height - margin)) / max(margin, 1e-6)
        return force

    def compute_fire_force(self) -> np.ndarray:
        # Attrazione verso gli incendi
        if not self.known_fires:
            return np.zeros(2, dtype=float)
        nearest_fire = min(self.known_fires, key=lambda fire: np.linalg.norm(np.array(fire, dtype=float) - self.target))
        nearest_vector = np.array(nearest_fire, dtype=float) - self.target
        dist = np.linalg.norm(nearest_vector)
        if dist < 1e-9:
            return np.zeros(2, dtype=float)
        spring_strength = max(dist - FIRE_EXTINGUISH_RADIUS * 0.5, 0.0) # Forza di attrazione verso il fuoco, smorzata quando il target è già vicino al raggio di estinzione.
        return K_FIRE_DRAGGING * spring_strength * normalize(nearest_vector)

    def _sample_exploration_target(self) -> np.ndarray:
        """Genera un obiettivo di esplorazione locale quando il drone deve riprendere a pattugliare senza un fuoco noto."""
        return np.array([self.rng.uniform(0.0, self.world.area_width), self.rng.uniform(0.0, self.world.area_height)], dtype=float)

    def _refresh_anchor(self, dt: float) -> None:
        """L'ancora si muove molto lentamente verso il target corrente. L'aggiornamento è volutamente molto smorzato."""
        delta = self.target - self.anchor_target
        if np.linalg.norm(delta) < 1e-6:
            return
        follow_step = min(0.25, ANCHOR_TO_TARGET_INTENSITY * dt)
        self.anchor_target += delta * follow_step

    def has_reached_target(self) -> bool:
        return bool(np.linalg.norm(self.position - self.target) < TARGET_REACHED_DISTANCE)

    def compute_total_force(self, neighbors: List['Drone']) -> np.ndarray:
        F_repulsion = self.compute_repulsion_between_targets(neighbors)
        if self.reloading:
            return clamp_magnitude(F_repulsion, MAX_FORCE_ON_TARGET)

        F_anchor = K_ANCHOR_DRAGGING * (self.anchor_target - self.target)
        F_fire = self.compute_fire_force()
        F_boundary = self.compute_boundary_force()

        total = F_repulsion + F_anchor + F_fire + F_boundary
        return clamp_magnitude(total, MAX_FORCE_ON_TARGET)

    def decide_and_move(self, neighbors: List['Drone']) -> None:
        """Fase 3: usando solo il proprio stato e quello (pubblico) dei vicini
        ottenuti via radio, il drone sposta il proprio target e si muove verso di esso.
        """
        self._maybe_start_reload()

        if self.reloading:
            self._move_toward_station(SIM_TIME_STEP)
            return

        self._update_target_position(neighbors, SIM_TIME_STEP)
        desired_velocity = self._compute_desired_velocity_with_avoidance()
        self._integrate_motion(desired_velocity, SIM_TIME_STEP)
        self._handle_target_reached()
        self._maybe_resume_exploration()
        self._update_trail()

    def _estimate_station_load(self, station_idx: int) -> int:
        load = 1 if self.reloading and self.water_station_idx == station_idx and self._is_in_station_service_area(station_idx) else 0
        for state in self.neighbor_states.values():
            if state.get("reloading") and state.get("water_station_idx") == station_idx:
                if np.linalg.norm(state["position"] - self.world.water_stations[station_idx]) <= WATER_STATION_SERVICE_RADIUS:
                    load += 1
        return load

    def _is_in_station_service_area(self, station_idx: int) -> bool:
        return bool(
            np.linalg.norm(self.position - self.world.water_stations[station_idx]) <= WATER_STATION_SERVICE_RADIUS
        )

    def _select_water_station(self) -> int:
        station_scores = []
        for station_idx, station_pos in enumerate(self.world.water_stations):
            load = self._estimate_station_load(station_idx)
            distance = np.linalg.norm(self.position - station_pos)
            station_scores.append((load >= WATER_STATION_CAPACITY, load, distance, station_idx))
        return min(station_scores)[3]

    def _station_service_priority(self) -> Tuple[int, int]:
        candidates = [(self.refuel_claim_age, self.idx)]
        if self.water_station_idx is None:
            return candidates[0]
        station_pos = self.world.water_stations[self.water_station_idx]
        for other_idx, state in self.neighbor_states.items():
            if not state.get("reloading") or state.get("water_station_idx") != self.water_station_idx:
                continue
            if np.linalg.norm(state["position"] - station_pos) <= WATER_STATION_WAIT_RADIUS:
                candidates.append((int(state.get("refuel_claim_age", 0)), other_idx))
        return min(candidates)

    def _can_enter_station_service_area(self) -> bool:
        if self.water_station_idx is None:
            return False
        station_idx = self.water_station_idx
        candidates = [(self.refuel_claim_age, self.idx)]
        station_pos = self.world.water_stations[station_idx]
        for other_idx, state in self.neighbor_states.items():
            if not state.get("reloading") or state.get("water_station_idx") != station_idx:
                continue
            if np.linalg.norm(state["position"] - station_pos) <= WATER_STATION_WAIT_RADIUS:
                candidates.append((int(state.get("refuel_claim_age", 0)), other_idx))
        candidates.sort()
        return self.idx in [idx for _, idx in candidates[:WATER_STATION_CAPACITY]]

    def _maybe_start_reload(self) -> None:
        """Decide se il drone deve iniziare il rifornimento, senza ancora muoversi."""
        if self.reloading or self.is_extinguishing_fire():
            return
        if self.water <= LOW_WATER_THRESHOLD:
            self.reloading = True
            self.water_station_idx = self._select_water_station()
            self.refuel_claim_age = 0
            self.original_target = self.world.water_stations[self.water_station_idx].copy()
            self.target = self.original_target.copy()
            self.target_velocity[:] = 0.0

    def _move_toward_station(self, dt: float) -> None:
        """Movimento verso la stazione idrica con gestione della coda di attesa."""
        area_width = self.world.area_width
        area_height = self.world.area_height

        if self.water_station_idx is None:
            self.water_station_idx = self._select_water_station()

        # Aggiunge il drone alla coda se non vi appartiene già
        self.refuel_claim_age += 1
        station_pos = self.world.water_stations[self.water_station_idx]
        service_priority = self._station_service_priority()
        can_service = self._can_enter_station_service_area()

        if can_service and service_priority[1] == self.idx:
            # È il primo in coda: si muove direttamente alla stazione
            target_pos = station_pos.copy()
        else:
            # In attesa: calcola un punto di stazionamento radiale attorno alla stazione
            local_claims = []
            for other_idx, state in self.neighbor_states.items():
                if state.get("reloading") and state.get("water_station_idx") == self.water_station_idx:
                    local_claims.append((int(state.get("refuel_claim_age", 0)), other_idx))
            local_claims.append((self.refuel_claim_age, self.idx))
            local_claims.sort()
            queue_index = next(i for i, (_, idx) in enumerate(local_claims) if idx == self.idx)
            angle = (queue_index * 2.399963) + (self.water_station_idx * 1.047197551)
            offset = np.array([np.cos(angle), np.sin(angle)], dtype=float) * (WATER_STATION_WAIT_RADIUS + 0.35 * queue_index)
            target_pos = station_pos + offset
            target_pos[0] = np.clip(target_pos[0], 0.0, area_width)
            target_pos[1] = np.clip(target_pos[1], 0.0, area_height)

        self.target = target_pos
        self.target_velocity[:] = 0.0
        dist_to_target = np.linalg.norm(target_pos - self.position)

        if can_service and service_priority[1] == self.idx and dist_to_target <= TARGET_REACHED_DISTANCE:
            self.velocity[:] = 0.0
            self.acceleration[:] = 0.0
            self.desired_velocity[:] = 0.0
            self.pid.reset()
            return

        desired_velocity = self._compute_desired_velocity_with_avoidance()
        self._integrate_motion(desired_velocity, dt, area_width, area_height)

    def _update_target_position(self, neighbors: List['Drone'], dt: float) -> None:
        """Aggiorna la posizione del target "virtuale" (non del drone) in base alle forze."""
        area_width = self.world.area_width
        area_height = self.world.area_height

        F_total_on_target = self.compute_total_force(neighbors)
        self.last_applied_force = F_total_on_target.copy()
        self.target_velocity = (1.0 - TARGET_VEL_AGING_FACTOR) * self.target_velocity + F_total_on_target * dt
        self.target_velocity = clamp_magnitude(self.target_velocity, MAX_TARGET_SPEED)
        self.target += self.target_velocity * dt
        self.target[0] = np.clip(self.target[0], 0.0, area_width)
        self.target[1] = np.clip(self.target[1], 0.0, area_height)
        self._refresh_anchor(dt)  # aggiorna l'ancora lentamente verso il target.

    def _compute_desired_velocity_with_avoidance(self) -> np.ndarray:
        """Velocità desiderata: punta al target, con evitamento locale predittivo."""
        desired_velocity = K_DESIRED_VEL_TO_TARGET * (self.target - self.position)

        speed = np.linalg.norm(self.velocity)  # velocità attuale del drone, non quella desiderata.
        safety_margin = AVOID_MIN_DISTANCE + SAFE_DISTANCE_K_VEL * speed
        emergency_avoidance = False
        emergency_vector = np.zeros(2, dtype=float)

        for other_state in self.neighbor_states.values():
            other_position = other_state["position"]
            other_velocity = other_state["velocity"]
            local_safety_margin = safety_margin
            if self.reloading and other_state.get("reloading") and self.water_station_idx is not None and other_state.get("water_station_idx") == self.water_station_idx:
                local_safety_margin = min(local_safety_margin, 0.65)

            distance_to_other_vector = self.position - other_position
            distance_to_other_scalar = np.linalg.norm(distance_to_other_vector)
            if distance_to_other_scalar < 1e-6:
                continue

            distance_to_other_unit = distance_to_other_vector / distance_to_other_scalar
            v_rel = self.velocity - other_velocity
            v_approach = np.dot(v_rel, distance_to_other_unit)

            if v_approach >= 0.0:  # si stanno già allontanando, nessun rischio
                continue

            predicted_rel = distance_to_other_vector + v_rel * AVOID_LOOKAHEAD
            predicted_dist = np.linalg.norm(predicted_rel)
            if predicted_dist <= 1e-6:
                # Posizione predetta degenere (quasi sovrapposizione): non possiamo
                # ricavarne una direzione. Ripieghiamo sulla direzione attuale
                # (nota, non predetta) verso l'altro drone, che resta un evitamento
                # ragionevole invece di ignorare del tutto il rischio di collisione.
                predicted_rel = distance_to_other_unit * max(distance_to_other_scalar, 1e-6)
                predicted_dist = np.linalg.norm(predicted_rel)

            if predicted_dist < local_safety_margin:
                avoid_dir = normalize(predicted_rel)
                avoid_strength = (local_safety_margin - predicted_dist) / max(local_safety_margin, 1e-6)
                desired_velocity += avoid_dir * (AVOID_FORCE * avoid_strength)

            if distance_to_other_scalar < EMERGENCY_AVOID_DISTANCE:
                emergency_avoidance = True
                emergency_vector += (distance_to_other_unit * (EMERGENCY_AVOID_DISTANCE - distance_to_other_scalar) * 6.0) - (v_rel * 0.8)

        if emergency_avoidance:
            desired_velocity = emergency_vector + 0.25 * (self.target - self.position)
            desired_velocity = clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)

        return clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)

    def _integrate_motion(self, desired_velocity: np.ndarray, dt: float, area_width: Optional[float] = None, area_height: Optional[float] = None) -> None:
        """Applica il PID condiviso (self.pid) per calcolare l'accelerazione e integra posizione/velocità."""
        if area_width is None:
            area_width = self.world.area_width
        if area_height is None:
            area_height = self.world.area_height

        self.desired_velocity = clamp_magnitude(desired_velocity, MAX_DRONE_SPEED)
        self.acceleration = self.pid.step(self.desired_velocity, self.velocity, self.acceleration, dt)

        self.velocity += self.acceleration * dt
        self.velocity = clamp_magnitude(self.velocity, MAX_DRONE_SPEED)
        self.position += self.velocity * dt
        self.position[0] = np.clip(self.position[0], 0.0, area_width)
        self.position[1] = np.clip(self.position[1], 0.0, area_height)

    def _handle_target_reached(self) -> None:
        """Al raggiungimento del target durante lo spegnimento o il rifornimento, esso viene congelato localmente. Questo evita che
        il drone resti in un loop di oscillazione quando il punto di interesse è già stato raggiunto.
        """
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

    def _maybe_resume_exploration(self) -> None:
        """Se il drone è fermo da troppo tempo ed è privo di fuochi noti, riprende
        l'esplorazione con un nuovo obiettivo casuale. Questo evita che un drone
        rimanga bloccato a vita se il fuoco che stava spegnendo è stato già
        spento e nessun altro gli ha dato nuove informazioni."""
        if not self.reloading and not self.known_fires and self.has_reached_target() and self.idle_steps > MAX_IDLE_STEPS:
            self.original_target = self._sample_exploration_target()
            self.anchor_target = self.original_target.copy()
            self.target = self.original_target.copy()
            self.target_velocity[:] = 0.0
            self.idle_steps = 0

    def _update_trail(self) -> None:
        moved = np.linalg.norm(self.position - self.last_trail_pos)
        if moved > TRAIL_MIN_MOVE:
            self.history.append(self.position.copy())
            self.last_trail_pos = self.position.copy()

    def is_extinguishing_fire(self) -> bool:
        """True quando il drone si trova sul target, entro il raggio di estinzione di un
        incendio ancora attivo nel mondo e ha acqua disponibile."""
        if not self.has_reached_target():
            return False
        if self.water <= 0.0:
            return False
        for fire in self.world._fires:
            if fire["health"] > 0.0 and np.linalg.norm(self.position - fire["pos"]) <= FIRE_EXTINGUISH_RADIUS:
                return True
        return False

    def try_extinguish(self) -> None:
        """Fase 4: il drone agisce sulla risorsa condivisa (salute dell'incendio)
        tramite l'unica interfaccia che la possiede: SimulationWorld.request_extinguish().
        """
        if not self.has_reached_target():
            return
        if self.water <= 0.0:
            return

        used = self.world.request_extinguish(self.position, self.water, DRONE_WATER_FLOW_RATE)
        self.water -= used
        if used > 1e-9:
            # Evita di tenere in memoria un fuoco ormai spento.
            for fire_pos in list(self.known_fires.keys()):
                if np.linalg.norm(self.position - np.array(fire_pos, dtype=float)) <= FIRE_EXTINGUISH_RADIUS * 1.25:
                    del self.known_fires[fire_pos]

    def try_reload(self) -> None:
        if not self.reloading:
            return
        if not self.has_reached_target():
            return
        self.water = min(DRONE_WATER_CAPACITY, self.water + WATER_STATION_REFILL_RATE * SIM_TIME_STEP)
        if self.water >= DRONE_WATER_CAPACITY - 1e-6:
            self.reloading = False
            # Rimuovi il drone dalla coda della stazione
            self.water_station_idx = None
            self.refuel_claim_age = 0
            
            known_fire_positions = [np.array(fire_pos, dtype=float) for fire_pos in self.known_fires]
            if known_fire_positions:
                nearest_fire = min(
                    known_fire_positions,
                    key=lambda fire_pos: np.linalg.norm(fire_pos - self.position),
                )
                self.original_target = nearest_fire.copy()
                self.target = nearest_fire.copy()
            else:
                self.original_target = self._sample_exploration_target()
                self.target = self.original_target.copy()
            self.target_velocity[:] = 0.0

    def run_step(self) -> None:
        """Sequenza completa di un passo di simulazione per QUESTO drone soltanto.

        Consolidata qui (invece che come 5 loop separati dentro SwarmSimulation.step)
        così che SwarmSimulation resti pura orchestrazione: rinfresca i vicini una
        volta per tutti i droni, poi chiama run_step() su ciascuno, senza conoscere né
        gestire i dettagli delle singole fasi (percezione, comunicazione, pianificazione,
        movimento, azione). I vicini vengono letti dalla cache del mondo (già aggiornata
        da SwarmSimulation prima del giro di run_step), non passati esplicitamente.
        """
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
# - la pulizia degli incendi estinti
# - il rendering del mondo in una finestra Pygame

def _default_fires() -> List[dict]:
    """Configurazione di incendi fissa e riproducibile (usata quando --random-fires non è passato)."""
    positions = [
        (AREA_WIDTH * 0.20, AREA_HEIGHT * 0.75),
        (AREA_WIDTH * 0.50, AREA_HEIGHT * 0.25),
        (AREA_WIDTH * 0.80, AREA_HEIGHT * 0.75),
    ]
    return [{"pos": np.array(p, dtype=float), "health": FIRE_HEALTH} for p in positions]


def _random_fires(rng: random.Random, count: int) -> List[dict]:
    """Genera `count` incendi in posizioni casuali entro l'area, con un margine dai bordi."""
    margin = MARGIN_REPULSION_BOUNDARY
    fires = []
    for _ in range(count):
        pos = np.array(
            [rng.uniform(margin, AREA_WIDTH - margin), rng.uniform(margin, AREA_HEIGHT - margin)],
            dtype=float,
        )
        fires.append({"pos": pos, "health": FIRE_HEALTH})
    return fires


def _default_water_stations() -> List[np.ndarray]:
    positions = [
        (AREA_WIDTH * 0.20, AREA_HEIGHT * 0.25),
        (AREA_WIDTH * 0.50, AREA_HEIGHT * 0.75),
        (AREA_WIDTH * 0.80, AREA_HEIGHT * 0.25),
    ]
    return [np.array(p, dtype=float) for p in positions[:NUM_WATER_STATIONS]]


def _random_water_stations(rng: random.Random, count: int) -> List[np.ndarray]:
    margin = MARGIN_REPULSION_BOUNDARY
    stations = []
    attempts = 0
    while len(stations) < count and attempts < count * 100:
        attempts += 1
        pos = np.array(
            [rng.uniform(margin, AREA_WIDTH - margin), rng.uniform(margin, AREA_HEIGHT - margin)],
            dtype=float,
        )
        if all(np.linalg.norm(pos - other) >= WATER_STATION_MIN_SEPARATION for other in stations):
            stations.append(pos)
    if len(stations) < count:
        return _default_water_stations()[:count]
    return stations


class SwarmSimulation:
    def __init__(self, headless: bool = False, show_force_vectors: bool = False, show_communication: bool = True,
                 random_fires: bool = False, random_stations: bool = False, log_collisions: bool = False):
        self.headless = headless
        self.show_force_vectors = show_force_vectors
        self.show_communication = show_communication
        self.log_collisions = log_collisions

        self.rng = random.Random(RANDOM_SEED)
        self.fires = _random_fires(self.rng, NUM_FIRES) if random_fires else _default_fires()
        self.water_stations = _random_water_stations(self.rng, NUM_WATER_STATIONS) if random_stations else _default_water_stations()

        self.drones: List[Drone] = []
        self.world = SimulationWorld(self.drones, self.fires, AREA_WIDTH, AREA_HEIGHT)
        self.world.water_stations = self.water_stations
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
        pygame.display.set_caption("Swarm Simulation 2D (Pygame)")
        self.font = pygame.font.SysFont("monospace", 15)

    def close(self):
        pygame.quit()

    def step(self, step_index: Optional[int] = None):
        self._last_neighbors = self.world.refresh_neighbors()

        for drone in self.drones:
            drone.run_step()

        for fire in self.fires:
            if fire["health"] > 0.0:
                fire["health"] += FIRE_GROWTH_RATE * SIM_TIME_STEP

        self.step_collisions = 0
        self._resolve_collisions(step_index)

        self._remove_extinguished_fires()

    def _remove_extinguished_fires(self):
        extinguished_positions = [f["pos"] for f in self.fires if f["health"] <= 0.0]
        self.fires = [f for f in self.fires if f["health"] > 0.0]
        self.world._fires = self.fires

        for drone in self.drones:
            for ext_pos in extinguished_positions:
                for fire_pos in list(drone.known_fires.keys()):
                    if np.linalg.norm(np.array(fire_pos) - ext_pos) < 0.5:
                        del drone.known_fires[fire_pos]

    def _resolve_collisions(self, step_index: Optional[int] = None):
        """Rileva le collisioni fisiche reali per scopi statistici, senza applicare
        alcuno spostamento forzato o trucco di posizionamento artificiale."""
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
            fx, fy = world_to_screen(fire["pos"])
            det_r = int((FIRE_DETECTION_RADIUS / AREA_WIDTH) * WINDOW_WIDTH)
            ext_r = int((FIRE_EXTINGUISH_RADIUS / AREA_WIDTH) * WINDOW_WIDTH)

            # Salute residua dell'incendio
            fire_txt = self.font.render(f"{fire['health']:.0f}", True, (255, 200, 50))
            self.screen.blit(fire_txt, (fx - 10, fy - 30))
            
            # Cerchio Rilevamento (Arancione)
            pygame.draw.circle(self.screen, (255, 140, 40), (fx, fy), det_r, 1)
            # Cerchio Estinzione
            pygame.draw.circle(self.screen, (200, 80, 0), (fx, fy), ext_r, 1)
            
            # Corpo Fuoco (colore dinamico basato sulla salute residua)
            ratio = max(0.0, min(fire["health"], FIRE_HEALTH) / FIRE_HEALTH)
            f_color = (int(100 + 155 * ratio), int(200 - 120 * ratio), int(100 - 100 * ratio))
            pygame.draw.circle(self.screen, f_color, (fx, fy), 6)
            #draw_transparent_circle(self.screen, f_color, (fx, fy), ext_r)

        # 3. Scie dei droni
        if SHOW_DRONES_TRAILS:
            for drone in self.drones:
                hist = list(drone.history)
                for k in range(len(hist) - 1):
                    p0 = world_to_screen(hist[k])
                    p1 = world_to_screen(hist[k + 1])
                    pygame.draw.line(self.screen, (60, 120, 180), p0, p1, 1)

        # 4. Connessioni di Comunicazione
        if self.show_communication:
            for drone in self.drones:
                for other in self._last_neighbors.get(drone.idx, []):
                    if other.idx > drone.idx:
                        p0 = world_to_screen(drone.position)
                        p1 = world_to_screen(other.position)
                        pygame.draw.line(self.screen, (100, 160, 220, 50), p0, p1, 1)

        # 5. Droni, Target e Vettori
        for drone in self.drones:
            px, py = world_to_screen(drone.position)
            tx, ty = world_to_screen(drone.target)
            ox, oy = world_to_screen(drone.original_target)

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
                pygame.draw.line(self.screen, (255, 110, 255, 120), (x0, y0), (x1, y1), 1)

            # Target Originale, Ancora e Target Corrente
            ax, ay = world_to_screen(drone.anchor_target)
            pygame.draw.circle(self.screen, (50, 220, 80), (ox, oy), 3)
            pygame.draw.circle(self.screen, (160, 160, 255), (ax, ay), 3, 1)
            pygame.draw.circle(self.screen, (230, 50, 50), (tx, ty), 4)

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
                pygame.draw.line(self.screen, (110, 250, 110, 120), (x0, y0), (x1, y1), 1)

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