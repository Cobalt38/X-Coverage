"""
Drone autonomo e controllore PID di velocità.
"""

import math
import random
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

import numpy as np

from config import GOLDEN_ANGLE, FirePos, SimConfig
from world import CommunicationModule, DroneMessage, clamp_magnitude, normalize, unit_from_angle, vec_to_tuple

if TYPE_CHECKING:
    from world import SimulationWorld


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

    def __init__(self, idx: int, rng: random.Random, world: 'SimulationWorld', seed: int):
        self.idx = idx
        self.world = world
        self.cfg: SimConfig = world.cfg
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
        self.water = self.cfg.DRONE_WATER_CAPACITY
        self.reloading = False
        self.water_station_idx: Optional[int] = None
        self.refuel_claim_age = 0
        self.station_slot: Optional[int] = None
        self.communication = CommunicationModule(self)
        self.pid = PIDController(self.cfg.PID_KP, self.cfg.PID_KI, self.cfg.PID_KD, self.cfg.PID_INTEGRAL_LIMIT, self.cfg.PID_MAX_OUTPUT_ACCEL, self.cfg.MAX_JERK)
        # Telemetria: letta solo dal MetricsCollector, mai dalla logica decisionale.
        self.in_emergency = False
        self.water_delivered = 0.0
        self.bounce_count = 0

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
            if self.known_fires[fire_pos] > self.cfg.FIRE_MEMORY_TTL_STEPS:
                del self.known_fires[fire_pos]

        # Aging memoria incendi spenti
        for fire_pos in list(self.extinguished_fires):
            self.extinguished_fires[fire_pos] += 1
            if self.extinguished_fires[fire_pos] > self.cfg.EXTINGUISHED_FIRE_MEMORY_TTL_STEPS:
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
            if np.linalg.norm(self.position - np.array(fire_pos)) <= self.cfg.FIRE_DETECTION_RADIUS:
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
        my_key = (0 if (self.water > 0.0 and my_dist <= self.cfg.FIRE_EXTINGUISH_RADIUS) else 1, my_dist, self.idx)
        rank = 0
        for other_idx, msg in self._neighbor_messages.items():
            dist = float(np.linalg.norm(msg.position - fire_arr))
            working_here = msg.extinguishing and dist <= self.cfg.FIRE_EXTINGUISH_RADIUS
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
            if self._fire_rank(fire_pos) < self.cfg.MAX_DRONES_ON_FIRE:
                return fire_pos, None
            self.saturated_fires[fire_pos] = self.cfg.FIRE_SATURATION_MEMORY_STEPS
            if bounce_from is None and np.linalg.norm(self.position - np.array(fire_pos)) <= self.cfg.FIRE_DETECTION_RADIUS:
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
        self.target = fire_arr + bearing * self.cfg.FIRE_WORK_RADIUS
        self.target[0] = np.clip(self.target[0], 0.0, self.world.area_width)
        self.target[1] = np.clip(self.target[1], 0.0, self.world.area_height)
        self.original_target = fire_arr.copy()
        self.anchor_target = self.target.copy()  # al rilascio del compito il drone riparte da qui
        self.target_velocity[:] = 0.0

    def _bounce_away_from(self, fire_pos: FirePos) -> None:
        """Rimbalzo radiale da un incendio saturo."""
        self.bounce_count += 1
        fire_arr = np.array(fire_pos, dtype=float)
        bounce_dir = normalize(self.position - fire_arr)
        if np.linalg.norm(bounce_dir) < 1e-6:
            bounce_dir = normalize(self.velocity) if np.linalg.norm(self.velocity) > 1e-6 else unit_from_angle(self.idx * GOLDEN_ANGLE)

        # Applica lo spostamento usando la variabile di configurazione
        self.target = self.position + bounce_dir * self.cfg.FIRE_SATURATION_BOUNCE_DISTANCE
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
            if dist >= self.cfg.TARGET_SEPARATION:
                continue
            if dist < 1e-6:
                pos_delta = self.position - msg.position
                direction = normalize(pos_delta) if np.linalg.norm(pos_delta) > 1e-6 else unit_from_angle(self.idx * GOLDEN_ANGLE)
                force += self.cfg.K_REPULSION_BETWEEN_TARGETS * self.cfg.TARGET_SEPARATION * direction
            else:
                force += self.cfg.K_REPULSION_BETWEEN_TARGETS * (self.cfg.TARGET_SEPARATION - dist) * delta / dist
        return clamp_magnitude(force, self.cfg.MAX_FORCE_ON_TARGET)

    def compute_boundary_force(self) -> np.ndarray:
        force = np.zeros(2, dtype=float)
        margin = self.cfg.MARGIN_REPULSION_BOUNDARY
        if self.target[0] < margin:
            force[0] += self.cfg.K_BOUNDARY_REPULSION * (margin - self.target[0]) / max(margin, 1e-6)
        elif self.target[0] > self.world.area_width - margin:
            force[0] -= self.cfg.K_BOUNDARY_REPULSION * (self.target[0] - (self.world.area_width - margin)) / max(margin, 1e-6)
        if self.target[1] < margin:
            force[1] += self.cfg.K_BOUNDARY_REPULSION * (margin - self.target[1]) / max(margin, 1e-6)
        elif self.target[1] > self.world.area_height - margin:
            force[1] -= self.cfg.K_BOUNDARY_REPULSION * (self.target[1] - (self.world.area_height - margin)) / max(margin, 1e-6)
        return force

    def compute_anchor_force(self) -> np.ndarray:
        return self.cfg.K_ANCHOR_DRAGGING * (self.anchor_target - self.target)

    def compute_total_force(self) -> np.ndarray:
        total = self.compute_repulsion_between_targets() + self.compute_anchor_force() + self.compute_boundary_force()
        return clamp_magnitude(total, self.cfg.MAX_FORCE_ON_TARGET)

    # --------------------------------------------------------
    # Dinamica del target
    # --------------------------------------------------------

    def _sample_exploration_target(self) -> np.ndarray:
        return np.array([self.rng.uniform(0.0, self.world.area_width), self.rng.uniform(0.0, self.world.area_height)], dtype=float)

    def _refresh_anchor(self) -> None:
        delta = self.target - self.anchor_target
        if np.linalg.norm(delta) > 1e-6:
            self.anchor_target += delta * (1.0 - math.exp(-self.cfg.ANCHOR_TO_TARGET_INTENSITY * self.cfg.SIM_TIME_STEP))

    def has_reached_target(self) -> bool:
        return bool(np.linalg.norm(self.position - self.target) < self.cfg.TARGET_REACHED_DISTANCE)

    def _update_target_position(self) -> None:
        force = self.compute_total_force()
        self.last_applied_force = force.copy()
        decay = math.exp(-self.cfg.TARGET_VEL_DAMPING_RATE * self.cfg.SIM_TIME_STEP)
        self.target_velocity = decay * self.target_velocity + force * self.cfg.SIM_TIME_STEP
        self.target_velocity = clamp_magnitude(self.target_velocity, self.cfg.MAX_TARGET_SPEED)
        self.target += self.target_velocity * self.cfg.SIM_TIME_STEP
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
        safety_margin = self.cfg.AVOID_MIN_DISTANCE + self.cfg.SAFE_DISTANCE_K_VEL * np.linalg.norm(self.velocity) # più veloce il drone -> maggiore il margine necessario ad evitare collisioni

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
            t_ca = float(np.clip(-np.dot(rel, v_rel) / speed_sq, 0.0, self.cfg.AVOID_LOOKAHEAD)) if speed_sq > 1e-12 else 0.0 # tempo di massimo avvicinamento predetto, ma se v_rel è nullo allora t_ca = 0
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
                predictive += direction * (self.cfg.K_AVOID_REPULSION * penetration + self.cfg.K_AVOID_DAMPING * closing_speed)

            if distance < self.cfg.EMERGENCY_AVOID_DISTANCE:
                emergency = True
                emergency_vector += unit_now * (self.cfg.EMERGENCY_AVOID_DISTANCE - distance) * self.cfg.EMERGENCY_AVOID_GAIN - v_rel * self.cfg.EMERGENCY_DAMPING

        return clamp_magnitude(predictive, self.cfg.AVOID_MAX_CORRECTION), emergency, emergency_vector

    def _compute_desired_velocity_with_avoidance(self) -> np.ndarray:
        """Navigazione verso il target + avoidance -> velocità desiderata limitata.

        La navigazione viene limitata a MAX_DRONE_SPEED PRIMA di sommare l'avoidance: altrimenti,
        con un target lontano (es. 0.7 * 15 m = 10.5 m/s), la correzione verrebbe diluita dal clamp finale.
        """
        navigation = clamp_magnitude(self.cfg.K_DESIRED_VEL_TO_TARGET * (self.target - self.position), self.cfg.MAX_DRONE_SPEED)
        mode = self.cfg.AVOIDANCE_MODE
        if mode == "none":
            # Ablation: nessun evitamento, solo navigazione (limite inferiore di sicurezza)
            self.in_emergency = False
            self.last_avoidance_force = np.zeros(2, dtype=float)
            return navigation
        predictive, emergency, emergency_vector = self._compute_collision_avoidance()
        if mode == "emergency-only":
            # Ablation: solo repulsione reattiva sulla distanza attuale, senza predizione CPA
            predictive = np.zeros(2, dtype=float)
        self.in_emergency = emergency
        if emergency:
            self.last_avoidance_force = emergency_vector
            desired_velocity = emergency_vector + self.cfg.EMERGENCY_TARGET_WEIGHT * navigation
        else:
            self.last_avoidance_force = predictive
            desired_velocity = navigation + predictive
        return clamp_magnitude(desired_velocity, self.cfg.MAX_DRONE_SPEED)

    # --------------------------------------------------------
    # Moto
    # --------------------------------------------------------

    def _integrate_motion(self, desired_velocity: np.ndarray) -> None:
        self.desired_velocity = clamp_magnitude(desired_velocity, self.cfg.MAX_DRONE_SPEED)
        self.acceleration = self.pid.step(self.desired_velocity, self.velocity, self.acceleration, self.cfg.SIM_TIME_STEP)
        self.velocity = clamp_magnitude(self.velocity + self.acceleration * self.cfg.SIM_TIME_STEP, self.cfg.MAX_DRONE_SPEED)
        self.position += self.velocity * self.cfg.SIM_TIME_STEP
        self.position[0] = np.clip(self.position[0], 0.0, self.world.area_width)
        self.position[1] = np.clip(self.position[1], 0.0, self.world.area_height)

    def _maybe_resume_exploration(self) -> None:
        """Nessun compito attivo e target raggiunto -> dopo MAX_IDLE_STEPS nuovo target casuale."""
        if not self.reloading and self.fire_target is None and self.has_reached_target():
            self.idle_steps += 1
            if self.idle_steps > self.cfg.MAX_IDLE_STEPS:
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
        return bool(np.linalg.norm(position - self.world.water_stations[station_idx]) <= self.cfg.WATER_STATION_SERVICE_RADIUS)

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
            candidates.append((load >= self.cfg.WATER_STATION_CAPACITY, np.linalg.norm(self.position - pos), load, i))
        return min(candidates)[3]

    def _station_slot_position(self, station_idx: int, slot: int) -> np.ndarray:
        station_pos = self.world.water_stations[station_idx]
        if self.cfg.WATER_STATION_CAPACITY <= 1:
            return station_pos.copy()
        angle = station_idx * math.pi / 3.0 + 2.0 * math.pi * slot / self.cfg.WATER_STATION_CAPACITY
        return station_pos + unit_from_angle(angle) * self.cfg.WATER_STATION_SLOT_RADIUS

    def _update_station_slot(self) -> None:
        """Ammissione (primi WATER_STATION_CAPACITY in priorità) e scelta di uno slot libero.

        Lo slot è "appiccicoso": una volta preso lo si tiene fino a fine rifornimento. In caso di
        conflitto (due droni scelgono lo stesso slot con dati vecchi di uno step) cede chi ha priorità minore.
        """
        competitors = self._station_competitors()
        my_rank = next(i for i, (_, idx, _) in enumerate(competitors) if idx == self.idx)
        if my_rank >= self.cfg.WATER_STATION_CAPACITY:
            self.station_slot = None
            return
        my_key = competitors[my_rank][0]
        others = [(key, slot) for key, idx, slot in competitors if idx != self.idx and slot is not None]
        if self.station_slot is not None and any(slot == self.station_slot and key < my_key for key, slot in others):
            self.station_slot = None
        if self.station_slot is None:
            taken = {slot for _, slot in others}
            free = [s for s in range(self.cfg.WATER_STATION_CAPACITY) if s not in taken]
            if free:
                self.station_slot = free[0]

    def _maybe_start_reload(self) -> None:
        """Se il drone ha poca acqua e non sta già rifornendo, sceglie una stazione e inizia a dirigersi verso di essa."""
        if self.reloading or self.is_extinguishing_fire() or self.water > self.cfg.LOW_WATER_THRESHOLD:
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
            target_pos = station_pos + bearing * self.cfg.WATER_STATION_WAIT_RADIUS
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
        self.water = min(self.cfg.DRONE_WATER_CAPACITY, self.water + self.cfg.WATER_STATION_REFILL_RATE * self.cfg.SIM_TIME_STEP)
        if self.water >= self.cfg.DRONE_WATER_CAPACITY - 1e-6:
            self.water = self.cfg.DRONE_WATER_CAPACITY
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
        return self.water > 0.0 and self.world.has_active_fire_near(self.position, self.cfg.FIRE_EXTINGUISH_RADIUS)

    def try_extinguish(self) -> None:
        if self.water <= 0.0:
            return
        used, extinguished = self.world.request_extinguish(self.position, self.water, self.cfg.DRONE_WATER_FLOW_RATE)
        self.water -= used
        self.water_delivered += used
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
            if saturated_nearby is not None and self.cfg.SATURATION_BOUNCE:
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

