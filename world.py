"""
Il mondo fisico della simulazione: incendi, canale radio, sensori e attuatori.

Contiene tutto ciò che NON è il drone:
    - funzioni vettoriali di supporto;
    - Fire: un incendio che cresce se nessuno lo spegne;
    - DroneMessage / CommunicationModule: cosa si dicono i droni e come ognuno ricorda i messaggi ricevuti;
    - RadioChannel: chi sente chi (portata radio) e quali messaggi si perdono;
    - SimulationWorld: sensori (vedere gli incendi) e attuatori (spruzzare acqua) messi a disposizione dei droni.
"""

import math
import random
from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

import numpy as np

from config import FirePos, SimConfig

if TYPE_CHECKING:
    from drone import Drone


# ------------------------------------------------------------
# Funzioni vettoriali
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


# ------------------------------------------------------------
# Incendio
# ------------------------------------------------------------

@dataclass(eq=False)
class Fire:
    pos: np.ndarray
    health: float
    growth_rate: float

    @property
    def active(self) -> bool:
        return self.health > 0.0

    def grow(self, dt: float) -> None:
        if self.active:
            self.health += self.growth_rate * dt

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

    Un messaggio perso dal canale semplicemente non arriva: in quel round il mittente
    è invisibile al ricevente (nessuna ritenzione del messaggio precedente).
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


class RadioChannel:
    """Modello del canale radio: portata (disco di raggio COMMUNICATION_RADIUS) e perdita di pacchetti.

    La perdita è Bernoulli i.i.d. per ogni coppia (mittente, ricevente) e per ogni round.
    Usa un generatore dedicato: con loss > 0 la sequenza degli incendi e le scelte dei droni
    non consumano gli stessi numeri casuali, quindi le condizioni iniziali restano confrontabili
    tra esperimenti con lo stesso seed. Con loss = 0 non viene estratto alcun numero.
    """

    def __init__(self, radius: float, loss_probability: float, seed: int):
        self.radius = radius
        self.loss_probability = loss_probability
        self.rng = random.Random(f"channel-{seed}")
        self.neighbor_map: Dict[int, List['Drone']] = {}
        # Contatori cumulativi (telemetria, non usati dai droni)
        self.messages_attempted = 0
        self.messages_delivered = 0

    @property
    def messages_dropped(self) -> int:
        return self.messages_attempted - self.messages_delivered

    def refresh_neighbors(self, drones: List['Drone']) -> Dict[int, List['Drone']]:
        """Chi è nel raggio di chi. Liste in ordine crescente di indice."""
        n = len(drones)
        neighbor_map: Dict[int, List['Drone']] = {d.idx: [] for d in drones}
        if n > 1:
            positions = np.array([d.position for d in drones])
            diff = positions[:, None, :] - positions[None, :, :]
            dist = np.sqrt(np.einsum("ijk,ijk->ij", diff, diff))
            in_range = dist <= self.radius
            np.fill_diagonal(in_range, False)
            for i, drone in enumerate(drones):
                neighbor_map[drone.idx] = [drones[j] for j in np.flatnonzero(in_range[i])]
        self.neighbor_map = neighbor_map
        return neighbor_map

    def broadcast(self, sender_idx: int, msg: DroneMessage) -> None:
        """Consegna il messaggio a tutti i droni nel raggio del mittente, salvo perdite."""
        receivers = self.neighbor_map.get(sender_idx, [])
        self.messages_attempted += len(receivers)
        loss = self.loss_probability
        for receiver in receivers:
            if loss > 0.0 and self.rng.random() < loss:
                continue
            receiver.communication.deliver(sender_idx, msg)
            self.messages_delivered += 1


# ------------------------------------------------------------
# Mondo
# ------------------------------------------------------------

class SimulationWorld:
    """Ambiente: geometria, sensori simulati, canale radio e azioni fisiche.

    Mantiene lo stato fisico globale perché è il simulatore, ma non decide nulla per i droni.
    """

    def __init__(self, cfg: SimConfig, drones: List['Drone'], fires: List[Fire], water_stations: List[np.ndarray],
                 channel: RadioChannel):
        self.cfg = cfg
        self._drones = drones
        self._fires = fires
        self.area_width = cfg.AREA_WIDTH
        self.area_height = cfg.AREA_HEIGHT
        self.water_stations = water_stations  # mappa nota a priori (infrastruttura fissa)
        self.channel = channel
        self.step_counter = 0
        self.extinguished_count = 0
        self.spawned_count = 0          # incendi figli generati dalla propagazione
        self.water_delivered = 0.0      # acqua totale effettivamente applicata sugli incendi

    @property
    def fires(self) -> List[Fire]:
        return self._fires

    def refresh_neighbors(self) -> Dict[int, List['Drone']]:
        """Modello di propagazione radio: chi può ricevere i messaggi di chi.

        È fisica del canale, non coordinamento: il drone non riceve mai questa mappa, solo i messaggi.
        """
        return self.channel.refresh_neighbors(self._drones)

    def broadcast(self, sender_idx: int, msg: DroneMessage) -> None:
        """Consegna il messaggio ai droni nel raggio radio del mittente (con eventuale perdita)."""
        self.channel.broadcast(sender_idx, msg)

    def sense_fires(self, position: np.ndarray) -> List[Fire]:
        """Sensore di bordo: incendi attivi entro FIRE_DETECTION_RADIUS."""
        radius = self.cfg.FIRE_DETECTION_RADIUS
        return [
            fire for fire in self._fires
            if fire.active and np.linalg.norm(position - fire.pos) <= radius
        ]

    def has_active_fire_near(self, position: np.ndarray, radius: float) -> bool:
        return any(fire.active and np.linalg.norm(position - fire.pos) <= radius for fire in self._fires)

    def request_extinguish(self, position: np.ndarray, water_available: float, flow_rate: float) -> Tuple[float, List[FirePos]]:
        """Attuatore: spruzza acqua sugli incendi nel raggio.

        Restituisce l'acqua usata e le posizioni degli incendi spenti da questo getto
        (il drone osserva direttamente l'esito della propria azione).
        """
        remaining = min(max(water_available, 0.0), flow_rate * self.cfg.SIM_TIME_STEP)
        total_used = 0.0
        extinguished: List[FirePos] = []
        for fire in self._fires:
            if remaining <= 1e-12:
                break
            if not fire.active or np.linalg.norm(position - fire.pos) > self.cfg.FIRE_EXTINGUISH_RADIUS:
                continue
            used = fire.extinguish(remaining)
            total_used += used
            remaining -= used
            if not fire.active:
                extinguished.append(vec_to_tuple(fire.pos))
        self.water_delivered += total_used
        if extinguished:
            self.extinguished_count += len(extinguished)
            self._remove_dead_fires()
        return total_used, extinguished

    def update_fires(self, rng: random.Random) -> None:
        cfg = self.cfg
        self.step_counter += 1
        spawn_queue: List[Fire] = []
        for fire in self._fires:
            fire.grow(cfg.SIM_TIME_STEP)
            if fire.health > cfg.FIRE_HEALTH * cfg.FIRE_SPAWN_THRESHOLD:
                if rng.random() < cfg.FIRE_SPAWN_PROB_PER_STEP:
                    angle = rng.uniform(0.0, 2.0 * math.pi)
                    dist = rng.uniform(0.5, cfg.FIRE_SPAWN_OFFSET_MAX)
                    new_pos = fire.pos + dist * unit_from_angle(angle)
                    new_pos[0] = float(np.clip(new_pos[0], cfg.FIRE_GENERATION_MARGIN, self.area_width - cfg.FIRE_GENERATION_MARGIN))
                    new_pos[1] = float(np.clip(new_pos[1], cfg.FIRE_GENERATION_MARGIN, self.area_height - cfg.FIRE_GENERATION_MARGIN))
                    spawn_queue.append(Fire(pos=new_pos, health=cfg.FIRE_HEALTH * cfg.FIRE_SPAWN_INITIAL_HEALTH,
                                            growth_rate=cfg.FIRE_GROWTH_RATE))
        self._fires.extend(spawn_queue)
        self.spawned_count += len(spawn_queue)
        self._remove_dead_fires()

    def _remove_dead_fires(self) -> None:
        self._fires[:] = [fire for fire in self._fires if fire.active]
