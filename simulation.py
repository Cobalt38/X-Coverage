"""
SwarmSimulation: orchestrazione centrale del round (nessuna decisione operativa).
"""

import random
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

from config import DEFAULT_CONFIG, SimConfig
from drone import Drone
from world import Fire, RadioChannel, SimulationWorld


class SwarmSimulation:
    def _default_fires(self) -> List[Fire]:
        cfg = self.cfg
        positions = [
            (cfg.AREA_WIDTH * 0.20, cfg.AREA_HEIGHT * 0.75),
            (cfg.AREA_WIDTH * 0.50, cfg.AREA_HEIGHT * 0.25),
            (cfg.AREA_WIDTH * 0.80, cfg.AREA_HEIGHT * 0.75),
        ]
        return [self._new_fire(np.array(pos, dtype=float)) for pos in positions]

    def _random_fires(self, count: int) -> List[Fire]:
        cfg = self.cfg
        margin = cfg.FIRE_GENERATION_MARGIN
        return [self._new_fire(np.array([self.rng.uniform(margin, cfg.AREA_WIDTH - margin),
                                         self.rng.uniform(margin, cfg.AREA_HEIGHT - margin)], dtype=float))
                for _ in range(count)]

    def _new_fire(self, pos: np.ndarray) -> Fire:
        return Fire(pos=pos, health=self.cfg.FIRE_HEALTH, growth_rate=self.cfg.FIRE_GROWTH_RATE)

    def _default_water_stations(self) -> List[np.ndarray]:
        cfg = self.cfg
        positions = [
            (cfg.AREA_WIDTH * 0.20, cfg.AREA_HEIGHT * 0.25),
            (cfg.AREA_WIDTH * 0.50, cfg.AREA_HEIGHT * 0.75),
            (cfg.AREA_WIDTH * 0.80, cfg.AREA_HEIGHT * 0.25),
        ]
        return [np.array(pos, dtype=float) for pos in positions[:cfg.NUM_WATER_STATIONS]]

    def _random_water_stations(self, count: int) -> List[np.ndarray]:
        cfg = self.cfg
        margin = cfg.WATER_STATION_GENERATION_MARGIN
        stations: List[np.ndarray] = []
        for _ in range(count * 100):
            if len(stations) >= count:
                break
            pos = np.array([self.rng.uniform(margin, cfg.AREA_WIDTH - margin),
                            self.rng.uniform(margin, cfg.AREA_HEIGHT - margin)], dtype=float)
            if all(np.linalg.norm(pos - other) >= cfg.WATER_STATION_MIN_SEPARATION for other in stations):
                stations.append(pos)
        return stations if len(stations) == count else self._default_water_stations()[:count]

    def __init__(self, seed: Optional[int] = None, random_fires: bool = False, random_stations: bool = False,
                 log_collisions: bool = False, cfg: SimConfig = DEFAULT_CONFIG):
        self.cfg = cfg
        self.seed = cfg.RANDOM_SEED if seed is None else seed
        self.log_collisions = log_collisions

        self.rng = random.Random(self.seed)
        self.fires = self._random_fires(cfg.NUM_FIRES) if random_fires else self._default_fires()
        self.water_stations = self._random_water_stations(cfg.NUM_WATER_STATIONS) if random_stations else self._default_water_stations()

        self.drones: List[Drone] = []
        self.channel = RadioChannel(cfg.COMMUNICATION_RADIUS, cfg.PACKET_LOSS, self.seed)
        self.world = SimulationWorld(cfg, self.drones, self.fires, self.water_stations, self.channel)
        self.drones.extend(Drone(i, self.rng, self.world, self.seed) for i in range(cfg.NUM_DRONES))

        self.step_count = 0
        self.step_collisions = 0        # coppie in contatto in questo step
        self.total_collisions = 0       # eventi di collisione (una coppia che ENTRA in contatto conta 1)
        self.contact_steps = 0          # step-coppia trascorsi in contatto (durata complessiva)
        self._pairs_in_contact: Set[Tuple[int, int]] = set()
        self.last_neighbors: Dict[int, List[Drone]] = {}
        # Distanze tra tutte le coppie (i < j) a fine step, riusate dalle metriche
        self.pair_distances = np.zeros(0, dtype=float)
        self._pair_i, self._pair_j = np.triu_indices(len(self.drones), k=1)

    @property
    def sim_time(self) -> float:
        return self.step_count * self.cfg.SIM_TIME_STEP

    @property
    def all_fires_extinguished(self) -> bool:
        """Stato assorbente: senza incendi attivi non può nascerne nessuno."""
        return not self.world.fires

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
        positions = np.array([d.position for d in self.drones])
        diff = positions[self._pair_i] - positions[self._pair_j]
        self.pair_distances = np.sqrt(np.einsum("ij,ij->i", diff, diff))

        in_contact = np.flatnonzero(self.pair_distances < self.cfg.DRONE_IMPACT_RADIUS)
        self.step_collisions = len(in_contact)
        self.contact_steps += len(in_contact)
        current: Set[Tuple[int, int]] = set()
        for k in in_contact:
            pair = (self.drones[self._pair_i[k]].idx, self.drones[self._pair_j[k]].idx)
            current.add(pair)
            if pair not in self._pairs_in_contact:
                self.total_collisions += 1
                if self.log_collisions:
                    print(f"[collision] step={self.step_count} t={self.sim_time:.2f}s ids={pair} "
                          f"dist={self.pair_distances[k]:.4f}")
        self._pairs_in_contact = current

    def summary(self) -> str:
        return (f"t={self.sim_time:.1f}s  incendi attivi={len(self.world.fires)}  spenti={self.world.extinguished_count}  "
                f"collisioni={self.total_collisions}  step-contatto={self.contact_steps}")
