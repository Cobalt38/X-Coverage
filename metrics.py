"""
Misure di una simulazione.

Due parti:
    1. METRICS: l'elenco di tutto ciò che si misura, con nome leggibile, unità, se è meglio un valore
       alto o basso, e una spiegazione. È la "legenda" usata anche per scrivere i report.
    2. MetricsCollector: osserva la simulazione dopo ogni passo e calcola le metriche.
       Legge soltanto: non modifica la simulazione e non usa numeri casuali, quindi misurare
       non cambia ciò che succede.

    sim = SwarmSimulation(...)
    metrics = MetricsCollector(sim)
    for _ in range(steps):
        sim.step()
        metrics.on_step()
    risultati = metrics.finalize()     # {"mission_complete": True, "collisions": 0, ...}

Un valore None significa "non definito in questa simulazione" (es. il tempo di spegnimento
di una missione non riuscita).
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

from simulation import SwarmSimulation
from world import vec_to_tuple


# ============================================================
# 1. Elenco delle metriche
# ============================================================

@dataclass(frozen=True)
class Metric:
    key: str            # nome della colonna nel CSV
    label: str          # nome leggibile nel report
    unit: str           # "%" = frazione mostrata in percentuale, "sì/no" = vero/falso
    better: str         # "alto", "basso" oppure "" se non c'è un valore migliore
    group: str
    description: str


METRICS: List[Metric] = [
    # --- Missione
    Metric("mission_complete", "Missione riuscita", "sì/no", "alto", "Missione",
           "Tutti gli incendi sono stati spenti entro la durata massima."),
    Metric("fire_overrun", "Incendi fuori controllo", "sì/no", "basso", "Missione",
           "La simulazione è stata interrotta perché gli incendi accesi erano troppi."),
    Metric("extinction_time_s", "Tempo per spegnere tutto", "s", "basso", "Missione",
           "Secondi fino allo spegnimento dell'ultimo incendio (solo missioni riuscite)."),
    Metric("fire_damage", "Danno degli incendi", "vita·s", "basso", "Missione",
           "Somma nel tempo della vita di tutti gli incendi accesi: premia chi spegne presto."),
    Metric("fires_spawned", "Incendi nati dalla propagazione", "", "basso", "Missione",
           "Un incendio lasciato crescere ne genera altri vicino a sé."),
    Metric("peak_active_fires", "Picco di incendi accesi", "", "basso", "Missione",
           "Massimo numero di incendi accesi contemporaneamente."),
    Metric("detection_delay_s", "Ritardo di avvistamento", "s", "basso", "Missione",
           "Tempo medio tra la nascita di un incendio e il primo drone che lo vede."),
    Metric("response_delay_s", "Ritardo di intervento", "s", "basso", "Missione",
           "Tempo medio tra l'avvistamento di un incendio e la prima acqua che riceve."),

    # --- Sicurezza
    Metric("collisions", "Collisioni", "", "basso", "Sicurezza",
           "Coppie di droni entrate a contatto (distanza < DRONE_IMPACT_RADIUS)."),
    Metric("near_misses", "Quasi-collisioni", "", "basso", "Sicurezza",
           "Coppie di droni scese sotto la distanza di emergenza (EMERGENCY_AVOID_DISTANCE)."),
    Metric("min_distance_m", "Distanza minima tra due droni", "m", "alto", "Sicurezza",
           "La distanza più piccola mai registrata tra due droni."),
    Metric("emergency_fraction", "Tempo in emergenza", "%", "basso", "Sicurezza",
           "Quota del tempo in cui i droni si scansano invece di lavorare."),

    # --- Come i droni usano il tempo (le sei voci sommano al 100%)
    Metric("time_explore", "Esplorazione", "%", "", "Uso del tempo",
           "Nessun compito: il drone perlustra l'area."),
    Metric("time_to_fire", "In volo verso un incendio", "%", "", "Uso del tempo",
           "Ha scelto un incendio e ci sta andando."),
    Metric("time_extinguishing", "Spegnimento", "%", "alto", "Uso del tempo",
           "Sta spruzzando acqua su un incendio."),
    Metric("time_to_station", "In volo verso una stazione", "%", "basso", "Uso del tempo",
           "Acqua quasi finita: sta andando a rifornirsi."),
    Metric("time_queue", "In coda alla stazione", "%", "basso", "Uso del tempo",
           "Arrivato alla stazione, aspetta che si liberi un posto."),
    Metric("time_refill", "Rifornimento", "%", "", "Uso del tempo",
           "Sta caricando acqua."),

    # --- Lavoro ed efficienza
    Metric("water_fairness", "Equità del lavoro", "", "alto", "Lavoro ed efficienza",
           "Indice di Jain sull'acqua erogata da ogni drone: 1 = tutti lavorano uguale, verso 0 = pochi fanno tutto."),
    Metric("overcrowding_s", "Sovraffollamento sugli incendi", "s", "basso", "Lavoro ed efficienza",
           "Secondi in cui su un incendio lavoravano più di MAX_DRONES_ON_FIRE droni."),
    Metric("queue_wait_mean_s", "Attesa media in coda", "s", "basso", "Lavoro ed efficienza",
           "Attesa media alla stazione idrica per ogni rifornimento."),
    Metric("queue_wait_max_s", "Attesa massima in coda", "s", "basso", "Lavoro ed efficienza",
           "L'attesa più lunga registrata a una stazione idrica."),
    Metric("distance_m", "Distanza percorsa", "m", "basso", "Lavoro ed efficienza",
           "Metri percorsi da tutti i droni insieme."),
    Metric("control_effort", "Sforzo di controllo", "m²/s³", "basso", "Lavoro ed efficienza",
           "Quanto i droni hanno accelerato e frenato (∫|a|² dt): approssima il consumo di batteria."),
    Metric("saturation_bounces", "Rimbalzi da incendi pieni", "", "", "Lavoro ed efficienza",
           "Volte in cui un drone si è allontanato da un incendio già occupato da abbastanza droni."),

    # --- Comunicazione e conoscenza
    Metric("message_loss", "Messaggi persi", "%", "basso", "Comunicazione e conoscenza",
           "Quota dei messaggi radio che non sono arrivati."),
    Metric("neighbors", "Vicini radio per drone", "", "alto", "Comunicazione e conoscenza",
           "Quanti altri droni sente in media ogni drone (messaggi effettivamente ricevuti)."),
    Metric("connected_fraction", "Sciame tutto connesso", "%", "alto", "Comunicazione e conoscenza",
           "Quota del tempo in cui ogni drone può raggiungere ogni altro, anche passando per altri."),
    Metric("fire_awareness", "Incendi noti ai droni", "%", "alto", "Comunicazione e conoscenza",
           "In media, quale quota degli incendi accesi conosce ogni drone."),
    Metric("phantom_fires", "Incendi fantasma per drone", "", "basso", "Comunicazione e conoscenza",
           "Incendi che un drone crede accesi ma sono già stati spenti."),
    Metric("info_age_s", "Età delle informazioni", "s", "basso", "Comunicazione e conoscenza",
           "Quanto sono vecchie, in media, le notizie che i droni hanno sugli incendi accesi."),
]

METRICS_BY_KEY = {m.key: m for m in METRICS}

# Come classificare il tempo di un drone (vedi le metriche "time_*")
_TASKS = ("explore", "to_fire", "extinguishing", "to_station", "queue", "refill")


# ============================================================
# 2. Raccolta
# ============================================================

class MetricsCollector:
    """Accumula le misure passo per passo; finalize() restituisce un valore per ogni metrica di METRICS."""

    def __init__(self, sim: SwarmSimulation, network_sample_every_s: float = 0.1):
        self.sim = sim
        self.cfg = sim.cfg
        self.dt = sim.cfg.SIM_TIME_STEP
        n = len(sim.drones)
        self._steps = 0
        self._prev_pos = self._drone_array("position")
        self._prev_water = self._drone_array("water")

        # Missione: per ogni incendio attivo (chiave id) -> [oggetto Fire, nascita, avvistamento, prima acqua, vita al passo prima]
        self._fires: Dict[int, list] = {}
        self._detection_delays: List[float] = []
        self._response_delays: List[float] = []
        self.fire_damage = 0.0
        self.peak_active_fires = len(sim.world.fires)
        self.extinction_time: Optional[float] = None
        self.overcrowding = 0.0
        self.fire_overrun = False   # impostato da chi esegue la simulazione, se la interrompe
        for fire in sim.world.fires:
            self._fires[id(fire)] = [fire, 0.0, None, None, fire.health]

        # Sicurezza
        self.min_distance = math.inf
        self.near_misses = 0
        self._pairs_near = np.zeros(n * (n - 1) // 2, dtype=bool)
        self.emergency_time = 0.0

        # Movimento
        self.distance = 0.0
        self.control_effort = 0.0

        # Uso del tempo e rifornimenti
        self.task_time = {task: 0.0 for task in _TASKS}
        self._queue_wait: Dict[int, float] = {}     # drone in rifornimento -> attesa accumulata
        self.queue_waits: List[float] = []           # attese dei rifornimenti conclusi

        # Rete e conoscenza (campionate: scorrono la memoria di tutti i droni)
        self._network_every = max(1, round(network_sample_every_s / self.dt))
        self._samples = 0
        self._neighbors_sum = 0.0
        self._connected_samples = 0
        self._phantom_sum = 0.0
        self._awareness_sum = 0.0
        self._awareness_samples = 0
        self._info_age_sum = 0.0
        self._info_age_n = 0

    def _drone_array(self, attribute: str) -> np.ndarray:
        return np.array([getattr(d, attribute) for d in self.sim.drones], dtype=float)

    # --------------------------------------------------------
    # Passo
    # --------------------------------------------------------

    def on_step(self) -> None:
        """Da chiamare dopo ogni sim.step()."""
        self._steps += 1
        t = self.sim.sim_time
        pos = self._drone_array("position")
        water = self._drone_array("water")
        accel = np.linalg.norm(self._drone_array("acceleration"), axis=1)
        refilled = water > self._prev_water + 1e-12   # l'acqua cresce solo durante il rifornimento

        self.distance += float(np.linalg.norm(pos - self._prev_pos, axis=1).sum())
        self.control_effort += float((accel ** 2).sum()) * self.dt
        self.emergency_time += sum(1 for d in self.sim.drones if d.in_emergency) * self.dt

        extinguishing = self._update_fires(t, pos, water)
        self._update_safety()
        self._update_time_budget(pos, refilled, extinguishing)
        if self._steps % self._network_every == 0:
            self._sample_network()

        self._prev_pos = pos
        self._prev_water = water

    def _update_fires(self, t: float, pos: np.ndarray, water: np.ndarray) -> np.ndarray:
        """Nascita, avvistamento, prima acqua e spegnimento di ogni incendio. Restituisce chi sta spegnendo."""
        cfg = self.cfg
        fires = self.sim.world.fires
        current = {id(f) for f in fires}
        for key in [k for k in self._fires if k not in current]:
            del self._fires[key]                              # spento
        for fire in fires:
            if id(fire) not in self._fires:
                self._fires[id(fire)] = [fire, t, None, None, fire.health]   # appena nato

        if not fires:
            if self.extinction_time is None:
                self.extinction_time = t
            return np.zeros(len(pos), dtype=bool)

        fire_pos = np.array([f.pos for f in fires])
        dist = np.linalg.norm(pos[:, None, :] - fire_pos[None, :, :], axis=2)   # (droni, incendi)
        seen = (dist <= cfg.FIRE_DETECTION_RADIUS).any(axis=0)
        working = (dist <= cfg.FIRE_EXTINGUISH_RADIUS) & (water > 0.0)[:, None]
        workers = working.sum(axis=0)

        for k, fire in enumerate(fires):
            record = self._fires[id(fire)]
            _, born, detected, first_water, last_health = record
            if detected is None and seen[k]:
                record[2] = detected = t
                self._detection_delays.append(t - born)
            if first_water is None and fire.health < last_health:
                record[3] = t
                if detected is not None:
                    self._response_delays.append(t - detected)
            record[4] = fire.health
            if workers[k] > cfg.MAX_DRONES_ON_FIRE:
                self.overcrowding += self.dt

        self.fire_damage += sum(f.health for f in fires) * self.dt
        self.peak_active_fires = max(self.peak_active_fires, len(fires))
        return working.any(axis=1)

    def _update_safety(self) -> None:
        d = self.sim.pair_distances    # distanze tra tutte le coppie, calcolate dalla simulazione
        if not d.size:
            return
        self.min_distance = min(self.min_distance, float(d.min()))
        near = d < self.cfg.EMERGENCY_AVOID_DISTANCE
        self.near_misses += int(np.count_nonzero(near & ~self._pairs_near))   # conta solo chi ENTRA
        self._pairs_near = near

    def _update_time_budget(self, pos: np.ndarray, refilled: np.ndarray, extinguishing: np.ndarray) -> None:
        """Assegna il passo di ogni drone a una sola attività, e misura le attese in coda."""
        cfg = self.cfg
        arrival_radius = cfg.WATER_STATION_WAIT_RADIUS + cfg.TARGET_REACHED_DISTANCE
        for i, drone in enumerate(self.sim.drones):
            in_reload = drone.reloading or refilled[i] or i in self._queue_wait
            if in_reload and i not in self._queue_wait:
                self._queue_wait[i] = 0.0                     # inizio di un rifornimento

            if refilled[i]:
                task = "refill"
            elif drone.reloading:
                station = self.sim.water_stations[drone.water_station_idx]
                at_station = np.linalg.norm(pos[i] - station) <= arrival_radius
                if at_station and drone.station_slot is None:
                    task = "queue"
                    self._queue_wait[i] += self.dt
                else:
                    task = "to_station"
            elif extinguishing[i]:
                task = "extinguishing"
            elif drone.fire_target is not None:
                task = "to_fire"
            else:
                task = "explore"
            self.task_time[task] += self.dt

            if i in self._queue_wait and not drone.reloading:
                self.queue_waits.append(self._queue_wait.pop(i))   # rifornimento concluso

    def _sample_network(self) -> None:
        sim = self.sim
        n = len(sim.drones)
        self._samples += 1
        self._neighbors_sum += sum(len(d.communication.neighbors) for d in sim.drones) / n
        self._connected_samples += _is_connected(sim.last_neighbors)

        active = {vec_to_tuple(f.pos) for f in sim.world.fires}
        phantom = awareness = 0.0
        for drone in sim.drones:
            known = drone.known_fires
            phantom += sum(1 for p in known if p not in active)
            if active:
                awareness += sum(1 for p in active if p in known) / len(active)
                ages = [age for p, age in known.items() if p in active]
                self._info_age_sum += sum(ages) * self.dt
                self._info_age_n += len(ages)
        self._phantom_sum += phantom / n
        if active:
            self._awareness_sum += awareness / n
            self._awareness_samples += 1

    # --------------------------------------------------------
    # Risultato
    # --------------------------------------------------------

    def finalize(self) -> Dict[str, Any]:
        sim = self.sim
        drone_time = len(sim.drones) * sim.sim_time
        channel = sim.channel
        water = [d.water_delivered for d in sim.drones]
        values = {
            "mission_complete": self.extinction_time is not None,
            "fire_overrun": self.fire_overrun,
            "extinction_time_s": self.extinction_time,
            "fire_damage": self.fire_damage,
            "fires_spawned": sim.world.spawned_count,
            "peak_active_fires": self.peak_active_fires,
            "detection_delay_s": _mean(self._detection_delays),
            "response_delay_s": _mean(self._response_delays),
            "collisions": sim.total_collisions,
            "near_misses": self.near_misses,
            "min_distance_m": self.min_distance if math.isfinite(self.min_distance) else None,
            "emergency_fraction": _ratio(self.emergency_time, drone_time),
            **{f"time_{task}": _ratio(self.task_time[task], drone_time) for task in _TASKS},
            "water_fairness": _jain_index(water),
            "overcrowding_s": self.overcrowding,
            "queue_wait_mean_s": _mean(self.queue_waits),
            "queue_wait_max_s": max(self.queue_waits) if self.queue_waits else None,
            "distance_m": self.distance,
            "control_effort": self.control_effort,
            "saturation_bounces": sum(d.bounce_count for d in sim.drones),
            "message_loss": _ratio(channel.messages_dropped, channel.messages_attempted),
            "neighbors": _ratio(self._neighbors_sum, self._samples),
            "connected_fraction": _ratio(self._connected_samples, self._samples),
            "fire_awareness": _ratio(self._awareness_sum, self._awareness_samples),
            "phantom_fires": _ratio(self._phantom_sum, self._samples),
            "info_age_s": _ratio(self._info_age_sum, self._info_age_n),
        }
        assert list(values) == [m.key for m in METRICS], "METRICS e finalize() devono elencare le stesse metriche"
        return values


# ------------------------------------------------------------
# Funzioni di supporto
# ------------------------------------------------------------

def _mean(values: List[float]) -> Optional[float]:
    return float(np.mean(values)) if values else None


def _ratio(num: float, den: float) -> Optional[float]:
    return num / den if den else None


def _jain_index(values: List[float]) -> Optional[float]:
    """Indice di equità di Jain: (Σx)² / (n·Σx²). Vale 1 se tutti i valori sono uguali, 1/n se uno solo è > 0."""
    x = np.asarray(values, dtype=float)
    denom = len(x) * float(np.sum(x * x))
    return float(np.sum(x) ** 2 / denom) if denom > 0 else None


def _is_connected(neighbors: Dict[int, list]) -> bool:
    """Il grafo radio è connesso? (visita a partire da un drone qualsiasi)"""
    if not neighbors:
        return True
    start = next(iter(neighbors))
    seen = {start}
    stack = [start]
    while stack:
        for other in neighbors.get(stack.pop(), []):
            if other.idx not in seen:
                seen.add(other.idx)
                stack.append(other.idx)
    return len(seen) == len(neighbors)
