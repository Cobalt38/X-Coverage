"""
GLI ESPERIMENTI: misurare lo sciame e confrontare varianti del sistema.

Un esperimento risponde a UNA domanda del tipo "questo meccanismo serve davvero?". Il modo di
rispondere è sempre lo stesso: si simula la stessa identica situazione con e senza il meccanismo,
molte volte, e si guarda se i risultati differiscono più di quanto potrebbe fare il caso.

Il file segue quel percorso, dall'alto in basso:

    1. COSA SI MISURA      l'elenco delle misure (METRICS) e l'oggetto che le raccoglie
                           passo dopo passo mentre la simulazione va avanti (Measurements).
    2. COSA SI CONFRONTA   scenari, varianti, esperimenti. Per aggiungerne uno basta una riga
                           nel dizionario EXPERIMENTS.
    3. COME SI ESEGUE      una simulazione per volta (run_one_simulation), tante in parallelo
                           (run_experiment).
    4. COME SI RIASSUME    media e intervallo di confidenza di ogni misura.
    5. COME SI CONFRONTA   il confronto a coppie con il riferimento, e il calcolo che dice se la
                           differenza è reale o è fortuna.
    6. IL REPORT           il file Markdown finale e il CSV con i dati grezzi.

Si lancia da main.py:   python main.py experiment ablation --runs 30
"""

import csv
import datetime
import itertools
import math
import os
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from drone import CoverageMemory
from simulation import MissionOutcome, MissionResult, Simulation
from world import DEFAULT_CONFIG, SimConfig, validate_config, vec_to_tuple

RESULTS_DIR = "results"
FIRST_SEED = 1            # le simulazioni usano i seed 1, 2, 3, ...
SIGNIFICANCE = 0.05       # sotto questa probabilità una differenza è considerata reale


# ============================================================
# 1. COSA SI MISURA
# ============================================================

@dataclass(frozen=True)
class Metric:
    """Una misura: come si chiama nel CSV, come si legge nel report, e cosa significa."""
    key: str             # nome della colonna nel file CSV
    label: str           # nome per esteso, usato nelle tabelle
    unit: str            # "s", "m", "%" (frazione mostrata in percentuale), "sì/no", oppure ""
    better: str          # "alto", "basso", o "" se non esiste un valore migliore
    group: str           # sezione del report in cui compare
    description: str


METRICS: List[Metric] = [
    # --- Com'è andata la missione
    Metric("mission_complete", "Missione riuscita", "sì/no", "alto", "Missione",
           "Tutti gli incendi sono stati spenti entro il tempo massimo."),
    Metric("fire_overrun", "Incendi fuori controllo", "sì/no", "basso", "Missione",
           "La simulazione è stata interrotta perché gli incendi accesi erano troppi."),
    Metric("extinction_time_s", "Tempo per spegnere tutto", "s", "basso", "Missione",
           "Secondi fino all'ultimo incendio spento. Ha senso solo per le missioni riuscite."),
    Metric("fire_damage", "Danno degli incendi", "vita·s", "", "Missione",
           "Somma, istante per istante, della vita di tutti gli incendi accesi. ATTENZIONE: "
           "cresce con la durata, e una missione che collassa presto si ferma prima. Per "
           "confrontare varianti usare la misura seguente, che è già divisa per il tempo."),
    Metric("burning_health_mean", "Fuoco acceso in media", "vita", "basso", "Missione",
           "Quanta vita di incendi era accesa in media, istante per istante: il danno diviso per "
           "la durata. Confrontabile tra missioni di durata diversa."),
    Metric("fires_spawned", "Incendi nati dalla propagazione", "", "basso", "Missione",
           "Un incendio lasciato crescere troppo ne genera altri vicino a sé."),
    Metric("fires_ignited", "Incendi nati da soli", "", "", "Missione",
           "Accensioni spontanee, indipendenti dagli incendi già presenti. Dipendono solo dal "
           "seed, quindi a parità di seed sono le stesse per tutte le varianti."),
    Metric("peak_active_fires", "Picco di incendi accesi", "", "basso", "Missione",
           "Il massimo numero di incendi accesi nello stesso momento."),
    Metric("detection_delay_s", "Ritardo di avvistamento", "s", "basso", "Missione",
           "Tempo medio tra la nascita di un incendio e il primo drone che lo vede. Va letto "
           "insieme alla misura seguente: una strategia che ignora del tutto certe zone MIGLIORA "
           "questa media, perché gli incendi che non trova mai non entrano nel conto."),
    Metric("fires_never_seen", "Incendi mai avvistati", "", "basso", "Missione",
           "Incendi ancora accesi alla fine che nessun drone ha mai visto."),
    Metric("response_delay_s", "Ritardo di intervento", "s", "basso", "Missione",
           "Tempo medio tra l'avvistamento di un incendio e la prima acqua che riceve."),

    # --- Sicurezza dello sciame
    Metric("collisions", "Urti", "", "basso", "Sicurezza",
           "Quante volte due droni si sono toccati (distanza sotto DRONE_IMPACT_RADIUS)."),
    Metric("drones_lost", "Droni fuori uso", "", "basso", "Sicurezza",
           "Droni precipitati in seguito a un urto: non volano più per il resto della missione."),
    Metric("near_misses", "Quasi-urti", "", "basso", "Sicurezza",
           "Quante volte due droni sono scesi sotto la distanza di emergenza senza toccarsi."),
    Metric("min_distance_m", "Distanza minima tra due droni", "m", "alto", "Sicurezza",
           "La distanza più piccola mai registrata tra due droni in volo."),
    Metric("emergency_fraction", "Tempo in emergenza", "%", "basso", "Sicurezza",
           "Quota di tempo passata a scansarsi invece che a lavorare."),

    # --- Come i droni usano il tempo (le voci sommano al 100%)
    Metric("time_patrol", "Perlustrazione", "%", "", "Uso del tempo",
           "Nessun compito: il drone sta cercando incendi."),
    Metric("time_to_fire", "In volo verso un incendio", "%", "", "Uso del tempo",
           "Ha scelto un incendio e ci sta andando."),
    Metric("time_extinguishing", "Spegnimento", "%", "alto", "Uso del tempo",
           "Sta spruzzando acqua su un incendio: è l'unico momento in cui fa il suo mestiere."),
    Metric("time_to_station", "In volo verso una stazione", "%", "basso", "Uso del tempo",
           "Acqua quasi finita: sta andando a rifornirsi."),
    Metric("time_queue", "In coda alla stazione", "%", "basso", "Uso del tempo",
           "È arrivato alla stazione e aspetta che si liberi un posto."),
    Metric("time_refill", "Rifornimento", "%", "", "Uso del tempo",
           "Sta caricando acqua."),
    Metric("time_out_of_service", "Fuori uso", "%", "basso", "Uso del tempo",
           "Tempo passato a terra dopo un urto."),

    # --- Efficienza
    Metric("water_fairness", "Equità del lavoro", "", "alto", "Lavoro ed efficienza",
           "Quanto equamente i droni si dividono il lavoro (indice di Jain sull'acqua erogata): "
           "1 = tutti uguali, verso 0 = pochi fanno tutto."),
    Metric("overcrowding_s", "Sovraffollamento sugli incendi", "s", "basso", "Lavoro ed efficienza",
           "Secondi in cui su un incendio lavoravano più droni del limite MAX_DRONES_ON_FIRE."),
    Metric("queue_wait_mean_s", "Attesa media in coda", "s", "basso", "Lavoro ed efficienza",
           "Quanto si aspetta, in media, prima di poter caricare acqua."),
    Metric("queue_wait_max_s", "Attesa massima in coda", "s", "basso", "Lavoro ed efficienza",
           "L'attesa più lunga registrata a una stazione."),
    Metric("distance_per_drone_m", "Distanza per drone", "m", "", "Lavoro ed efficienza",
           "Metri percorsi in media da ogni drone ancora in volo, al netto della durata "
           "(metri per drone al secondo × durata). Non ha un verso migliore: volare di più non "
           "è né buono né cattivo di per sé."),
    Metric("control_effort_rate", "Sforzo di controllo", "m²/s³", "basso", "Lavoro ed efficienza",
           "Quanto ogni drone accelera e frena, al secondo: approssima il consumo di batteria. "
           "Diviso per i secondi di volo effettivi, quindi confrontabile tra missioni diverse."),
    Metric("saturation_bounces", "Rimbalzi da incendi affollati", "", "", "Lavoro ed efficienza",
           "Quante volte un drone si è allontanato da un incendio dove era di troppo."),

    # --- Comunicazione e conoscenza condivisa
    Metric("message_loss", "Messaggi persi", "%", "basso", "Comunicazione e conoscenza",
           "Quota dei messaggi radio che non sono arrivati a destinazione."),
    Metric("neighbors", "Vicini radio per drone", "", "alto", "Comunicazione e conoscenza",
           "Quanti altri droni sente in media ciascuno."),
    Metric("connected_fraction", "Sciame tutto connesso", "%", "alto", "Comunicazione e conoscenza",
           "Quota di tempo in cui ogni drone può raggiungere ogni altro, anche passando per altri."),
    Metric("fire_awareness", "Incendi noti ai droni", "%", "alto", "Comunicazione e conoscenza",
           "In media, quale quota degli incendi accesi conosce ciascun drone."),
    Metric("phantom_fires", "Incendi fantasma per drone", "", "basso", "Comunicazione e conoscenza",
           "Incendi che un drone crede accesi ma che sono già stati spenti."),
    Metric("info_age_s", "Età delle informazioni", "s", "basso", "Comunicazione e conoscenza",
           "Quanto sono vecchie, in media, le notizie che i droni hanno sugli incendi accesi."),

    # --- Perlustrazione del terreno
    Metric("coverage_staleness_s", "Obsolescenza del terreno", "s", "basso", "Perlustrazione",
           "Da quanto tempo, in media, una zona non viene guardata da nessuno, pesando ogni zona "
           "per la sua importanza. È l'obiettivo classico della copertura persistente."),
    Metric("coverage_staleness_hot_s", "Obsolescenza delle zone importanti", "s", "basso", "Perlustrazione",
           "Come sopra, ma solo sulle zone di valore alto (importanza ≥ 0.5)."),
    Metric("coverage_staleness_cold_s", "Obsolescenza del resto dell'area", "s", "basso", "Perlustrazione",
           "Come sopra, sulle zone di valore basso. Insieme alla precedente mostra come una "
           "strategia distribuisce l'attenzione: concentrarsi sulle zone importanti peggiora questa."),
    Metric("coverage_staleness_max_s", "Obsolescenza della zona peggiore", "s", "basso", "Perlustrazione",
           "La zona importante lasciata più a lungo senza controllo."),
]

METRICS_BY_KEY = {metric.key: metric for metric in METRICS}

# Le attività tra cui si divide il tempo di un drone (le misure "time_*").
ACTIVITIES = ("patrol", "to_fire", "extinguishing", "to_station", "queue", "refill", "out_of_service")


class Measurements:
    """Osserva una simulazione e ne registra tutto il misurabile.

    È un "osservatore" (vedi simulation.Watcher): la simulazione lo chiama dopo ogni passo. Legge
    soltanto, non tocca niente e non usa numeri casuali, quindi misurare non cambia l'esito.

        simulation = Simulation(seed=1)
        measurements = Measurements(simulation)
        simulation.run(max_time_s=300, watchers=[measurements])
        results = measurements.results()      # {"collisions": 0, "fire_damage": 12345.6, ...}
    """

    # Ogni incendio viene seguito da quando nasce: [oggetto, nascita, avvistamento, prima acqua, vita precedente]
    BORN, FIRST_SEEN, FIRST_WATER, LAST_HEALTH = 1, 2, 3, 4

    def __init__(self, simulation: Simulation, network_sample_every_s: float = 0.1):
        self.simulation = simulation
        self.config = simulation.config
        self.dt = simulation.config.SIM_TIME_STEP
        self.steps = 0

        drone_count = len(simulation.drones)
        self._previous_positions = self._read("position")
        self._previous_water = self._read("water")

        # Missione
        self._fire_records: Dict[int, list] = {fire.uid: [fire, 0.0, None, None, fire.health]
                                               for fire in simulation.world.fires}
        self._detection_delays: List[float] = []
        self._response_delays: List[float] = []
        self.fire_damage = 0.0
        self.peak_active_fires = len(simulation.world.fires)
        self.extinction_time: Optional[float] = None
        self.overcrowding_seconds = 0.0

        # Sicurezza
        self.min_distance = math.inf
        self.near_misses = 0
        self._pairs_already_close = np.zeros(drone_count * (drone_count - 1) // 2, dtype=bool)
        self.emergency_seconds = 0.0

        # Movimento
        self.distance_travelled = 0.0
        self.control_effort = 0.0
        self.flight_seconds = 0.0      # secondi-drone effettivamente volati (esclude chi è a terra)

        # Uso del tempo e code
        self.activity_seconds = {activity: 0.0 for activity in ACTIVITIES}
        self._queue_wait: Dict[int, float] = {}
        self.completed_queue_waits: List[float] = []

        # Perlustrazione: mappa "chi ha guardato dove", ricostruita dal simulatore e indipendente
        # da quella dei droni. Serve a giudicare la strategia, non a farla funzionare.
        self._ground_truth_coverage = CoverageMemory(simulation.terrain.shape)
        self._importance = simulation.terrain.grid
        self._importance_total = float(self._importance.sum())
        self._important_zones = self._importance >= 0.5
        self._staleness_sum = 0.0
        self._staleness_hot_sum = 0.0
        self._staleness_cold_sum = 0.0
        self._staleness_samples = 0
        self.worst_staleness = 0.0

        # Rete e conoscenza: si campionano ogni tanto, perché richiedono di scorrere la memoria
        # di tutti i droni e sarebbe inutilmente costoso farlo cento volte al secondo.
        self._network_every = max(1, round(network_sample_every_s / self.dt))
        self._network_samples = 0
        self._neighbors_sum = 0.0
        self._connected_samples = 0
        self._phantom_sum = 0.0
        self._awareness_sum = 0.0
        self._awareness_samples = 0
        self._info_age_sum = 0.0
        self._info_age_count = 0

    def _read(self, attribute: str) -> np.ndarray:
        return np.array([getattr(drone, attribute) for drone in self.simulation.drones], dtype=float)

    # --- Chiamato dopo ogni passo -------------------------------------------

    def after_step(self, simulation: Simulation) -> None:
        self.steps += 1
        now = simulation.sim_time
        positions = self._read("position")
        water = self._read("water")
        accelerations = np.linalg.norm(self._read("acceleration"), axis=1)
        refilling = water > self._previous_water + 1e-12      # l'acqua cresce solo alla stazione
        flying = np.array([drone.is_flying for drone in simulation.drones])

        self.flight_seconds += float(flying.sum()) * self.dt
        self.distance_travelled += float(np.linalg.norm(positions - self._previous_positions, axis=1).sum())
        self.control_effort += float((accelerations ** 2).sum()) * self.dt
        self.emergency_seconds += sum(1 for drone in simulation.drones
                                      if drone.is_flying and drone.in_emergency) * self.dt

        extinguishing = self._update_fires(now, positions, water, flying)
        self._update_safety()
        self._update_activities(positions, refilling, extinguishing, flying)
        self._update_coverage(positions[flying] if flying.any() else np.zeros((0, 2)))
        if self.steps % self._network_every == 0:
            self._sample_network()

        self._previous_positions = positions
        self._previous_water = water

    # --- Incendi: nascita, avvistamento, prima acqua, spegnimento -----------

    def _update_fires(self, now: float, positions: np.ndarray, water: np.ndarray,
                      flying: np.ndarray) -> np.ndarray:
        """Aggiorna la storia di ogni incendio. Restituisce quali droni stanno spegnendo."""
        fires = self.simulation.world.fires
        alive_ids = {fire.uid for fire in fires}
        for fire_id in [fire_id for fire_id in self._fire_records if fire_id not in alive_ids]:
            del self._fire_records[fire_id]                 # spento: esce dal registro
        for fire in fires:
            if fire.uid not in self._fire_records:
                self._fire_records[fire.uid] = [fire, now, None, None, fire.health]   # appena nato

        if not fires:
            if self.extinction_time is None and self.config.IGNITION_RATE_PER_S <= 0.0:
                self.extinction_time = now
            return np.zeros(len(positions), dtype=bool)

        fire_positions = np.array([fire.pos for fire in fires])
        distances = np.linalg.norm(positions[:, None, :] - fire_positions[None, :, :], axis=2)
        # Un drone a terra non vede e non spegne: conta solo chi vola.
        visible = ((distances <= self.config.FIRE_DETECTION_RADIUS) & flying[:, None]).any(axis=0)
        watering = (distances <= self.config.FIRE_EXTINGUISH_RADIUS) & (water > 0.0)[:, None] & flying[:, None]

        for index, fire in enumerate(fires):
            record = self._fire_records[fire.uid]
            if record[self.FIRST_SEEN] is None and visible[index]:
                record[self.FIRST_SEEN] = now
                self._detection_delays.append(now - record[self.BORN])
            if record[self.FIRST_WATER] is None and fire.health < record[self.LAST_HEALTH]:
                record[self.FIRST_WATER] = now
                if record[self.FIRST_SEEN] is not None:
                    self._response_delays.append(now - record[self.FIRST_SEEN])
            record[self.LAST_HEALTH] = fire.health
            if watering[:, index].sum() > self.config.MAX_DRONES_ON_FIRE:
                self.overcrowding_seconds += self.dt

        self.fire_damage += sum(fire.health for fire in fires) * self.dt
        self.peak_active_fires = max(self.peak_active_fires, len(fires))
        return watering.any(axis=1)

    # --- Sicurezza ----------------------------------------------------------

    def _update_safety(self) -> None:
        distances = self.simulation.pair_distances     # già calcolate dalla simulazione
        if not distances.size:
            return
        closest = float(distances.min())
        if math.isfinite(closest):                     # infinito = nessuna coppia ancora in volo
            self.min_distance = min(self.min_distance, closest)
        # Un quasi-urto si conta una volta sola: quando la coppia SCENDE sotto la soglia.
        too_close_now = distances < self.config.EMERGENCY_AVOID_DISTANCE
        self.near_misses += int(np.sum(too_close_now & ~self._pairs_already_close))
        self._pairs_already_close = too_close_now

    # --- Uso del tempo ------------------------------------------------------

    def _update_activities(self, positions: np.ndarray, refilling: np.ndarray,
                           extinguishing: np.ndarray, flying: np.ndarray) -> None:
        """Classifica, per ogni drone, cosa stava facendo in questo passo."""
        for index, drone in enumerate(self.simulation.drones):
            if not flying[index]:
                activity = "out_of_service"
            elif refilling[index]:
                activity = "refill"
            elif drone.reloading:
                station = self.simulation.water_stations[drone.water_station_idx] if drone.water_station_idx is not None else None
                arrived = station is not None and np.linalg.norm(positions[index] - station) <= self.config.WATER_STATION_WAIT_RADIUS
                activity = "queue" if arrived else "to_station"
            elif extinguishing[index]:
                activity = "extinguishing"
            elif drone.fire_target is not None:
                activity = "to_fire"
            else:
                activity = "patrol"
            self.activity_seconds[activity] += self.dt

            # Tempo perso in coda: si accumula finché il drone non riesce a caricare acqua.
            if activity == "queue":
                self._queue_wait[drone.idx] = self._queue_wait.get(drone.idx, 0.0) + self.dt
            elif drone.idx in self._queue_wait and not drone.reloading:
                waited = self._queue_wait.pop(drone.idx)
                if flying[index]:
                    self.completed_queue_waits.append(waited)
                # Se invece è precipitato, l'attesa non è "finita": registrarla come conclusa
                # farebbe sembrare le code più corte proprio nelle varianti che perdono droni.

    # --- Perlustrazione -----------------------------------------------------

    def _update_coverage(self, flying_positions: np.ndarray) -> None:
        """Da quanto tempo ogni zona non viene guardata da nessuno.

        Le zone mai guardate valgono quanto è durata la missione fin qui, quindi una zona ignorata
        pesa sempre di più con il passare del tempo.
        """
        terrain = self.simulation.terrain
        step = self.simulation.step_count
        for position in flying_positions:
            self._ground_truth_coverage.mark_seen(
                terrain.cells_within(position, self.config.FIRE_DETECTION_RADIUS), step)

        if self.steps % self._network_every:
            return
        staleness = self._ground_truth_coverage.seconds_since_seen(step, self.dt,
                                                                   cap_seconds=self.simulation.sim_time)
        self._staleness_sum += float((staleness * self._importance).sum()) / max(self._importance_total, 1e-9)
        self._staleness_samples += 1
        if self._important_zones.any():
            self._staleness_hot_sum += float(staleness[self._important_zones].mean())
            self.worst_staleness = max(self.worst_staleness, float(staleness[self._important_zones].max()))
        if (~self._important_zones).any():
            self._staleness_cold_sum += float(staleness[~self._important_zones].mean())

    # --- Rete e conoscenza --------------------------------------------------

    def _sample_network(self) -> None:
        simulation = self.simulation
        # Un drone distrutto è fuori dal clock: la sua cassetta postale resta congelata all'ultimo
        # giro prima dell'urto. Contarlo falserebbe tutto ("vicini per drone" fatto di fantasmi).
        drones = [drone for drone in simulation.drones if drone.radio_works]
        if not drones:
            return
        self._network_samples += 1
        self._neighbors_sum += sum(len(drone.neighbors_heard) for drone in drones) / len(drones)
        self._connected_samples += int(_swarm_is_connected(simulation.neighbors_now))

        burning_now = {vec_to_tuple(fire.pos) for fire in simulation.world.fires}
        phantom = known_share = 0.0
        for drone in drones:
            phantom += sum(1 for fire_pos in drone.known_fires if fire_pos not in burning_now)
            if burning_now:
                known_share += sum(1 for fire_pos in burning_now if fire_pos in drone.known_fires) / len(burning_now)
                ages = [age for fire_pos, age in drone.known_fires.items() if fire_pos in burning_now]
                self._info_age_sum += sum(ages) * self.dt
                self._info_age_count += len(ages)
        self._phantom_sum += phantom / len(drones)
        if burning_now:
            self._awareness_sum += known_share / len(drones)
            self._awareness_samples += 1

    # --- Risultato finale ---------------------------------------------------

    def results(self, mission: MissionResult) -> Dict[str, Any]:
        """Tutte le misure, pronte per una riga di CSV."""
        simulation = self.simulation
        total_drone_seconds = len(simulation.drones) * simulation.sim_time
        channel = simulation.world.channel
        # L'equità si misura tra chi è ancora in volo: includere i caduti farebbe sembrare
        # "ingiusto" uno sciame che ha solo perso dei droni, cosa già contata da drones_lost.
        water_per_drone = [drone.water_delivered for drone in simulation.drones if drone.is_flying]
        all_queue_waits = self.completed_queue_waits + list(self._queue_wait.values())

        # Con le accensioni spontanee la missione non ha una fine: "riuscita" non vuol dire
        # niente, e riportare 0% sarebbe fuorviante quanto riportare 100%. Si dichiara indefinita.
        endless = self.config.IGNITION_RATE_PER_S > 0.0
        values = {
            "mission_complete": None if endless else mission.success,
            "fire_overrun": mission.outcome is MissionOutcome.OUT_OF_CONTROL,
            "extinction_time_s": self.extinction_time,
            "fire_damage": self.fire_damage,
            "burning_health_mean": _share(self.fire_damage, simulation.sim_time),
            "fires_spawned": simulation.world.spawned_count,
            "fires_ignited": simulation.world.ignited_count,
            "peak_active_fires": self.peak_active_fires,
            "detection_delay_s": _average(self._detection_delays),
            "fires_never_seen": sum(1 for record in self._fire_records.values()
                                    if record[self.FIRST_SEEN] is None),
            "response_delay_s": _average(self._response_delays),

            "collisions": simulation.collisions,
            "drones_lost": simulation.drones_lost,
            "near_misses": self.near_misses,
            "min_distance_m": self.min_distance if math.isfinite(self.min_distance) else None,
            "emergency_fraction": _share(self.emergency_seconds, self.flight_seconds),

            **{f"time_{activity}": _share(self.activity_seconds[activity], total_drone_seconds)
               for activity in ACTIVITIES},

            "water_fairness": _fairness(water_per_drone),
            "overcrowding_s": self.overcrowding_seconds,
            # Le attese ancora in corso a fine missione contano: scartarle nasconderebbe proprio
            # i casi peggiori, cioè i droni rimasti in coda fino all'ultimo.
            "queue_wait_mean_s": _average(all_queue_waits),
            "queue_wait_max_s": max(all_queue_waits) if all_queue_waits else None,
            "distance_per_drone_m": _share(self.distance_travelled * simulation.sim_time, self.flight_seconds),
            "control_effort_rate": _share(self.control_effort, self.flight_seconds),
            "saturation_bounces": sum(drone.bounce_count for drone in simulation.drones),

            "message_loss": _share(channel.messages_dropped, channel.messages_attempted),
            "neighbors": _share(self._neighbors_sum, self._network_samples),
            "connected_fraction": _share(self._connected_samples, self._network_samples),
            "fire_awareness": _share(self._awareness_sum, self._awareness_samples),
            "phantom_fires": _share(self._phantom_sum, self._network_samples),
            "info_age_s": _share(self._info_age_sum, self._info_age_count),

            "coverage_staleness_s": _share(self._staleness_sum, self._staleness_samples),
            "coverage_staleness_hot_s": _share(self._staleness_hot_sum, self._staleness_samples),
            "coverage_staleness_cold_s": _share(self._staleness_cold_sum, self._staleness_samples),
            "coverage_staleness_max_s": self.worst_staleness,
        }
        assert list(values) == [metric.key for metric in METRICS], \
            "METRICS e results() devono elencare le stesse misure, nello stesso ordine"
        return values


def _average(values: List[float]) -> Optional[float]:
    return float(np.mean(values)) if values else None


def _share(part: float, whole: float) -> Optional[float]:
    return float(part / whole) if whole else None


def _fairness(values: List[float]) -> Optional[float]:
    """Indice di Jain: 1 se tutti hanno lavorato uguale, verso 0 se pochi hanno fatto tutto."""
    total = sum(values)
    if not values or total <= 0:
        return None
    return float(total ** 2 / (len(values) * sum(value ** 2 for value in values)))


def _swarm_is_connected(neighbors: Dict[int, list]) -> bool:
    """Vero se, partendo da un drone qualsiasi, si raggiungono tutti gli altri passando di vicino in vicino."""
    if not neighbors:
        return False
    start = next(iter(neighbors))
    seen = {start}
    to_visit = [start]
    while to_visit:
        current = to_visit.pop()
        for neighbor in neighbors.get(current, []):
            if neighbor.idx not in seen:
                seen.add(neighbor.idx)
                to_visit.append(neighbor.idx)
    return len(seen) == len(neighbors)


# ============================================================
# 2. COSA SI CONFRONTA
# ============================================================

@dataclass(frozen=True)
class Scenario:
    """La situazione di partenza: quanti incendi, quanto grande l'area, quando fermarsi."""
    name: str
    description: str
    params: Dict[str, Any] = field(default_factory=dict)   # parametri diversi dai valori di default
    random_fires: bool = True
    random_stations: bool = True
    max_time_s: float = 300.0
    max_active_fires: int = 40        # oltre questa soglia la missione è considerata persa


@dataclass(frozen=True)
class Variant:
    """Una versione del sistema da mettere alla prova: un nome e i parametri che cambia."""
    name: str
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Experiment:
    """Una domanda, lo scenario in cui la si pone, e le varianti da confrontare.

    La prima variante è il RIFERIMENTO: tutte le altre vengono confrontate con quella.
    """
    question: str
    scenario: Scenario
    variants: List[Variant]


SCENARIOS = {
    "facile": Scenario(
        "facile",
        "Tre incendi e tre stazioni in posizioni casuali, tutto il resto ai valori di default. "
        "Lo sciame ce la fa quasi sempre: utile come controllo, poco per distinguere le varianti."),
    "critico": Scenario(
        "critico",
        "Cinque incendi in posizioni casuali che crescono più in fretta del normale (0.6 vita/s). "
        "Lo sciame riesce circa due volte su tre: è in questa zona di difficoltà che le differenze "
        "tra varianti si vedono.",
        params={"NUM_FIRES": 5, "FIRE_GROWTH_RATE": 0.6}, max_time_s=600.0),
    "ricerca": Scenario(
        "ricerca",
        "Area quattro volte più grande (40 × 24 m), un solo incendio iniziale e accensioni "
        "spontanee distribuite secondo l'importanza del terreno (una ogni ~33 s). Qui il problema "
        "non è l'acqua ma TROVARE gli incendi: i droni passano l'80% del tempo a perlustrare, "
        "contro il 6% dello scenario 'critico'.",
        params={"AREA_WIDTH": 40.0, "AREA_HEIGHT": 24.0, "NUM_FIRES": 1, "IGNITION_RATE_PER_S": 0.03},
        max_time_s=300.0),
}

EXPERIMENTS = {
    "ablation": Experiment(
        "Quanto serve davvero ciascun meccanismo dello sciame?",
        SCENARIOS["critico"],
        [Variant("sistema completo"),
         Variant("senza predizione degli urti", {"AVOIDANCE_MODE": "emergency-only"}),
         Variant("senza evitamento", {"AVOIDANCE_MODE": "none"}),
         Variant("radio con metà messaggi persi", {"PACKET_LOSS": 0.5}),
         Variant("senza allontanarsi dagli incendi affollati", {"SATURATION_BOUNCE": False})]),
    "urti": Experiment(
        "Quanto costa un urto allo sciame, e quanti droni può perdere prima di non farcela più?",
        SCENARIOS["critico"],
        [Variant("urti innocui", {"COLLISION_DAMAGE": False}),
         Variant("urti con danni"),
         Variant("urti con danni, senza evitamento", {"AVOIDANCE_MODE": "none"}),
         Variant("urti sempre fatali", {"COLLISION_TOTAL_LOSS_SPEED": 0.0}),
         # Che cosa cambia se un rottame resta un ostacolo invece di essere sorvolato.
         Variant("relitti ingombranti", {"WRECK_BLOCKS_FLIGHT": True})]),
    "guasti": Experiment(
        "Un guasto parziale fa più danni di uno totale?",
        SCENARIOS["critico"],
        # Lo sciame è sano (evitamento acceso, nessun urto), e al secondo 30 tre droni su dodici si
        # rompono di colpo. Cambia solo COME si rompono: il danno alla singola macchina è sempre lo
        # stesso, quello che cambia è cosa continuano a raccontare agli altri.
        [Variant("nessun guasto", {}),
         Variant("perdita totale (radio spenta)",
                 {"FAILURE_INJECTION_COUNT": 3, "FAILURE_INJECTION_RADIO_OFF": True}),
         Variant("avaria dichiarata", {"FAILURE_INJECTION_COUNT": 3}),
         Variant("guasto silenzioso",
                 {"FAILURE_INJECTION_COUNT": 3, "SILENT_FAILURE_PROBABILITY": 1.0})]),
    "radio": Experiment(
        "Quanto peggiora lo sciame quando la radio perde messaggi?",
        SCENARIOS["critico"],
        [Variant(f"{loss:.0%} di messaggi persi", {"PACKET_LOSS": loss})
         for loss in (0.0, 0.1, 0.2, 0.3, 0.5)]),
    "difficolta": Experiment(
        "Fino a che velocità di crescita degli incendi lo sciame riesce a contenerli?",
        SCENARIOS["critico"],
        [Variant(f"crescita {rate} vita/s", {"FIRE_GROWTH_RATE": rate}) for rate in (0.5, 0.6, 0.7, 0.8)]),
    "flotta": Experiment(
        "Quanti droni servono? Come cambiano risultati e sicurezza con la dimensione dello sciame?",
        SCENARIOS["critico"],
        [Variant(f"{count} droni", {"NUM_DRONES": count}) for count in (12, 8, 16, 20)]),
    "copertura": Experiment(
        "Perlustrare dove non si guarda da più tempo fa trovare prima gli incendi?",
        SCENARIOS["ricerca"],
        [Variant("ricerca casuale"),
         Variant("copertura persistente", {"EXPLORATION_MODE": "coverage"}),
         Variant("copertura uniforme", {"EXPLORATION_MODE": "coverage", "COVERAGE_IMPORTANCE_EXPONENT": 0.0})]),
    "importanza": Experiment(
        "Quanto conviene concentrare la perlustrazione dove gli incendi sono più probabili?",
        SCENARIOS["ricerca"],
        [Variant(f"concentrazione γ={gamma:g}",
                 {"EXPLORATION_MODE": "coverage", "COVERAGE_IMPORTANCE_EXPONENT": gamma})
         for gamma in (0.0, 0.5, 1.0, 2.0)]),
}


def config_for(experiment: Experiment, variant: Variant) -> SimConfig:
    """I parametri di una variante: default + quelli dello scenario + quelli della variante."""
    return DEFAULT_CONFIG.with_overrides(**{**experiment.scenario.params, **variant.params})


# ============================================================
# 3. COME SI ESEGUE
# ============================================================

def run_one_simulation(variant_name: str, config: SimConfig, scenario: Scenario, seed: int) -> Dict[str, Any]:
    """Una simulazione completa e misurata. Restituisce una riga di risultati.

    Il seed decide tutto ciò che è casuale, quindi la stessa coppia (scenario, seed) produce
    sempre la stessa situazione di partenza, qualunque sia la variante in prova.
    """
    simulation = Simulation(seed=seed, random_fires=scenario.random_fires,
                            random_stations=scenario.random_stations, config=config)
    measurements = Measurements(simulation)
    mission = simulation.run(max_time_s=scenario.max_time_s,
                             max_active_fires=scenario.max_active_fires,
                             watchers=[measurements])
    return {"variant": variant_name, "seed": seed, "duration_s": round(mission.elapsed_s, 2),
            "outcome": mission.outcome.value, **measurements.results(mission)}


def run_experiment(experiment: Experiment, runs: int, workers: Optional[int] = None) -> List[Dict[str, Any]]:
    """Esegue tutte le simulazioni dell'esperimento: ogni variante su ognuno degli stessi `runs` seed.

    Le simulazioni sono indipendenti, quindi girano in parallelo su tutti i core disponibili.
    """
    seeds = range(FIRST_SEED, FIRST_SEED + runs)
    for variant in experiment.variants:
        for problem in validate_config(config_for(experiment, variant)):
            print(f"  ATTENZIONE [{variant.name}]: {problem}")

    jobs = [(variant.name, config_for(experiment, variant), experiment.scenario, seed)
            for variant in experiment.variants for seed in seeds]
    rows: List[Dict[str, Any]] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        running = [pool.submit(run_one_simulation, *job) for job in jobs]
        for finished, future in enumerate(as_completed(running), start=1):
            row = future.result()
            rows.append(row)
            print(f"  [{finished}/{len(jobs)}] {row['variant']}, seed {row['seed']}: "
                  f"{row['outcome']} dopo {row['duration_s']:.0f} s", flush=True)

    order = {variant.name: position for position, variant in enumerate(experiment.variants)}
    rows.sort(key=lambda row: (order[row["variant"]], row["seed"]))
    return rows


# ============================================================
# 4. COME SI RIASSUME: media e intervallo di confidenza
# ============================================================

@dataclass
class Summary:
    """Il riassunto di una misura su più simulazioni."""
    average: Optional[float]
    low: Optional[float]      # estremi dell'intervallo di confidenza al 95%
    high: Optional[float]
    runs: int                 # simulazioni in cui la misura era definita


def summarize(values: List[Any], metric: Metric) -> Summary:
    """Media di una misura e intervallo in cui, con il 95% di confidenza, sta la media "vera".

    L'intervallo dice quanto ci si può fidare: se è largo, servono più simulazioni. Si restringe
    come la radice del numero di simulazioni, quindi per dimezzarlo ne servono quattro volte tante.

    Per le misure numeriche si usa l'intervallo classico basato sulla t di Student; per quelle
    sì/no (dove la media è una percentuale di successi) l'intervallo di Wilson, che resta sensato
    anche quando i successi sono quasi zero o quasi tutti.
    """
    values = [value for value in values if value is not None]
    runs = len(values)
    if runs == 0:
        return Summary(None, None, None, 0)

    if metric.unit == "sì/no":
        successes = sum(1 for value in values if value) / runs
        z = 1.959964
        center = (successes + z * z / (2 * runs)) / (1 + z * z / runs)
        half_width = z * math.sqrt(successes * (1 - successes) / runs + z * z / (4 * runs * runs)) / (1 + z * z / runs)
        return Summary(successes, max(0.0, center - half_width), min(1.0, center + half_width), runs)

    numbers = np.asarray(values, dtype=float)
    average = float(numbers.mean())
    if runs == 1:
        return Summary(average, None, None, 1)
    half_width = _t_value_95(runs - 1) * float(numbers.std(ddof=1)) / math.sqrt(runs)
    # Nessuna misura può essere negativa, e le percentuali non superano il 100%.
    highest = average + half_width if metric.unit != "%" else min(1.0, average + half_width)
    return Summary(average, max(0.0, average - half_width), highest, runs)


def _t_value_95(degrees_of_freedom: int) -> float:
    """Il moltiplicatore dell'intervallo al 95%: con poche simulazioni è più grande di 1.96."""
    table = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179,
             2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064,
             2.060, 2.056, 2.052, 2.048, 2.045, 2.042]
    if degrees_of_freedom <= len(table):
        return table[degrees_of_freedom - 1]
    z = 1.959964                      # con molte simulazioni tende al valore della normale
    return z + (z ** 3 + z) / (4 * degrees_of_freedom)


# ============================================================
# 5. COME SI CONFRONTA una variante con il riferimento
# ============================================================

@dataclass
class Difference:
    """Il confronto di una misura tra riferimento e variante."""
    reference_average: float
    variant_average: float
    probability_of_luck: float    # quanto sarebbe facile ottenere per caso una differenza così
    pairs: int = 0                # su quante coppie di simulazioni si basa il confronto
    # Probabilità corretta per il numero di confronti fatti nel report (vedi holm_correction).
    corrected_probability: Optional[float] = None

    @property
    def is_real(self) -> bool:
        probability = self.corrected_probability
        if probability is None:
            probability = self.probability_of_luck
        return probability < SIGNIFICANCE


def compare_to_reference(reference_runs: List[Dict[str, Any]], variant_runs: List[Dict[str, Any]],
                         metric: Metric) -> Optional[Difference]:
    """Confronta due varianti SIMULAZIONE PER SIMULAZIONE, accoppiandole per seed.

    Il seed 7 del riferimento e il seed 7 della variante partono dalla stessa identica situazione:
    stessi incendi, stesse stazioni, stesse posizioni iniziali dei droni. Confrontando coppie così
    si elimina la fortuna dello scenario, e restano solo gli effetti della modifica in esame.

    Da qui escono due numeri: di quanto cambia la media, e quanto è probabile che un cambiamento
    del genere sia semplicemente fortuna (vedi le due funzioni qui sotto).
    """
    reference_by_seed = {row["seed"]: row[metric.key] for row in reference_runs}
    pairs = [(reference_by_seed[row["seed"]], row[metric.key]) for row in variant_runs
             if row["seed"] in reference_by_seed
             and reference_by_seed[row["seed"]] is not None and row[metric.key] is not None]
    if not pairs:
        return None                       # la misura non è definita in nessuna coppia

    reference_values = np.array([float(reference) for reference, _ in pairs])
    variant_values = np.array([float(variant) for _, variant in pairs])

    if metric.unit == "sì/no":
        only_reference_succeeded = int(np.sum((reference_values == 1) & (variant_values == 0)))
        only_variant_succeeded = int(np.sum((reference_values == 0) & (variant_values == 1)))
        probability = _probability_of_luck_yes_no(only_reference_succeeded, only_variant_succeeded)
    else:
        probability = _probability_of_luck(variant_values - reference_values)

    return Difference(float(reference_values.mean()), float(variant_values.mean()), probability,
                      pairs=len(pairs))


def _probability_of_luck(differences: np.ndarray, attempts: int = 20000) -> float:
    """Quanto sarebbe facile ottenere per caso una differenza media grande come quella osservata.

    `differences` contiene, per ogni coppia di simulazioni con lo stesso seed, quanto ha fatto la
    variante meno quanto ha fatto il riferimento. Se la modifica non servisse a niente, il segno di
    ciascuna di queste differenze sarebbe deciso dal caso, come una moneta: a volte la variante
    farebbe un po' meglio, a volte un po' peggio, senza una direzione.

    Si simula proprio questo: si prendono le differenze osservate e si prova a CAMBIARNE I SEGNI in
    tutti i modi possibili (o in 20 000 modi estratti a caso, se le coppie sono troppe per provarli
    tutti: con 16 coppie le combinazioni sono già 65 536). Ogni combinazione di segni rappresenta
    un mondo in cui la modifica non conta nulla. La risposta è la quota di questi mondi in cui la
    differenza media risulta grande almeno quanto quella davvero osservata.

    Esempio: cinque coppie con differenze [-3, -2, -4, -1, -2]. La variante fa meglio in tutte e
    cinque. Cambiando i segni a caso capita di rado di ottenere una media altrettanto estrema:
    la probabilità esce bassa e la differenza viene giudicata reale.
    """
    if np.allclose(differences, 0.0):
        return 1.0                     # nessuna differenza: sicuramente non c'è niente da vedere

    pairs = len(differences)
    if pairs <= 16:
        sign_patterns = np.array(list(itertools.product((-1.0, 1.0), repeat=pairs)))
    else:
        # Seme fisso: lo stesso esperimento rifatto dà lo stesso p-value.
        sign_patterns = np.random.default_rng(0).choice((-1.0, 1.0), size=(attempts, pairs))

    averages_by_luck = np.abs((sign_patterns * differences).mean(axis=1))
    observed_average = abs(differences.mean())
    return float(np.mean(averages_by_luck >= observed_average - 1e-12))


def _probability_of_luck_yes_no(only_reference_succeeded: int, only_variant_succeeded: int) -> float:
    """La stessa domanda, per le misure sì/no come "missione riuscita" (test di McNemar).

    Le coppie in cui entrambe riescono, o entrambe falliscono, non dicono nulla su quale sia
    meglio: contano solo quelle DISCORDI. Se la modifica non contasse, ogni coppia discorde
    cadrebbe da una parte o dall'altra come il lancio di una moneta. Si calcola quindi quanto
    sarebbe improbabile uno sbilanciamento estremo come quello osservato.

    Esempio: su 6 coppie discordi la variante vince 6 a 0. È come fare sei teste di fila:
    succede in 2 casi su 64, cioè con probabilità 0.03, quindi la differenza è reale.
    """
    discordant_pairs = only_reference_succeeded + only_variant_succeeded
    if discordant_pairs == 0:
        return 1.0
    weaker_side = min(only_reference_succeeded, only_variant_succeeded)
    one_tail = sum(math.comb(discordant_pairs, count) for count in range(weaker_side + 1)) / 2 ** discordant_pairs
    return min(1.0, 2.0 * one_tail)      # due code: lo sbilanciamento può essere in entrambi i versi


def holm_correction(differences: List[Difference]) -> None:
    """Corregge le probabilità per il numero di confronti fatti, e le scrive nei Difference.

    Un report confronta decine di misure su più varianti: un centinaio di test insieme. Anche se
    nessuna variante cambiasse davvero qualcosa, con la soglia del 5% ci si aspetterebbero circa
    cinque "scoperte" per puro caso — e infatti, provando con dati puramente casuali, un report
    non corretto mostra in media una decina di marcatori ▲▼ inventati.

    Il metodo di Holm ordina le probabilità dalla più piccola alla più grande e moltiplica la
    k-esima per il numero di test che restano da fare (N-k+1), imponendo poi che la sequenza non
    decresca. È il correttivo standard più semplice che resta valido anche quando le misure sono
    correlate tra loro, come qui (chi spegne prima percorre meno strada, consuma meno, e così via).
    """
    ordered = sorted(differences, key=lambda difference: difference.probability_of_luck)
    total = len(ordered)
    running_max = 0.0
    for position, difference in enumerate(ordered):
        corrected = min(1.0, difference.probability_of_luck * (total - position))
        running_max = max(running_max, corrected)
        difference.corrected_probability = running_max


def verdict(metric: Metric, difference: Optional[Difference]) -> str:
    """'migliore', 'peggiore' o 'diverso' se la differenza è reale; stringa vuota altrimenti."""
    if difference is None or not difference.is_real or difference.variant_average == difference.reference_average:
        return ""
    if not metric.better:
        return "diverso"
    variant_is_higher = difference.variant_average > difference.reference_average
    return "migliore" if variant_is_higher == (metric.better == "alto") else "peggiore"


# ============================================================
# 6. IL REPORT
# ============================================================

MARKERS = {"migliore": " ▲", "peggiore": " ▼", "diverso": " ◆", "": ""}

FEW_RUNS = 20    # sotto questa soglia anche differenze grandi faticano a risultare significative

# Le misure citate nel riassunto iniziale; tutte le altre restano nelle tabelle.
HEADLINE_METRICS = ["mission_complete", "extinction_time_s", "burning_health_mean", "collisions",
                    "drones_lost", "fires_never_seen"]


def all_comparisons(experiment: Experiment, runs_by_variant: Dict[str, list]) -> Dict[tuple, Difference]:
    """Tutti i confronti del report, calcolati una volta sola e corretti per la molteplicità."""
    reference_name = experiment.variants[0].name
    comparisons: Dict[tuple, Difference] = {}
    for variant in experiment.variants[1:]:
        for metric in METRICS:
            difference = compare_to_reference(runs_by_variant[reference_name],
                                              runs_by_variant[variant.name], metric)
            if difference is not None:
                comparisons[(variant.name, metric.key)] = difference
    holm_correction(list(comparisons.values()))
    return comparisons


def write_report(name: str, experiment: Experiment, rows: List[Dict[str, Any]], runs: int,
                 elapsed_s: float, folder: str) -> str:
    """Scrive report.md: la lettura umana dell'esperimento."""
    runs_by_variant = {variant.name: [row for row in rows if row["variant"] == variant.name]
                       for variant in experiment.variants}
    reference_name = experiment.variants[0].name
    scenario = experiment.scenario
    comparisons = all_comparisons(experiment, runs_by_variant)

    lines = [
        f"# Esperimento «{name}»", "",
        f"**Domanda.** {experiment.question}", "",
        f"**Scenario «{scenario.name}».** {scenario.description}",
        f"Parametri diversi dal default: {_describe_params(scenario.params)}. Ogni simulazione dura "
        f"al massimo {scenario.max_time_s:.0f} s e viene fermata (contando come fallita) se gli "
        f"incendi accesi superano {scenario.max_active_fires}.", "",
        "**Varianti.** " + "; ".join(f"«{variant.name}» ({_describe_params(variant.params)})"
                                     for variant in experiment.variants)
        + f". Il riferimento è «{reference_name}».", "",
        f"**Metodo.** {runs} simulazioni per variante, con i seed da {FIRST_SEED} a "
        f"{FIRST_SEED + runs - 1}, gli stessi per tutte le varianti: ogni variante affronta quindi "
        f"esattamente le stesse situazioni di partenza. Eseguito il "
        f"{datetime.datetime.now():%d/%m/%Y alle %H:%M} in {elapsed_s / 60:.1f} minuti "
        f"(codice: commit {_git_commit()}).", "",
        "## In breve", "",
    ]
    if runs < FEW_RUNS:
        smallest_possible = 2 / 2 ** runs
        lines += [f"⚠ Poche simulazioni per variante ({runs}). Con {runs} coppie, anche una variante "
                  f"che vince su TUTTE le simulazioni ottiene al massimo una probabilità di "
                  f"{smallest_possible:.2g}; moltiplicata per il numero di confronti del report, può "
                  f"restare sopra la soglia. In pratica sotto le {FEW_RUNS} simulazioni può non "
                  f"comparire nessun marcatore ▲▼ anche davanti a differenze enormi: le differenze "
                  f"vanno lette dalle medie e dagli intervalli. Per conclusioni affidabili: "
                  f"--runs {FEW_RUNS} o più.", ""]
    lines += [_headline(experiment, variant, runs_by_variant, comparisons)
              for variant in experiment.variants[1:]]

    lines += ["", "## Risultati", "",
              "Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. "
              f"▲ / ▼ = significativamente migliore / peggiore di «{reference_name}»; "
              "◆ = significativamente diverso, per le misure senza un verso migliore."]

    for group in dict.fromkeys(metric.group for metric in METRICS):
        lines += ["", f"### {group}", "",
                  "| Misura | " + " | ".join(variant.name for variant in experiment.variants) + " |",
                  "|---|" + "---|" * len(experiment.variants)]
        for metric in (metric for metric in METRICS if metric.group == group):
            cells = []
            for variant in experiment.variants:
                summary = summarize([row[metric.key] for row in runs_by_variant[variant.name]], metric)
                cell = _format(metric, summary.average)
                if summary.low is not None:
                    cell += f" ({_format(metric, summary.low)}–{_format(metric, summary.high)})"
                if 0 < summary.runs < runs:  # la misura non era definita in tutte le simulazioni
                    cell += f" [su {summary.runs}]"
                if variant.name != reference_name:
                    cell += MARKERS[verdict(metric, comparisons.get((variant.name, metric.key)))]
                cells.append(cell)
            lines.append(f"| {metric.label}{_unit_suffix(metric)} | " + " | ".join(cells) + " |")

    lines += [
        "", "## Come leggere questi numeri", "",
        "- **Intervallo di confidenza al 95%**: ripetendo l'esperimento all'infinito, la media vera "
        "cadrebbe quasi sempre dentro quell'intervallo. Se è largo, servono più simulazioni.",
        f"- **▲ ▼ ◆**: la differenza rispetto al riferimento supera il test statistico "
        f"(probabilità che sia fortuna sotto {SIGNIFICANCE:.0%}). Le simulazioni sono confrontate a "
        "coppie con lo stesso seed, e le probabilità sono corrette con il metodo di Holm per il "
        "numero di confronti fatti in questo report: senza quella correzione, una decina di "
        "marcatori sarebbero falsi allarmi dovuti al solo numero di test. Attenzione: \"reale\" non "
        "vuol dire \"grande\" — guardare anche di quanto cambia la media.",
        "- **[su N]** accanto a un valore: quella misura non era definita in tutte le simulazioni "
        "(per esempio il tempo di spegnimento esiste solo per le missioni riuscite), quindi media e "
        "intervallo si basano su N simulazioni invece che su tutte.",
        "- **Tempo per spegnere tutto** è calcolato solo sulle missioni riuscite: va letto insieme a "
        "«Missione riuscita», altrimenti una variante che fallisce spesso sembra veloce.",
        "- Le simulazioni finiscono in momenti diversi (chi riesce prima si ferma prima), quindi le "
        "misure che si accumulano nel tempo — danno, distanza, sforzo — vanno confrontate con "
        "prudenza tra varianti con esiti molto diversi.",
        "", "## Che cosa significa ogni misura", "",
        *[f"- **{metric.label}** (`{metric.key}`): {metric.description}" for metric in METRICS],
        "", "I dati di ogni singola simulazione sono in `runs.csv`: una riga per simulazione, e come "
        "nomi di colonna quelli tra parentesi qui sopra.",
    ]

    path = os.path.join(folder, "report.md")
    with open(path, "w", encoding="utf-8") as report_file:
        report_file.write("\n".join(lines) + "\n")
    return path


def _headline(experiment: Experiment, variant: Variant, runs_by_variant: Dict[str, list],
              comparisons: Dict[tuple, Difference]) -> str:
    """Una riga di sintesi per una variante: cosa è cambiato davvero rispetto al riferimento."""
    reference_name = experiment.variants[0].name
    important_changes, other_changes = [], 0
    for metric in METRICS:
        difference = comparisons.get((variant.name, metric.key))
        judgement = verdict(metric, difference)
        if not judgement:
            continue
        if metric.key in HEADLINE_METRICS:
            on_subset = "" if difference.pairs == len(runs_by_variant[variant.name]) else \
                f", su {difference.pairs} simulazioni"
            important_changes.append(f"{metric.label.lower()} {_format(metric, difference.reference_average)} → "
                                     f"{_format(metric, difference.variant_average)} ({judgement}{on_subset})")
        else:
            other_changes += 1

    if not important_changes and not other_changes:
        return f"- **{variant.name}**: nessuna differenza significativa rispetto a «{reference_name}»."
    text = f"- **{variant.name}**: " + ("; ".join(important_changes) if important_changes
                                        else "nessun cambiamento tra le misure principali")
    if other_changes == 1:
        text += ". Cambia in modo significativo anche un'altra misura (vedi tabelle)"
    elif other_changes:
        text += f". Cambiano in modo significativo anche altre {other_changes} misure (vedi tabelle)"
    return text + "."


def write_runs_csv(rows: List[Dict[str, Any]], folder: str) -> str:
    """Scrive runs.csv: una riga per simulazione, per chi vuole rifare i grafici da sé."""
    path = os.path.join(folder, "runs.csv")
    with open(path, "w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows({key: ("" if value is None else value) for key, value in row.items()}
                         for row in rows)
    return path


def _format(metric: Metric, value: Optional[float]) -> str:
    if value is None:
        return "–"
    if metric.unit in ("%", "sì/no"):
        return f"{value * 100:.0f}%" if value >= 0.1 or value == 0 else f"{value * 100:.1f}%"
    if abs(value) >= 1000:
        return f"{value:,.0f}".replace(",", " ")     # spazio, non punto: sembrerebbe un decimale
    return f"{value:.3g}"


def _unit_suffix(metric: Metric) -> str:
    return "" if metric.unit in ("", "%", "sì/no") else f" [{metric.unit}]"


def _describe_params(params: Dict[str, Any]) -> str:
    return ", ".join(f"{name} = {value}" for name, value in params.items()) or "nessuno"


def _git_commit() -> str:
    """La versione del codice con cui è stato ottenuto il risultato: serve per poterlo rifare."""
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                                capture_output=True, text=True, timeout=5)
        uncommitted = subprocess.run(["git", "status", "--porcelain"],
                                     capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return "sconosciuto"
    if commit.returncode != 0:
        return "sconosciuto"
    return commit.stdout.strip() + (" con modifiche non salvate" if uncommitted.stdout.strip() else "")


# ============================================================
# AVVIO (chiamato da main.py)
# ============================================================

def list_experiments() -> None:
    print("Esperimenti disponibili (python main.py experiment NOME):\n")
    for name, experiment in EXPERIMENTS.items():
        print(f"  {name:<12} {experiment.question}")
        print(f"  {'':<12} varianti: {', '.join(variant.name for variant in experiment.variants)}\n")


def run_and_report(name: str, runs: int, workers: Optional[int] = None) -> str:
    """Esegue un esperimento, scrive report e dati, e restituisce la cartella dei risultati."""
    experiment = EXPERIMENTS[name]
    print(f"Esperimento «{name}»: {experiment.question}")
    print(f"{len(experiment.variants)} varianti × {runs} simulazioni, scenario «{experiment.scenario.name}»\n")

    started = time.perf_counter()
    rows = run_experiment(experiment, runs, workers)
    elapsed = time.perf_counter() - started

    folder = os.path.join(RESULTS_DIR, f"{name}_{datetime.datetime.now():%Y%m%d-%H%M}")
    os.makedirs(folder, exist_ok=True)
    report_path = write_report(name, experiment, rows, runs, elapsed, folder)
    data_path = write_runs_csv(rows, folder)

    print(f"\nFatto in {elapsed / 60:.1f} minuti.")
    print(f"Report:      {report_path}")
    print(f"Dati grezzi: {data_path}")
    return folder
