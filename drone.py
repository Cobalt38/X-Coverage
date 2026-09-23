"""
IL DRONE: che cosa sa, che cosa decide, come si muove.

Ogni drone è autonomo. Nessuno gli dice quale incendio spegnere o a quale stazione andare: decide
da solo con quello che vede (raggio FIRE_DETECTION_RADIUS), quello che ricorda e quello che gli
raccontano i vicini via radio. Nel file non esiste una riga che legga lo stato di un altro drone:
tutto passa dai messaggi (DroneMessage in world.py).

Un passo di simulazione, dall'inizio alla fine (metodo step() in fondo al file):

    1. leggo i messaggi arrivati       mailbox.begin_round()
    2. guardo e invecchio la memoria   sense_environment()
    3. unisco ciò che dicono i vicini  merge_neighbor_knowledge()
    4. scelgo cosa fare e mi muovo     decide_and_move()
    5. spruzzo acqua, se ho un fuoco a portata   try_extinguish()
    6. carico acqua, se sono alla stazione       try_reload()

E la decisione al punto 4 è una scala di priorità di tre gradini:

    acqua quasi finita?  -> vado alla stazione           (_move_toward_station)
    conosco un incendio non già affollato?  -> ci vado   (_engage_fire)
    nessuno dei due      -> perlustro                    (_exploration_target)

Il file è diviso in tre parti: il controllore PID (come si traduce "voglio andare lì" in
accelerazione), la memoria di copertura (dove si è già guardato) e la classe Drone.

Ordine di lettura del progetto: world.py, poi questo file, poi simulation.py.
"""

import math
import random
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

import numpy as np

from world import (GOLDEN_ANGLE, DroneMessage, DroneStatus, FirePos, ImportanceMap, Mailbox,
                   SimConfig, clamp_magnitude, normalize, unit_from_angle, vec_to_tuple)

if TYPE_CHECKING:
    from world import SimulationWorld


# ============================================================
# 1. CONTROLLORE PID
# ============================================================

class PIDController:
    """Trasforma "vorrei andare a questa velocità" in un'accelerazione realistica.

    Un drone vero non cambia velocità all'istante. Il PID guarda l'errore (velocità voluta meno
    velocità attuale) e produce un'accelerazione, con tre limiti fisici:
        - l'accelerazione massima (PID_MAX_OUTPUT_ACCEL);
        - quanto in fretta l'accelerazione può cambiare, cioè il jerk (MAX_JERK);
        - un tetto all'errore accumulato, per non "caricare" l'integrale durante una manovra lunga.

    Conseguenza pratica, importante per l'evitamento: da 1 m/s servono circa mezzo secondo e
    trenta centimetri per fermarsi. Le distanze di sicurezza sono dimensionate su questo.
    """

    def __init__(self, config: SimConfig):
        self.proportional_gain = config.PID_KP
        self.integral_gain = config.PID_KI
        self.derivative_gain = config.PID_KD
        self.integral_limit = config.PID_INTEGRAL_LIMIT
        self.max_acceleration = config.PID_MAX_OUTPUT_ACCEL
        self.max_jerk = config.MAX_JERK
        self.accumulated_error = np.zeros(2, dtype=float)
        self.previous_error = np.zeros(2, dtype=float)

    def reset(self) -> None:
        """Da chiamare quando il drone cambia compito: l'errore accumulato prima non c'entra più."""
        self.accumulated_error[:] = 0.0
        self.previous_error[:] = 0.0

    def acceleration_for(self, desired_velocity: np.ndarray, current_velocity: np.ndarray,
                         current_acceleration: np.ndarray, dt: float) -> np.ndarray:
        error = desired_velocity - current_velocity
        error_change = (error - self.previous_error) / max(dt, 1e-12)
        self.accumulated_error += error * dt
        self.accumulated_error = np.clip(self.accumulated_error, -self.integral_limit, self.integral_limit)

        wanted = (self.proportional_gain * error
                  + self.integral_gain * self.accumulated_error
                  + self.derivative_gain * error_change)
        wanted = clamp_magnitude(wanted, self.max_acceleration)

        # Limite di jerk: l'accelerazione può spostarsi solo di un tanto per passo.
        change = clamp_magnitude(wanted - current_acceleration, self.max_jerk * dt)
        new_acceleration = clamp_magnitude(current_acceleration + change, self.max_acceleration)

        self.previous_error = error.copy()
        return new_acceleration


# ============================================================
# 2. MEMORIA DI COPERTURA: dove si è già guardato
# ============================================================

class CoverageMemory:
    """Per ogni cella dell'area, il passo di simulazione in cui è stata vista l'ultima volta.

    Serve alla perlustrazione "coverage": invece di scegliere un punto a caso, il drone va dove
    non si guarda da più tempo. Due dettagli di progetto:

    - Non si memorizza un valore che decade, ma l'ISTANTE dell'ultima occhiata. La "freschezza"
      si calcola quando serve, quindi non c'è niente da aggiornare a ogni passo.
    - Unire due mappe è un massimo cella per cella. Questo rende la fusione indifferente
      all'ordine, alle ripetizioni e ai messaggi persi: esattamente come per le età delle notizie
      sugli incendi. Due droni che si incontrano dopo mezz'ora ottengono la stessa mappa
      qualunque sia stata la strada dell'informazione.
    """

    NEVER = -(2 ** 40)   # "mai vista": tanto vecchia da avere freschezza nulla

    def __init__(self, shape: Tuple[int, int]):
        self.last_seen = np.full(shape, self.NEVER, dtype=np.int64)

    def mark_seen(self, cells, step: int) -> None:
        """Registra che a questo passo le celle indicate sono state osservate."""
        if cells is None:
            return
        columns, rows, inside_the_circle = cells
        block = self.last_seen[columns, rows]     # è una vista: scrivere qui scrive nella mappa
        np.maximum(block, step, out=block, where=inside_the_circle)

    def merge(self, other_last_seen: np.ndarray) -> None:
        np.maximum(self.last_seen, other_last_seen, out=self.last_seen)

    def snapshot(self) -> np.ndarray:
        """Copia da allegare a un messaggio (il mittente continuerà a scrivere sulla propria)."""
        return self.last_seen.copy()

    def freshness(self, step: int, half_life_steps: float) -> np.ndarray:
        """1 = appena vista, 0 = vista tanto tempo fa. Si dimezza ogni `half_life_steps`."""
        age = (step - self.last_seen).astype(float)
        return np.exp(-math.log(2.0) * age / max(half_life_steps, 1e-9))

    def seconds_since_seen(self, step: int, dt: float, cap_seconds: float) -> np.ndarray:
        """Da quanti secondi ogni cella non viene guardata (le mai viste valgono `cap_seconds`)."""
        return np.minimum((step - self.last_seen).astype(float) * dt, cap_seconds)


def choose_patrol_point(terrain: ImportanceMap, coverage: CoverageMemory, position: np.ndarray,
                        neighbor_positions: Sequence[np.ndarray], step: int,
                        config: SimConfig) -> np.ndarray:
    """Dove conviene andare a dare un'occhiata, secondo questo drone.

    Due ingredienti:

    1. LA ZONA DI COMPETENZA. Il drone considera solo le celle che sono più vicine a lui che a
       qualunque vicino di cui abbia notizie: è la sua cella di Voronoi, calcolata però solo con
       i droni che sente in quel momento. Se è isolato, la sua zona è tutta la mappa. Questo evita
       che due droni vicini vadano a controllare lo stesso punto, senza bisogno di accordarsi.

    2. IL COSTO DI OGNI CELLA. Vale la pena andare dove non si guarda da tempo e dove il terreno
       conta, e non vale la pena fare troppa strada:

           costo(cella) = -importanza^γ · (1 - freschezza) + COVERAGE_W_DIST · distanza

       Si sceglie la cella di costo minimo. L'esponente γ decide quanto ci si concentra sulle
       zone di valore: con γ = 0 tutte le zone contano uguale.
    """
    distance_sq_from_me = (terrain.X - position[0]) ** 2 + (terrain.Y - position[1]) ** 2

    my_zone = np.ones(terrain.shape, dtype=bool)
    for neighbor in neighbor_positions:
        distance_sq_from_neighbor = (terrain.X - neighbor[0]) ** 2 + (terrain.Y - neighbor[1]) ** 2
        my_zone &= distance_sq_from_me <= distance_sq_from_neighbor

    freshness = coverage.freshness(step, config.COVERAGE_HALF_LIFE_STEPS)
    worth_a_look = terrain.weight(config.COVERAGE_IMPORTANCE_EXPONENT) * (1.0 - freshness)
    cost = -worth_a_look + config.COVERAGE_W_DIST * np.sqrt(distance_sq_from_me)
    cost = np.where(my_zone, cost, np.inf)

    column, row = np.unravel_index(int(np.argmin(cost)), cost.shape)
    return np.array([terrain.X[column, row], terrain.Y[column, row]], dtype=float)


# ============================================================
# 3. IL DRONE
# ============================================================

class Drone:
    """Un agente dello sciame: memoria, decisioni, volo.

    Stato principale:
        position, velocity, acceleration    dove si trova e come si muove
        target                              dove vuole andare adesso
        anchor_target                       il punto "base" attorno a cui gravita in perlustrazione
        water                               quanta acqua ha nel serbatoio
        known_fires / extinguished_fires    cosa crede degli incendi, con l'età di ogni notizia
        status                              in volo, a terra o distrutto (vedi break_down())
    """

    def __init__(self, idx: int, rng: random.Random, world: 'SimulationWorld', seed: int):
        self.idx = idx                     # identificatore: serve anche a rompere i pareggi
        self.world = world
        self.config: SimConfig = world.config
        # Generatore casuale personale: così le scelte del drone non consumano quello degli
        # incendi, e cambiare il comportamento dei droni non cambia dove scoppiano i fuochi.
        self.rng = random.Random(f"drone-{seed}-{idx}")

        self.position = np.array([rng.uniform(0.0, world.area_width),
                                  rng.uniform(0.0, world.area_height)], dtype=float)
        self.velocity = np.zeros(2, dtype=float)
        self.acceleration = np.zeros(2, dtype=float)
        self.desired_velocity = np.zeros(2, dtype=float)
        self.status = DroneStatus.FLYING
        self.burning_steps = 0             # da quanti passi il relitto sta dentro un incendio

        # L'obiettivo non è un punto fisso: si muove in un campo di forze (vedi sezione 6).
        self.original_target = np.array([rng.uniform(0.0, world.area_width),
                                         rng.uniform(0.0, world.area_height)], dtype=float)
        self.anchor_target = self.original_target.copy()
        self.target = self.original_target.copy()
        self.target_velocity = np.zeros(2, dtype=float)
        self.idle_steps = 0

        # Memoria: posizione dell'incendio -> età della notizia, in passi di simulazione.
        self.known_fires: Dict[FirePos, int] = {}
        self.extinguished_fires: Dict[FirePos, int] = {}
        self.saturated_fires: Dict[FirePos, int] = {}   # incendi già affollati -> per quanto ignorarli
        self.fire_target: Optional[FirePos] = None
        # Compito lasciato a metà per andare a fare acqua. È un'intenzione PRIVATA: non viene
        # comunicata e non invecchia, perché non è una notizia sul mondo ma un promemoria.
        self.interrupted_fire: Optional[FirePos] = None
        self.interrupted_target: Optional[np.ndarray] = None

        self.water = self.config.DRONE_WATER_CAPACITY
        self.reloading = False
        self.water_station_idx: Optional[int] = None
        self.refuel_claim_age = 0          # da quanti passi ha chiesto di rifornirsi
        self.station_slot: Optional[int] = None

        self.mailbox = Mailbox(self)
        self.pid = PIDController(self.config)
        self.coverage: Optional[CoverageMemory] = (
            CoverageMemory(world.terrain.shape) if self.config.EXPLORATION_MODE == "coverage" else None)
        self._last_patrol_plan_step = -10 ** 9

        # Numeri raccolti solo per le statistiche: la logica di volo non li guarda mai.
        self.in_emergency = False
        self.water_delivered = 0.0
        self.bounce_count = 0
        self.last_applied_force = np.zeros(2, dtype=float)
        self.last_avoidance_force = np.zeros(2, dtype=float)

    # --- Stato della macchina ----------------------------------------------

    @property
    def is_flying(self) -> bool:
        return self.status is DroneStatus.FLYING

    @property
    def radio_works(self) -> bool:
        return self.status is not DroneStatus.DESTROYED

    @property
    def neighbors_heard(self) -> Dict[int, DroneMessage]:
        """I messaggi ricevuti in questo giro: tutto ciò che il drone sa degli altri."""
        return self.mailbox.neighbors

    def break_down(self, radio_destroyed: bool) -> None:
        """Il drone è stato coinvolto in un urto e non vola più. Non si ripara.

        Due gravità diverse, decise dalla velocità dell'impatto (vedi simulation.py):
            - motori rotti: precipita dov'è, ma la radio continua a funzionare. Resta un ripetitore
              fermo che invecchia e ritrasmette ciò che sapeva: l'informazione non si perde.
            - perdita totale: anche la radio è spenta. Per lo sciame è come se non fosse mai esistito.
        """
        self.status = DroneStatus.DESTROYED if radio_destroyed else DroneStatus.GROUNDED
        self.velocity[:] = 0.0
        self.acceleration[:] = 0.0
        self.desired_velocity[:] = 0.0
        self.target = self.position.copy()
        self.anchor_target = self.position.copy()
        # Lascia liberi i compiti che aveva preso, altrimenti gli altri lo conterebbero ancora.
        self.fire_target = None
        self.reloading = False
        self.water_station_idx = None
        self.station_slot = None

    # --- 4. COMUNICAZIONE --------------------------------------------------

    def _build_message(self) -> DroneMessage:
        """Il messaggio che il drone trasmette ai vicini all'inizio di ogni passo."""
        return DroneMessage(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            target=self.target.copy(),
            reloading=self.reloading,
            water_station_idx=self.water_station_idx,
            refuel_claim_age=self.refuel_claim_age,
            station_slot=self.station_slot,
            # Un drone a terra non sta spegnendo niente, anche se è caduto vicino a un fuoco.
            extinguishing=self.is_flying and self.is_extinguishing_fire(),
            fire_target=self.fire_target,
            known_fires=dict(self.known_fires),
            extinguished_fires=dict(self.extinguished_fires),
            coverage=self._coverage_to_share(),
        )

    def _coverage_to_share(self) -> Optional[np.ndarray]:
        """La mappa di copertura si trasmette ogni COVERAGE_SYNC_PERIOD_S, non a ogni passo.

        È di gran lunga il dato più pesante del messaggio (una griglia intera), quindi mandarla
        continuamente sarebbe poco realistico. I droni trasmettono a turni sfalsati, così non
        parlano tutti nello stesso passo.
        """
        if self.coverage is None:
            return None
        sync_steps = self.config.COVERAGE_SYNC_STEPS
        if self.world.step_counter % sync_steps != self.idx % sync_steps:
            return None
        return self.coverage.snapshot()

    def broadcast(self) -> None:
        """Prima fase di ogni passo: tutti trasmettono, poi tutti ragionano."""
        if self.radio_works:
            self.world.broadcast(self.idx, self._build_message())

    def merge_neighbor_knowledge(self) -> None:
        self.mailbox.merge_fire_knowledge()
        if self.coverage is not None:
            for message in self.neighbors_heard.values():
                if message.coverage is not None:
                    self.coverage.merge(message.coverage)

    # --- 5. PERCEZIONE E MEMORIA -------------------------------------------

    def _age_memory(self) -> None:
        """Invecchia di un passo tutte le notizie e dimentica quelle scadute.

        L'età serve al gossip: quando due droni si scambiano notizie sullo stesso incendio, vince
        la più fresca. Una notizia che nessuno riconferma finisce per scadere da sola.
        """
        for fire_pos in list(self.known_fires):
            self.known_fires[fire_pos] += 1
            if self.known_fires[fire_pos] > self.config.FIRE_MEMORY_TTL_STEPS:
                del self.known_fires[fire_pos]

        for fire_pos in list(self.extinguished_fires):
            self.extinguished_fires[fire_pos] += 1
            if self.extinguished_fires[fire_pos] > self.config.EXTINGUISHED_FIRE_MEMORY_TTL_STEPS:
                del self.extinguished_fires[fire_pos]

        for fire_pos in list(self.saturated_fires):
            self.saturated_fires[fire_pos] -= 1
            if self.saturated_fires[fire_pos] <= 0:
                del self.saturated_fires[fire_pos]

    def sense_environment(self) -> None:
        """Guarda intorno a sé e aggiorna la memoria di conseguenza."""
        self._age_memory()

        visible_now = {vec_to_tuple(fire.pos) for fire in self.world.sense_fires(self.position)}

        # Segna come "guardate" tutte le celle entro il raggio del sensore.
        if self.coverage is not None:
            self.coverage.mark_seen(
                self.world.terrain.cells_within(self.position, self.config.FIRE_DETECTION_RADIUS),
                self.world.step_counter)

        # Smentita sul posto: se sono abbastanza vicino da vedere un incendio che ricordo e non lo
        # vedo, allora è spento. Da qui la notizia parte e si propaga agli altri via radio.
        for fire_pos in list(self.known_fires):
            if fire_pos in visible_now:
                continue
            if np.linalg.norm(self.position - np.array(fire_pos)) <= self.config.FIRE_DETECTION_RADIUS:
                del self.known_fires[fire_pos]
                self.extinguished_fires[fire_pos] = 0
                if self.fire_target == fire_pos:
                    self.fire_target = None

        # Quello che vedo con i miei occhi batte qualunque cosa mi abbiano raccontato.
        for fire_pos in visible_now:
            self.known_fires[fire_pos] = 0
            self.extinguished_fires.pop(fire_pos, None)

        if self.fire_target is not None and self.fire_target not in self.known_fires:
            self.fire_target = None

    # --- 6. SCELTA DELL'INCENDIO -------------------------------------------

    def _drones_with_priority_on(self, fire_pos: FirePos) -> int:
        """Quanti droni, tra quelli che sento, hanno più titolo di me a occuparsi di questo incendio.

        MAX_DRONES_ON_FIRE non può essere un contatore globale: nessuno lo potrebbe leggere. Ogni
        drone lo confronta con una stima locale, ordinando i pretendenti con la stessa regola:

            1. chi sta già spruzzando su quel fuoco (non si scalza chi sta lavorando);
            2. chi è più vicino;
            3. a parità, chi ha l'indice più basso.

        Applicando tutti la stessa regola agli stessi dati si arriva alla stessa conclusione senza
        doversi mettere d'accordo. E funziona perché la geometria lo garantisce: due droni sullo
        stesso incendio distano al massimo 2 × FIRE_EXTINGUISH_RADIUS, quindi si sentono per forza.
        """
        fire = np.array(fire_pos)
        my_distance = float(np.linalg.norm(self.position - fire))
        i_am_working_here = self.water > 0.0 and my_distance <= self.config.FIRE_EXTINGUISH_RADIUS
        my_claim = (0 if i_am_working_here else 1, my_distance, self.idx)

        better_claims = 0
        for other_idx, message in self.neighbors_heard.items():
            distance = float(np.linalg.norm(message.position - fire))
            working_here = message.extinguishing and distance <= self.config.FIRE_EXTINGUISH_RADIUS
            interested = working_here or message.fire_target == fire_pos
            if interested and (0 if working_here else 1, distance, other_idx) < my_claim:
                better_claims += 1
        return better_claims

    def _choose_fire(self) -> Tuple[Optional[FirePos], Optional[FirePos]]:
        """Quale incendio andare a spegnere, tra quelli che conosco.

        Restituisce due cose: l'incendio scelto (o None) e l'eventuale incendio già affollato da cui
        conviene allontanarsi subito, perché ci si è finiti dentro.

        Tre regole, in ordine:
            - si riparte dall'incendio di cui ci si stava già occupando, per non cambiare idea di
              continuo tra due fuochi quasi equidistanti;
            - gli incendi dove si è di troppo vengono segnati come "affollati" e ignorati per
              qualche secondo, altrimenti il drone tornerebbe subito a ronzarci attorno;
            - se il più vicino è affollato si passa al successivo, non si rinuncia.
        """
        candidates = sorted((fire_pos for fire_pos in self.known_fires if fire_pos not in self.saturated_fires),
                            key=lambda fire_pos: np.linalg.norm(np.array(fire_pos) - self.position))
        if self.fire_target in candidates:
            candidates.remove(self.fire_target)
            candidates.insert(0, self.fire_target)

        crowded_and_close: Optional[FirePos] = None
        for fire_pos in candidates:
            if self._drones_with_priority_on(fire_pos) < self.config.MAX_DRONES_ON_FIRE:
                return fire_pos, None
            self.saturated_fires[fire_pos] = self.config.FIRE_SATURATION_MEMORY_STEPS
            already_there = np.linalg.norm(self.position - np.array(fire_pos)) <= self.config.FIRE_DETECTION_RADIUS
            if crowded_and_close is None and already_there:
                crowded_and_close = fire_pos
        return None, crowded_and_close

    def _engage_fire(self, fire_pos: FirePos) -> None:
        """Punta all'incendio, fermandosi sull'anello di lavoro dal lato da cui si arriva.

        L'obiettivo fissa solo la DISTANZA dal fuoco (FIRE_WORK_RADIUS); dove disporsi lungo
        l'anello lo decide da sé l'evitamento collisioni, che spinge i droni di lato finché non
        sono distanziati. Così tre droni si distribuiscono attorno al fuoco senza coordinarsi,
        e non serve indebolire l'evitamento per farli lavorare insieme.
        """
        self.fire_target = fire_pos
        fire = np.array(fire_pos, dtype=float)
        direction_of_arrival = normalize(self.position - fire)
        if np.linalg.norm(direction_of_arrival) < 1e-6:        # sono esattamente sopra il fuoco
            direction_of_arrival = unit_from_angle(self.idx * GOLDEN_ANGLE)

        self.target = self._inside_area(fire + direction_of_arrival * self.config.FIRE_WORK_RADIUS)
        self.original_target = fire.copy()
        self.anchor_target = self.target.copy()   # quando il fuoco sarà spento, riparte da qui
        self.target_velocity[:] = 0.0

    def _back_away_from(self, fire_pos: FirePos) -> None:
        """Si allontana da un incendio dove è di troppo, per lasciare spazio a chi ci lavora."""
        self.bounce_count += 1
        fire = np.array(fire_pos, dtype=float)
        away = normalize(self.position - fire)
        if np.linalg.norm(away) < 1e-6:
            away = normalize(self.velocity) if np.linalg.norm(self.velocity) > 1e-6 else unit_from_angle(self.idx * GOLDEN_ANGLE)

        self.target = self._inside_area(self.position + away * self.config.FIRE_SATURATION_BOUNCE_DISTANCE)
        self.original_target = self.target.copy()
        self.anchor_target = self.target.copy()
        self.target_velocity[:] = 0.0

    def _inside_area(self, point: np.ndarray) -> np.ndarray:
        point = point.copy()
        point[0] = np.clip(point[0], 0.0, self.world.area_width)
        point[1] = np.clip(point[1], 0.0, self.world.area_height)
        return point

    # --- 7. PERLUSTRAZIONE -------------------------------------------------

    def _patrol_point(self) -> np.ndarray:
        """Il prossimo punto da andare a controllare quando non c'è niente da spegnere.

        Con EXPLORATION_MODE = "random" è un punto a caso nell'area: semplice, e sorprendentemente
        difficile da battere quando i droni sono tanti e l'area è piccola.
        Con "coverage" è la cella che più chiede una visita (vedi choose_patrol_point).
        """
        if self.coverage is None:
            return np.array([self.rng.uniform(0.0, self.world.area_width),
                             self.rng.uniform(0.0, self.world.area_height)], dtype=float)
        self._last_patrol_plan_step = self.world.step_counter
        neighbor_positions = [message.position for message in self.neighbors_heard.values()]
        return choose_patrol_point(self.world.terrain, self.coverage, self.position,
                                   neighbor_positions, self.world.step_counter, self.config)

    def _head_to(self, point: np.ndarray) -> None:
        self.original_target = point
        self.anchor_target = point.copy()
        self.target = point.copy()
        self.target_velocity[:] = 0.0
        self.idle_steps = 0

    def _maybe_pick_new_patrol_point(self) -> None:
        """Quando serve una nuova meta di perlustrazione.

        Appena arrivato (e fermo lì per MAX_IDLE_STEPS) se ne sceglie un'altra. In modalità
        "coverage" si ricalcola anche durante il volo, ogni COVERAGE_REPLAN_S: nel frattempo la
        mappa cambia, sia per le proprie occhiate sia per quelle raccontate dai vicini.
        """
        patrolling = not self.reloading and self.fire_target is None
        if not patrolling:
            self.idle_steps = 0
            return

        if self.coverage is not None and \
                self.world.step_counter - self._last_patrol_plan_step >= self.config.COVERAGE_REPLAN_STEPS:
            self._head_to(self._patrol_point())
            return

        if self.has_reached_target():
            self.idle_steps += 1
            if self.idle_steps > self.config.MAX_IDLE_STEPS:
                self._head_to(self._patrol_point())
        else:
            self.idle_steps = 0

    # --- 8. L'OBIETTIVO MOBILE ---------------------------------------------
    # In perlustrazione il drone non insegue un punto fermo: insegue un obiettivo che a sua volta
    # si muove, spinto da tre forze. Serve a far sì che gli obiettivi di droni vicini si respingano,
    # coprendo più area, senza che nessuno li assegni dall'alto.

    def repulsion_between_targets(self) -> np.ndarray:
        """Gli obiettivi di due droni vicini si respingono, così non vanno a controllare lo stesso punto."""
        force = np.zeros(2, dtype=float)
        separation = self.config.TARGET_SEPARATION
        for message in self.neighbors_heard.values():
            offset = self.target - message.target
            distance = np.linalg.norm(offset)
            if distance >= separation:
                continue
            if distance < 1e-6:                      # obiettivi coincidenti: si usa la posizione
                between_drones = self.position - message.position
                direction = (normalize(between_drones) if np.linalg.norm(between_drones) > 1e-6
                             else unit_from_angle(self.idx * GOLDEN_ANGLE))
                force += self.config.K_REPULSION_BETWEEN_TARGETS * separation * direction
            else:
                force += self.config.K_REPULSION_BETWEEN_TARGETS * (separation - distance) * offset / distance
        return clamp_magnitude(force, self.config.MAX_FORCE_ON_TARGET)

    def boundary_force(self) -> np.ndarray:
        """Respinge l'obiettivo verso l'interno quando si avvicina troppo a un bordo."""
        force = np.zeros(2, dtype=float)
        margin = self.config.MARGIN_REPULSION_BOUNDARY
        gain = self.config.K_BOUNDARY_REPULSION
        if self.target[0] < margin:
            force[0] += gain * (margin - self.target[0]) / max(margin, 1e-6)
        elif self.target[0] > self.world.area_width - margin:
            force[0] -= gain * (self.target[0] - (self.world.area_width - margin)) / max(margin, 1e-6)
        if self.target[1] < margin:
            force[1] += gain * (margin - self.target[1]) / max(margin, 1e-6)
        elif self.target[1] > self.world.area_height - margin:
            force[1] -= gain * (self.target[1] - (self.world.area_height - margin)) / max(margin, 1e-6)
        return force

    def anchor_force(self) -> np.ndarray:
        """Richiama l'obiettivo verso l'ancora, cioè il punto base di questa perlustrazione."""
        return self.config.K_ANCHOR_DRAGGING * (self.anchor_target - self.target)

    def total_force_on_target(self) -> np.ndarray:
        total = self.repulsion_between_targets() + self.anchor_force() + self.boundary_force()
        return clamp_magnitude(total, self.config.MAX_FORCE_ON_TARGET)

    def has_reached_target(self) -> bool:
        return bool(np.linalg.norm(self.position - self.target) < self.config.TARGET_REACHED_DISTANCE)

    def _move_target(self) -> None:
        """Fa avanzare l'obiettivo di un passo sotto l'effetto delle forze (solo in perlustrazione)."""
        config = self.config
        force = self.total_force_on_target()
        self.last_applied_force = force.copy()

        slow_down = math.exp(-config.TARGET_VEL_DAMPING_RATE * config.SIM_TIME_STEP)
        self.target_velocity = slow_down * self.target_velocity + force * config.SIM_TIME_STEP
        self.target_velocity = clamp_magnitude(self.target_velocity, config.MAX_TARGET_SPEED)
        self.target = self._inside_area(self.target + self.target_velocity * config.SIM_TIME_STEP)

        # L'ancora insegue l'obiettivo molto lentamente: è quasi un punto fisso, che viene
        # spostato di colpo quando il drone cambia compito.
        offset = self.target - self.anchor_target
        if np.linalg.norm(offset) > 1e-6:
            self.anchor_target += offset * (1.0 - math.exp(-config.ANCHOR_TO_TARGET_INTENSITY * config.SIM_TIME_STEP))

    # --- 9. EVITAMENTO DELLE COLLISIONI ------------------------------------

    def _avoidance_correction(self) -> Tuple[np.ndarray, bool, np.ndarray]:
        """Di quanto correggere la velocità per non finire addosso a nessuno.

        LIVELLO 1, predittivo. Per ogni vicino si calcola il punto di massimo avvicinamento: dove e
        quando i due si troveranno più vicini, se entrambi proseguono dritti. In pratica, guardando
        la posizione relativa `r` e la velocità relativa `v`, il momento critico è

            t = -(r · v) / |v|²          limitato all'intervallo [0, AVOID_LOOKAHEAD]

        e lo scarto con cui si mancheranno è `r + v·t`. Se quello scarto è minore del margine di
        sicurezza, si spinge NELLA SUA DIREZIONE: questo allarga il passaggio, cioè fa scansare di
        lato invece di frenare frontalmente. Guardare il momento critico e non solo "dove saremo
        tra 1.5 s" è essenziale: due droni che si incrociano possono sfiorarsi a metà strada ed
        essere di nuovo lontani alla fine dell'intervallo.

        LIVELLO 2, emergenza. Se la distanza ATTUALE è già sotto EMERGENCY_AVOID_DISTANCE, ci si
        scansa e basta: del compito resta solo EMERGENCY_TARGET_WEIGHT. Questo livello non viene
        mai disattivato, nemmeno tra due droni che stanno spegnendo lo stesso incendio.

        Il livello 3 è l'urto vero e proprio, ed è gestito da simulation.py.

        Restituisce (correzione predittiva, emergenza sì/no, vettore di emergenza).
        """
        config = self.config
        predictive = np.zeros(2, dtype=float)
        emergency_push = np.zeros(2, dtype=float)
        in_emergency = False
        # Più si va veloci, più distanza serve: il margine cresce con la propria velocità.
        safety_margin = config.AVOID_MIN_DISTANCE + config.SAFE_DISTANCE_K_VEL * np.linalg.norm(self.velocity)

        for other_idx, message in self.neighbors_heard.items():
            offset = self.position - message.position
            distance = np.linalg.norm(offset)
            relative_velocity = self.velocity - message.velocity

            if distance > 1e-9:
                direction_now = offset / distance
            else:
                # Sovrapposti esattamente: i due scelgono versi opposti in base all'indice.
                direction_now = np.array([1.0, 0.0]) if self.idx < other_idx else np.array([-1.0, 0.0])

            closing_speed_sq = float(np.dot(relative_velocity, relative_velocity))
            if closing_speed_sq > 1e-12:
                time_to_closest = float(np.clip(-np.dot(offset, relative_velocity) / closing_speed_sq,
                                                0.0, config.AVOID_LOOKAHEAD))
            else:
                time_to_closest = 0.0                  # stessa velocità: la distanza non cambia

            miss_vector = offset + relative_velocity * time_to_closest
            miss_distance = np.linalg.norm(miss_vector)

            if miss_distance > 1e-6:
                push_direction = miss_vector / miss_distance
            elif closing_speed_sq > 1e-12:
                # Frontale perfetto: si scarta di lato. L'altro drone vede la velocità relativa
                # opposta, quindi sceglie il lato opposto e i due si separano.
                heading = relative_velocity / math.sqrt(closing_speed_sq)
                push_direction = np.array([-heading[1], heading[0]])
            else:
                push_direction = direction_now

            approach_speed = max(0.0, -float(np.dot(relative_velocity, direction_now)))

            if miss_distance < safety_margin:
                how_much_too_close = (safety_margin - miss_distance) / safety_margin
                predictive += push_direction * (config.K_AVOID_REPULSION * how_much_too_close
                                                + config.K_AVOID_DAMPING * approach_speed)

            if distance < config.EMERGENCY_AVOID_DISTANCE:
                in_emergency = True
                emergency_push += (direction_now * (config.EMERGENCY_AVOID_DISTANCE - distance) * config.EMERGENCY_AVOID_GAIN
                                   - relative_velocity * config.EMERGENCY_DAMPING)

        return clamp_magnitude(predictive, config.AVOID_MAX_CORRECTION), in_emergency, emergency_push

    def _desired_velocity(self) -> np.ndarray:
        """Velocità che il drone vorrebbe avere: andare verso l'obiettivo, più scansare chi incontra.

        Attenzione all'ordine: la parte "vai verso l'obiettivo" viene limitata alla velocità massima
        PRIMA di sommare l'evitamento. Altrimenti, con un obiettivo lontano, quella parte varrebbe
        dieci volte tanto e il limite finale schiaccerebbe la correzione fino a renderla inutile.
        """
        config = self.config
        toward_target = clamp_magnitude(config.K_DESIRED_VEL_TO_TARGET * (self.target - self.position),
                                        config.MAX_DRONE_SPEED)

        if config.AVOIDANCE_MODE == "none":              # variante di confronto: nessun evitamento
            self.in_emergency = False
            self.last_avoidance_force = np.zeros(2, dtype=float)
            return toward_target

        predictive, in_emergency, emergency_push = self._avoidance_correction()
        if config.AVOIDANCE_MODE == "emergency-only":    # variante: solo la reazione ravvicinata
            predictive = np.zeros(2, dtype=float)

        self.in_emergency = in_emergency
        if in_emergency:
            self.last_avoidance_force = emergency_push
            desired = emergency_push + config.EMERGENCY_TARGET_WEIGHT * toward_target
        else:
            self.last_avoidance_force = predictive
            desired = toward_target + predictive
        return clamp_magnitude(desired, config.MAX_DRONE_SPEED)

    def _fly(self) -> None:
        """Un passo di volo: velocità voluta -> PID -> accelerazione -> velocità -> posizione."""
        config = self.config
        self.desired_velocity = clamp_magnitude(self._desired_velocity(), config.MAX_DRONE_SPEED)
        self.acceleration = self.pid.acceleration_for(self.desired_velocity, self.velocity,
                                                      self.acceleration, config.SIM_TIME_STEP)
        self.velocity = clamp_magnitude(self.velocity + self.acceleration * config.SIM_TIME_STEP,
                                        config.MAX_DRONE_SPEED)
        self.position += self.velocity * config.SIM_TIME_STEP
        self.position[0] = np.clip(self.position[0], 0.0, self.world.area_width)
        self.position[1] = np.clip(self.position[1], 0.0, self.world.area_height)

    # --- 10. RIFORNIMENTO ---------------------------------------------------
    # Le stazioni hanno pochi posti, quindi serve una coda. Non c'è nessuno che la gestisce: ogni
    # drone la ricostruisce da sé dai messaggi, e tutti arrivano allo stesso ordine perché usano
    # la stessa regola di precedenza.

    def _is_in_service_area(self, position: np.ndarray, station_idx: int) -> bool:
        distance = np.linalg.norm(position - self.world.water_stations[station_idx])
        return bool(distance <= self.config.WATER_STATION_SERVICE_RADIUS)

    def _queue_priority(self, station_slot: Optional[int], position: np.ndarray,
                        waiting_since: int, idx: int) -> Tuple[int, int, int]:
        """Precedenza alla stazione: più piccola la tupla, prima si passa.

            1. chi sta già caricando acqua non viene mai scalzato;
            2. poi chi aspetta da più tempo (la classica coda: primo arrivato, primo servito);
            3. a parità, l'indice più basso.
        """
        already_being_served = (station_slot is not None
                                and self._is_in_service_area(position, self.water_station_idx))
        return (0 if already_being_served else 1, -waiting_since, idx)

    def _queue_at_my_station(self) -> List[Tuple[Tuple[int, int, int], int, Optional[int]]]:
        """La coda alla mia stazione come la vedo io: (precedenza, indice, posto), dal primo all'ultimo."""
        queue = [(self._queue_priority(self.station_slot, self.position, self.refuel_claim_age, self.idx),
                  self.idx, self.station_slot)]
        for other_idx, message in self.neighbors_heard.items():
            if message.reloading and message.water_station_idx == self.water_station_idx:
                queue.append((self._queue_priority(message.station_slot, message.position,
                                                   message.refuel_claim_age, other_idx),
                              other_idx, message.station_slot))
        queue.sort()
        return queue

    def _busiest_known_load(self, station_idx: int) -> int:
        """Quanti droni, tra quelli che sento, sono diretti a questa stazione."""
        return sum(1 for message in self.neighbors_heard.values()
                   if message.reloading and message.water_station_idx == station_idx)

    def _choose_station(self) -> int:
        """Sceglie dove andare a fare acqua: prima quelle non sature, poi la più vicina."""
        options = []
        for station_idx, station_position in enumerate(self.world.water_stations):
            load = self._busiest_known_load(station_idx)
            crowded = load >= self.config.WATER_STATION_CAPACITY
            options.append((crowded, np.linalg.norm(self.position - station_position), load, station_idx))
        return min(options)[3]

    def _slot_position(self, station_idx: int, slot: int) -> np.ndarray:
        """Dove si mette fisicamente chi occupa il posto numero `slot` di questa stazione."""
        station_position = self.world.water_stations[station_idx]
        if self.config.WATER_STATION_CAPACITY <= 1:
            return station_position.copy()
        angle = station_idx * math.pi / 3.0 + 2.0 * math.pi * slot / self.config.WATER_STATION_CAPACITY
        return station_position + unit_from_angle(angle) * self.config.WATER_STATION_SLOT_RADIUS

    def _update_my_slot(self) -> None:
        """Decide se tocca a me e quale posto occupo.

        Il posto è "appiccicoso": una volta preso lo si tiene fino alla fine del rifornimento,
        altrimenti la partenza di un drone farebbe spostare gli altri a metà carico. Se due droni
        scelgono lo stesso posto (succede: le informazioni hanno sempre un passo di ritardo)
        rinuncia quello con meno precedenza, e al passo dopo ne prende un altro.
        """
        queue = self._queue_at_my_station()
        my_place_in_queue = next(index for index, (_, idx, _) in enumerate(queue) if idx == self.idx)

        if my_place_in_queue >= self.config.WATER_STATION_CAPACITY:
            self.station_slot = None                  # tocca ad altri: aspetto
            return

        my_priority = queue[my_place_in_queue][0]
        others = [(priority, slot) for priority, idx, slot in queue if idx != self.idx and slot is not None]

        if self.station_slot is not None and any(slot == self.station_slot and priority < my_priority
                                                 for priority, slot in others):
            self.station_slot = None
        if self.station_slot is None:
            taken = {slot for _, slot in others}
            free = [slot for slot in range(self.config.WATER_STATION_CAPACITY) if slot not in taken]
            if free:
                self.station_slot = free[0]

    def _start_refueling_if_needed(self) -> None:
        """Sotto la soglia d'acqua si interrompe tutto e si va a fare rifornimento."""
        if self.reloading or self.is_extinguishing_fire() or self.water > self.config.LOW_WATER_THRESHOLD:
            return
        self.reloading = True
        self.water_station_idx = self._choose_station()
        self.refuel_claim_age = 0
        self.station_slot = None
        # Si annota dove si era rimasti: al ritorno si riprende da lì invece di ricominciare a caso.
        self.interrupted_fire = self.fire_target
        self.interrupted_target = self.target.copy()
        self.fire_target = None
        self.original_target = self.world.water_stations[self.water_station_idx].copy()
        self.target = self.original_target.copy()
        self.anchor_target = self.target.copy()
        self.target_velocity[:] = 0.0
        self.pid.reset()

    def _move_toward_station(self) -> None:
        """Va al proprio posto se è il suo turno, altrimenti aspetta sull'anello di attesa."""
        if self.water_station_idx is None:
            self.water_station_idx = self._choose_station()
        self.refuel_claim_age += 1
        self._update_my_slot()
        station_position = self.world.water_stations[self.water_station_idx]

        if self.station_slot is not None:
            destination = self._slot_position(self.water_station_idx, self.station_slot)
        else:
            # Si aspetta sull'anello, dal lato da cui si è arrivati: anche qui la spaziatura tra
            # chi aspetta la sistema l'evitamento collisioni, senza assegnare posti.
            side = normalize(self.position - station_position)
            if np.linalg.norm(side) < 1e-6:
                side = unit_from_angle(self.idx * GOLDEN_ANGLE)
            destination = self._inside_area(station_position + side * self.config.WATER_STATION_WAIT_RADIUS)

        self.target = destination
        self.anchor_target = destination.copy()
        self.target_velocity[:] = 0.0
        self._fly()

    def try_reload(self) -> None:
        """Carica acqua, ma solo se è davvero il suo turno ed è arrivato al proprio posto."""
        if not self.reloading or self.station_slot is None or not self.has_reached_target():
            return
        if not self._is_in_service_area(self.position, self.water_station_idx):
            return

        self.water = min(self.config.DRONE_WATER_CAPACITY,
                         self.water + self.config.WATER_STATION_REFILL_RATE * self.config.SIM_TIME_STEP)
        if self.water < self.config.DRONE_WATER_CAPACITY - 1e-6:
            return

        # Serbatoio pieno: libera il posto e riprende il lavoro interrotto.
        self.water = self.config.DRONE_WATER_CAPACITY
        self.reloading = False
        self.water_station_idx = None
        self.refuel_claim_age = 0
        self.station_slot = None
        self.fire_target = None
        self.target_velocity[:] = 0.0
        self._head_to(self._point_to_resume_from())
        self.interrupted_fire = None
        self.interrupted_target = None
        self.pid.reset()

    def _point_to_resume_from(self) -> np.ndarray:
        """Dove tornare dopo aver fatto acqua.

        Prima scelta: l'incendio di cui ci si stava occupando, se nessuno nel frattempo ha detto che
        è spento. Serve perché il viaggio fino alla stazione dura spesso più di FIRE_MEMORY_TTL_S:
        senza questo promemoria il drone tornerebbe operativo avendo dimenticato l'emergenza che
        stava gestendo, e ripartirebbe a caso.

        Attenzione: l'incendio NON viene rimesso tra quelli "noti". Il drone non ha osservato nulla
        di nuovo, ha solo intenzione di andare a controllare; rimetterlo in memoria significherebbe
        raccontare agli altri un avvistamento mai avvenuto. Arrivato sul posto, o lo vede (e la
        notizia riparte fresca) o non lo vede (e parte la smentita).
        """
        if self.interrupted_fire is not None and self.interrupted_fire not in self.extinguished_fires:
            return np.array(self.interrupted_fire, dtype=float)
        if self.interrupted_target is not None:
            return self.interrupted_target.copy()
        return self._patrol_point()

    # --- 11. SPEGNIMENTO ----------------------------------------------------

    def is_extinguishing_fire(self) -> bool:
        return self.water > 0.0 and self.world.has_active_fire_near(self.position,
                                                                    self.config.FIRE_EXTINGUISH_RADIUS)

    def try_extinguish(self) -> None:
        """Spruzza acqua sugli incendi a portata e registra quelli che si spengono."""
        if self.water <= 0.0:
            return
        used, extinguished = self.world.request_extinguish(self.position, self.water,
                                                           self.config.DRONE_WATER_FLOW_RATE)
        self.water -= used
        self.water_delivered += used
        if self.water < 1e-9:      # residui di arrotondamento: il serbatoio è vuoto
            self.water = 0.0

        # Il drone ha visto con i propri occhi spegnersi questi incendi: lo annota, e da qui la
        # notizia si propaga ai vicini, che smetteranno di volarci verso.
        for fire_pos in extinguished:
            self.known_fires.pop(fire_pos, None)
            self.saturated_fires.pop(fire_pos, None)
            self.extinguished_fires[fire_pos] = 0
            if self.fire_target == fire_pos:
                self.fire_target = None

    # --- 12. DECISIONE E PASSO ---------------------------------------------

    def decide_and_move(self) -> None:
        """La scala di priorità: rifornimento, poi incendio, poi perlustrazione."""
        self._start_refueling_if_needed()
        if self.reloading:
            self._move_toward_station()
            return

        chosen_fire, crowded_fire_nearby = self._choose_fire()
        if chosen_fire is not None:
            self._engage_fire(chosen_fire)
        else:
            self.fire_target = None
            if crowded_fire_nearby is not None and self.config.SATURATION_BOUNCE:
                self._back_away_from(crowded_fire_nearby)
            self._move_target()        # perlustrazione: l'obiettivo si sposta nel campo di forze

        self._fly()
        self._maybe_pick_new_patrol_point()

    def step(self) -> None:
        """Un passo di simulazione di questo drone."""
        if self.status is DroneStatus.DESTROYED:
            return

        self.mailbox.begin_round()

        if self.status is DroneStatus.GROUNDED:
            # È a terra ma la radio funziona: non vede e non agisce più, però il suo orologio va
            # avanti. Continua a invecchiare e a ritrasmettere ciò che sa, quindi resta un
            # ripetitore fermo, utile allo sciame.
            self._age_memory()
            self.merge_neighbor_knowledge()
            return

        self.sense_environment()
        self.merge_neighbor_knowledge()
        self.decide_and_move()
        self.try_extinguish()
        self.try_reload()
