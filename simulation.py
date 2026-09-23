"""
LA SIMULAZIONE: far girare il mondo, e guardarlo.

Tre cose, in quest'ordine:

    1. Simulation   prepara lo scenario e porta avanti la missione. Il metodo che conta è run():
                    gli si dicono i limiti (quanto tempo al massimo, quando arrendersi) e
                    restituisce com'è andata. Chi lo chiama non scrive nessun ciclo.
    2. Gli urti     due droni che si toccano si rompono davvero: _handle_collisions().
    3. Renderer     la finestra Pygame. È un "osservatore": guarda la simulazione mentre va
                    avanti, non la comanda.

Un passo di simulazione (_advance_one_step) ha sempre lo stesso ordine:

    chi sente chi  ->  tutti trasmettono  ->  tutti ragionano e si muovono
                   ->  gli incendi crescono  ->  si controllano gli urti

Separare "tutti trasmettono" da "tutti ragionano" è ciò che rende la simulazione equa: nessun
drone ragiona sapendo già cosa ha appena deciso il drone prima di lui.

Ordine di lettura del progetto: world.py, drone.py, questo file.
"""

import random
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Protocol, Sequence, Set, Tuple

import numpy as np

from drone import CoverageMemory, Drone
from world import (DEFAULT_CONFIG, DroneStatus, Fire, ImportanceMap, SimConfig, SimulationWorld,
                   clamp_magnitude)


class MissionOutcome(Enum):
    """Come può finire una missione."""
    ALL_FIRES_OUT = "tutti gli incendi spenti"
    OUT_OF_CONTROL = "incendi fuori controllo"
    TIME_LIMIT = "tempo scaduto"
    SWARM_DOWN = "nessun drone ancora in volo"
    STOPPED = "interrotta"          # la finestra è stata chiusa


@dataclass
class MissionResult:
    """Il verdetto di una missione: quello che chi lancia la simulazione vuole sapere."""
    outcome: MissionOutcome
    elapsed_s: float
    fires_left: int
    fires_extinguished: int
    drones_flying: int
    drones_lost: int
    collisions: int

    @property
    def success(self) -> bool:
        return self.outcome is MissionOutcome.ALL_FIRES_OUT

    def describe(self) -> str:
        if self.success:
            return f"riuscita in {self.elapsed_s:.0f} s"
        return f"{self.outcome.value} dopo {self.elapsed_s:.0f} s ({self.fires_left} incendi accesi)"


class Watcher(Protocol):
    """Chi vuole guardare la simulazione mentre va avanti, senza comandarla.

    Lo implementano la finestra (per disegnare) e le misure (per registrare). Il metodo viene
    chiamato dopo ogni passo; se restituisce False la simulazione si ferma.
    """

    def after_step(self, simulation: "Simulation") -> Optional[bool]: ...


class Simulation:
    """Uno scenario completo: l'area, gli incendi, le stazioni e lo sciame.

        simulation = Simulation(seed=3)
        result = simulation.run(max_time_s=300)
        print(result.describe())
    """

    def __init__(self, seed: Optional[int] = None, random_fires: bool = False,
                 random_stations: bool = False, config: SimConfig = DEFAULT_CONFIG,
                 log_collisions: bool = False):
        self.config = config
        self.seed = config.RANDOM_SEED if seed is None else seed
        self.log_collisions = log_collisions
        # Un solo generatore casuale per lo scenario: decide posizioni iniziali e propagazione.
        # Droni, radio e accensioni spontanee ne hanno uno proprio, così cambiare il comportamento
        # dei droni non cambia lo scenario in cui vengono confrontati.
        self.rng = random.Random(self.seed)

        self.fires = self._random_fires() if random_fires else self._default_fires()
        self.water_stations = self._random_stations() if random_stations else self._default_stations()
        self.terrain = ImportanceMap(config)
        self.drones: List[Drone] = []
        self.world = SimulationWorld(config, self.drones, self.fires, self.water_stations,
                                     self.terrain, self.seed)
        self.drones.extend(Drone(idx, self.rng, self.world, self.seed) for idx in range(config.NUM_DRONES))

        self.collisions = 0              # urti avvenuti (una coppia che entra in contatto conta 1)
        self.contact_steps = 0           # quanto a lungo, in totale, i droni sono rimasti sovrapposti
        self.step_collisions = 0
        self._pairs_touching: Set[Tuple[int, int]] = set()
        self.neighbors_now: Dict[int, List[Drone]] = {}
        # Distanze tra tutte le coppie di droni, ricalcolate a ogni passo e riusate dalle misure.
        # Le coppie sono sempre le stesse, nello stesso ordine, anche quando un drone precipita:
        # chi non vola più risulta a distanza infinita, così non conta né come urto né come
        # quasi-urto, ma l'indice di ogni coppia resta stabile per chi confronta un passo con l'altro.
        # Generatore dedicato ai guasti provocati: separato da tutto il resto.
        self.failure_rng = random.Random(f"failure-{self.seed}")
        self._failures_injected = False
        self._pair_first, self._pair_second = np.triu_indices(len(self.drones), k=1)
        self.pair_distances = np.full(len(self._pair_first), np.inf)

    # --- Lo scenario di partenza -------------------------------------------

    def _new_fire(self, position: np.ndarray) -> Fire:
        return Fire(pos=position, health=self.config.FIRE_HEALTH, growth_rate=self.config.FIRE_GROWTH_RATE)

    def _default_fires(self) -> List[Fire]:
        """Incendi in posizioni fisse: lo scenario di riferimento, sempre uguale.

        Se ne vengono chiesti più di quanti siano i posti previsti, i restanti sono casuali.
        """
        width, height = self.config.AREA_WIDTH, self.config.AREA_HEIGHT
        positions = [(width * 0.20, height * 0.75), (width * 0.50, height * 0.25),
                     (width * 0.80, height * 0.75), (width * 0.35, height * 0.35),
                     (width * 0.65, height * 0.80), (width * 0.90, height * 0.40)]
        wanted = self.config.NUM_FIRES
        fires = [self._new_fire(np.array(position, dtype=float)) for position in positions[:wanted]]
        if len(fires) < wanted:
            fires += self._random_fires()[:wanted - len(fires)]
        return fires

    def _random_fires(self) -> List[Fire]:
        margin = self.config.FIRE_GENERATION_MARGIN
        return [self._new_fire(np.array([self.rng.uniform(margin, self.config.AREA_WIDTH - margin),
                                         self.rng.uniform(margin, self.config.AREA_HEIGHT - margin)], dtype=float))
                for _ in range(self.config.NUM_FIRES)]

    # I droni estraggono le proprie posizioni iniziali da un generatore personale (vedi Drone):
    # così cambiare il numero di droni non sposta la sequenza casuale degli incendi, e due varianti
    # con lo stesso seed affrontano davvero lo stesso scenario.

    def _default_stations(self) -> List[np.ndarray]:
        width, height = self.config.AREA_WIDTH, self.config.AREA_HEIGHT
        positions = [(width * 0.20, height * 0.25), (width * 0.50, height * 0.75), (width * 0.80, height * 0.25)]
        return [np.array(position, dtype=float) for position in positions[:self.config.NUM_WATER_STATIONS]]

    def _random_stations(self) -> List[np.ndarray]:
        """Stazioni a caso, ma mai troppo vicine tra loro (altrimenti le code si sovrappongono)."""
        margin = self.config.WATER_STATION_GENERATION_MARGIN
        wanted = self.config.NUM_WATER_STATIONS
        stations: List[np.ndarray] = []
        for _ in range(wanted * 100):
            if len(stations) >= wanted:
                break
            candidate = np.array([self.rng.uniform(margin, self.config.AREA_WIDTH - margin),
                                  self.rng.uniform(margin, self.config.AREA_HEIGHT - margin)], dtype=float)
            if all(np.linalg.norm(candidate - other) >= self.config.WATER_STATION_MIN_SEPARATION
                   for other in stations):
                stations.append(candidate)
        if len(stations) == wanted:
            return stations
        return self._default_stations()[:wanted]   # area troppo piccola: si ripiega sulle posizioni fisse

    # --- Stato ---------------------------------------------------------------

    @property
    def step_count(self) -> int:
        """Quanti battiti sono passati. Il tempo lo tiene il clock, non la simulazione."""
        return self.world.clock.step_count

    @property
    def sim_time(self) -> float:
        """Secondi simulati dall'inizio."""
        return self.world.clock.now_s

    @property
    def flying_drones(self) -> List[Drone]:
        return [drone for drone in self.drones if drone.is_flying]

    @property
    def drones_lost(self) -> int:
        return sum(1 for drone in self.drones if not drone.is_flying)

    def summary(self) -> str:
        """Una riga di stato leggibile, usata dalla finestra e dalla modalità senza finestra."""
        text = (f"t={self.sim_time:.1f}s  incendi accesi={len(self.world.fires)}  "
                f"spenti={self.world.extinguished_count}  urti={self.collisions}")
        lost = self.drones_lost
        if lost:
            silent = sum(1 for drone in self.drones if drone.status is DroneStatus.SILENT)
            with_radio = sum(1 for drone in self.drones if drone.radio_works and not drone.is_flying)
            text += f"  droni fuori uso={lost} (radio ancora attiva: {with_radio}"
            text += f", di cui {silent} che si dichiarano in volo)" if silent else ")"
        return text

    # --- Il ciclo -------------------------------------------------------------

    def run(self, max_time_s: Optional[float] = None, max_active_fires: Optional[int] = None,
            watchers: Sequence[Watcher] = ()) -> MissionResult:
        """Porta avanti la missione fino alla fine e dice com'è andata.

        Si ferma alla prima di queste condizioni:
            - tutti gli incendi sono spenti (e non ne possono nascere di spontanei);
            - gli incendi accesi superano `max_active_fires`: situazione fuori controllo;
            - nessun drone è più in volo;
            - è passato `max_time_s`;
            - un osservatore ha chiesto di fermarsi (per esempio: finestra chiusa).

        `max_time_s=None` vuol dire "senza limite": è il caso della finestra interattiva, che va
        avanti finché l'utente non la chiude.
        """
        outcome: Optional[MissionOutcome] = None
        while outcome is None:
            self._advance_one_step()

            stop_requested = False
            for watcher in watchers:
                if watcher.after_step(self) is False:
                    stop_requested = True

            if stop_requested:
                outcome = MissionOutcome.STOPPED
            elif self._all_fires_out():
                outcome = MissionOutcome.ALL_FIRES_OUT
            elif max_active_fires is not None and len(self.world.fires) > max_active_fires:
                outcome = MissionOutcome.OUT_OF_CONTROL
            elif not self.flying_drones:
                outcome = MissionOutcome.SWARM_DOWN
            elif max_time_s is not None and self.sim_time >= max_time_s:
                outcome = MissionOutcome.TIME_LIMIT

        return MissionResult(outcome=outcome, elapsed_s=self.sim_time, fires_left=len(self.world.fires),
                             fires_extinguished=self.world.extinguished_count,
                             drones_flying=len(self.flying_drones), drones_lost=self.drones_lost,
                             collisions=self.collisions)

    def _all_fires_out(self) -> bool:
        """Nessun incendio acceso E nessuno che possa accendersi da solo: la missione è finita.

        Con le accensioni spontanee attive, invece, "nessun incendio" è solo una tregua.
        """
        return not self.world.fires and self.config.IGNITION_RATE_PER_S <= 0.0

    def _advance_one_step(self) -> None:
        """Un passo: si aggiorna chi sente chi, batte il clock, poi il mondo ne trae le conseguenze.

        Il clock non è un capo che comanda i droni: è solo il tempo che passa. Sono i droni che,
        alla nascita, hanno chiesto di essere svegliati a ogni battito (vedi Clock in world.py).
        """
        self.neighbors_now = self.world.refresh_neighbors()
        self.world.clock.tick()
        self.world.update_fires(self.rng)
        self._handle_collisions()
        self._inject_failures()
        self._burn_wrecks()

    # --- Urti ------------------------------------------------------------------

    def _handle_collisions(self) -> None:
        """Trova i droni che si sono toccati e applica i danni.

        Non esiste nessuna "risposta all'urto" che li separi: se si toccano è perché l'evitamento
        locale ha fallito, e il danno è la conseguenza. La gravità dipende dalla velocità RELATIVA
        con cui si sono avvicinati, non da quella rispetto al terreno: due droni che viaggiano
        affiancati e si sfiorano fanno molto meno danno di due che si vengono incontro.

            impatto sotto COLLISION_TOTAL_LOSS_SPEED  ->  motori rotti, radio ancora viva
            impatto sopra                             ->  perdita totale
        """
        self.step_collisions = 0
        if len(self.drones) < 2:
            return

        first, second = self._pair_first, self._pair_second
        positions = np.array([drone.position for drone in self.drones])
        offsets = positions[first] - positions[second]
        distances = np.sqrt(np.einsum("ij,ij->i", offsets, offsets))
        in_flight = np.array([drone.is_flying for drone in self.drones])
        self.pair_distances = np.where(in_flight[first] & in_flight[second], distances, np.inf)

        # Prima si registra tutto quello che è successo, POI si applicano i danni: rompere un drone
        # gli azzera la velocità, e se lo stesso drone è coinvolto in due urti nello stesso passo il
        # secondo impatto risulterebbe più lento di quello che è stato davvero.
        still_touching: Set[Tuple[int, int]] = set()
        new_impacts: List[Tuple[Drone, Drone, float]] = []
        for pair_index in np.flatnonzero(self.pair_distances < self.config.DRONE_IMPACT_RADIUS):
            one, other = self.drones[first[pair_index]], self.drones[second[pair_index]]
            pair = (one.idx, other.idx)
            still_touching.add(pair)
            self.step_collisions += 1
            self.contact_steps += 1
            if pair in self._pairs_touching:
                continue                       # stesso urto del passo precedente, non uno nuovo

            self.collisions += 1
            impact_speed = float(np.linalg.norm(one.velocity - other.velocity))
            new_impacts.append((one, other, impact_speed))
            if self.log_collisions:
                print(f"[urto] t={self.sim_time:.2f}s droni {pair} "
                      f"distanza={self.pair_distances[pair_index]:.3f} m "
                      f"impatto={impact_speed:.2f} m/s")

        if self.config.COLLISION_DAMAGE:
            for one, other, impact_speed in new_impacts:
                radio_destroyed = impact_speed >= self.config.COLLISION_TOTAL_LOSS_SPEED
                one.break_down(radio_destroyed)
                other.break_down(radio_destroyed)

        self._pairs_touching = still_touching

    def _inject_failures(self) -> None:
        """Rompe di proposito qualche drone a un istante prefissato (se richiesto dai parametri).

        Serve a studiare la resilienza senza dipendere dal caso: con l'evitamento acceso gli urti
        non avvengono quasi mai, quindi per sapere come reagisce lo sciame alla perdita di k droni
        bisogna provocarla. Il momento e le vittime sono decisi da un generatore dedicato, così
        accendere l'opzione non sposta nient'altro.
        """
        if self.config.FAILURE_INJECTION_COUNT <= 0 or self._failures_injected:
            return
        if self.sim_time < self.config.FAILURE_INJECTION_TIME_S:
            return
        self._failures_injected = True
        victims = self.failure_rng.sample(self.flying_drones,
                                          min(self.config.FAILURE_INJECTION_COUNT, len(self.flying_drones)))
        for victim in victims:
            victim.break_down(radio_destroyed=self.config.FAILURE_INJECTION_RADIO_OFF)

    def _burn_wrecks(self) -> None:
        """Un relitto caduto dentro un incendio brucia: dopo WRECK_BURN_TIME_S tace anche la radio."""
        for drone in self.drones:
            if drone.status not in (DroneStatus.GROUNDED, DroneStatus.SILENT):
                continue
            if not self.world.has_active_fire_near(drone.position, self.config.FIRE_EXTINGUISH_RADIUS):
                continue
            drone.burning_steps += 1
            if drone.burning_steps >= self.config.WRECK_BURN_STEPS:
                drone.break_down(radio_destroyed=True)


# ============================================================
# LA FINESTRA
# ============================================================

WINDOW_WIDTH = 1600
WINDOW_HEIGHT = 960          # stesse proporzioni dell'area di default (20 × 12 m)

TARGET_LINE_COLOR = (255, 110, 255, 100)     # tratteggio drone -> obiettivo
ANCHOR_LINE_COLOR = (110, 250, 110, 100)     # tratteggio drone -> ancora
RADIO_LINE_COLOR = (100, 160, 220, 60)       # chi sta parlando con chi
HEATMAP_MODES = ("", "importanza", "obsolescenza")
HEATMAP_FULL_SCALE_S = 60.0                  # obsolescenza alla quale la mappa è tutta accesa


class Renderer:
    """Disegna la simulazione in una finestra Pygame. Guarda soltanto: non decide niente.

    Tasti: P pausa, V vettori delle forze, C collegamenti radio, H mappa del terreno, Q esci.
    """

    def __init__(self, simulation: Simulation, show_forces: bool = False, show_radio: bool = False,
                 frames_per_second: int = 60):
        import pygame      # importato qui: senza finestra il progetto non richiede Pygame
        self.pygame = pygame
        self.simulation = simulation
        self.config = simulation.config
        self.show_forces = show_forces
        self.show_radio = show_radio
        self.heatmap_mode = 0
        self.paused = False
        self.frames_per_second = frames_per_second

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.transparent_layer = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA)
        self.font = pygame.font.SysFont("monospace", 15)
        self.clock = pygame.time.Clock()
        pygame.display.set_caption("X-Coverage — sciame di droni antincendio")

        # Mappa "dove è passato qualcuno", ricostruita qui dalle posizioni vere dei droni: è ciò
        # che vede l'osservatore esterno, non la credenza di un singolo drone.
        self.seen_by_anyone = CoverageMemory(simulation.terrain.shape)

    # --- Osservatore ---------------------------------------------------------

    def after_step(self, simulation: Simulation) -> Optional[bool]:
        """Chiamato dopo ogni passo: disegna, legge i tasti, e dice se continuare."""
        if not self._handle_events():
            return False
        while self.paused:                  # in pausa si continua a disegnare, ma il mondo è fermo
            if not self._handle_events():
                return False
            self.draw()
            self.clock.tick(self.frames_per_second)
        self.draw()
        self.clock.tick(self.frames_per_second)
        return True

    def _handle_events(self) -> bool:
        """Legge tastiera e mouse. Restituisce False se l'utente vuole uscire."""
        pygame = self.pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    self.paused = not self.paused
                elif event.key == pygame.K_v:
                    self.show_forces = not self.show_forces
                elif event.key == pygame.K_c:
                    self.show_radio = not self.show_radio
                elif event.key == pygame.K_h:
                    self.heatmap_mode = (self.heatmap_mode + 1) % len(HEATMAP_MODES)
        return True

    def close(self) -> None:
        self.pygame.quit()

    # --- Da metri a pixel -----------------------------------------------------

    def to_screen(self, position: np.ndarray) -> Tuple[int, int]:
        """L'asse Y va capovolto: nel mondo cresce verso l'alto, sullo schermo verso il basso."""
        return (int(position[0] / self.config.AREA_WIDTH * WINDOW_WIDTH),
                int((1.0 - position[1] / self.config.AREA_HEIGHT) * WINDOW_HEIGHT))

    def to_pixels(self, length_in_meters: float) -> int:
        return max(1, int(length_in_meters / self.config.AREA_WIDTH * WINDOW_WIDTH))

    # --- Disegno --------------------------------------------------------------

    def draw(self) -> None:
        self.screen.fill((20, 20, 25))
        self.transparent_layer.fill((0, 0, 0, 0))
        self._update_seen_map()
        if self.heatmap_mode:
            self._draw_heatmap()
        self._draw_stations()
        self._draw_fires()
        self._draw_radio_links()
        self._draw_intentions()
        self.screen.blit(self.transparent_layer, (0, 0))
        self._draw_drones()
        self._draw_status_bar()
        self.pygame.display.flip()

    def _update_seen_map(self) -> None:
        for drone in self.simulation.flying_drones:
            self.seen_by_anyone.mark_seen(
                self.simulation.terrain.cells_within(drone.position, self.config.FIRE_DETECTION_RADIUS),
                self.simulation.step_count)

    def _draw_heatmap(self) -> None:
        """Sfondo: quanto vale il terreno, oppure da quanto tempo nessuno lo guarda."""
        if HEATMAP_MODES[self.heatmap_mode] == "importanza":
            level = self.simulation.terrain.grid
            tint = (110.0, 38.0, 14.0)      # arancione: dove conta tenere d'occhio
        else:
            seconds = self.seen_by_anyone.seconds_since_seen(self.simulation.step_count,
                                                             self.config.SIM_TIME_STEP,
                                                             HEATMAP_FULL_SCALE_S)
            level = np.clip(seconds / HEATMAP_FULL_SCALE_S, 0.0, 1.0)
            tint = (28.0, 48.0, 105.0)      # blu: da quanto non ci passa nessuno
        colors = np.empty(level.shape + (3,), dtype=np.uint8)
        for channel in range(3):
            colors[:, :, channel] = (20 + level * tint[channel]).astype(np.uint8)
        # La griglia ha x sulle righe, come Pygame: va solo capovolta in verticale.
        surface = self.pygame.surfarray.make_surface(np.flip(colors, axis=1))
        self.screen.blit(self.pygame.transform.smoothscale(surface, (WINDOW_WIDTH, WINDOW_HEIGHT)), (0, 0))

    def _draw_stations(self) -> None:
        for station_idx, position in enumerate(self.simulation.water_stations):
            x, y = self.to_screen(position)
            self.pygame.draw.circle(self.screen, (30, 100, 200), (x, y),
                                    self.to_pixels(self.config.WATER_STATION_SERVICE_RADIUS), 2)
            self.pygame.draw.circle(self.transparent_layer, (30, 100, 200, 70), (x, y),
                                    self.to_pixels(self.config.WATER_STATION_WAIT_RADIUS), 1)
            label = self.font.render(f"ACQUA {station_idx}", True, (100, 180, 255))
            self.screen.blit(label, (x - 30, y - self.to_pixels(self.config.WATER_STATION_SERVICE_RADIUS) - 18))

    def _draw_fires(self) -> None:
        for fire in self.simulation.world.fires:
            x, y = self.to_screen(fire.pos)
            self.pygame.draw.circle(self.screen, (255, 140, 40), (x, y),
                                    self.to_pixels(self.config.FIRE_DETECTION_RADIUS), 1)
            water_radius = self.to_pixels(self.config.FIRE_EXTINGUISH_RADIUS)
            self.pygame.draw.circle(self.screen, (200, 80, 0), (x, y), water_radius, 1)

            intensity = max(0.0, min(fire.health, self.config.FIRE_HEALTH) / self.config.FIRE_HEALTH)
            color = (int(100 + 155 * intensity), int(200 - 120 * intensity), int(100 - 100 * intensity), 50)
            self._filled_circle(color, (x, y), water_radius)
            self.screen.blit(self.font.render(f"{fire.health:.0f}", True, (255, 200, 50)), (x - 10, y - 30))

    def _draw_radio_links(self) -> None:
        if not self.show_radio:
            return
        for drone in self.simulation.drones:
            for other in self.simulation.neighbors_now.get(drone.idx, []):
                if other.idx > drone.idx:
                    self.pygame.draw.line(self.transparent_layer, RADIO_LINE_COLOR,
                                          self.to_screen(drone.position), self.to_screen(other.position), 1)

    def _draw_intentions(self) -> None:
        """Tratteggi che mostrano dove ogni drone sta andando e attorno a quale punto gravita."""
        for drone in self.simulation.flying_drones:
            start = self.to_screen(drone.position)
            self._dashed_line(TARGET_LINE_COLOR, start, self.to_screen(drone.target), 6, 5)
            self._dashed_line(ANCHOR_LINE_COLOR, start, self.to_screen(drone.anchor_target), 4, 5)

    def _draw_drones(self) -> None:
        for drone in self.simulation.drones:
            x, y = self.to_screen(drone.position)

            if not drone.is_flying:
                self._draw_wreck(drone, x, y)
                continue

            green_dot = self.to_screen(drone.original_target)
            self.pygame.draw.circle(self.screen, (50, 220, 80), green_dot, 3)
            anchor_dot = self.to_screen(drone.anchor_target)
            self.pygame.draw.circle(self.screen, (160, 160, 255), anchor_dot, 3, 1)
            target_dot = self.to_screen(drone.target)
            self.pygame.draw.circle(self.screen, (230, 50, 50), target_dot, 4)

            if self.show_forces:
                self._force_arrow(drone.target, drone.repulsion_between_targets(), (255, 200, 50), 0.2)
                self._force_arrow(drone.target, drone.anchor_force(), (50, 255, 200), 0.2)
                self._force_arrow(drone.target, drone.boundary_force(), (50, 50, 230), 0.2)
                self._force_arrow(drone.target, drone.last_applied_force, (255, 255, 255), 0.2)
                # Questa agisce sul drone, non sull'obiettivo: è la correzione anti-collisione.
                self._force_arrow(drone.position, drone.last_avoidance_force, (255, 80, 255), 0.5)

            if drone.is_extinguishing_fire():
                self.pygame.draw.circle(self.screen, (255, 230, 20), (x, y), 12, 2)
            if drone.reloading and drone.station_slot is not None:
                self.pygame.draw.circle(self.screen, (20, 200, 255), (x, y), 14, 2)

            water_left = drone.water / self.config.DRONE_WATER_CAPACITY
            color = (int(255 * (1.0 - water_left)), int(180 * water_left + 50), int(255 * water_left))
            self.pygame.draw.circle(self.screen, color, (x, y), self.to_pixels(self.config.DRONE_IMPACT_RADIUS))

    def _draw_wreck(self, drone: Drone, x: int, y: int) -> None:
        """Un drone precipitato: una X. Grigia se la radio funziona ancora, rossa se è spento del tutto."""
        radio_alive = drone.status is DroneStatus.GROUNDED
        color = (150, 150, 150) if radio_alive else (120, 40, 40)
        size = 7
        self.pygame.draw.line(self.screen, color, (x - size, y - size), (x + size, y + size), 2)
        self.pygame.draw.line(self.screen, color, (x - size, y + size), (x + size, y - size), 2)
        if radio_alive:
            self.pygame.draw.circle(self.transparent_layer, (150, 150, 150, 60), (x, y), size + 5, 1)

    def _draw_status_bar(self) -> None:
        status = self.simulation.summary() + ("   [PAUSA]" if self.paused else "")
        self.screen.blit(self.font.render(status, True, (255, 255, 0)), (10, 10))

        keys = "P pausa  V forze  C radio  H mappa  Q esci"
        if self.heatmap_mode:
            keys += f"  [{HEATMAP_MODES[self.heatmap_mode]}]"
        for note in self._non_default_settings():
            keys += f"   |  {note}"
        self.screen.blit(self.font.render(keys, True, (150, 150, 150)), (10, 28))

    def _non_default_settings(self) -> List[str]:
        """Ricorda a chi guarda che questa non è la configurazione standard."""
        config = self.config
        notes = []
        if config.AVOIDANCE_MODE != "full":
            notes.append(f"evitamento={config.AVOIDANCE_MODE}")
        if config.PACKET_LOSS > 0.0:
            notes.append(f"messaggi persi={config.PACKET_LOSS:.0%}")
        if config.EXPLORATION_MODE != "random":
            notes.append(f"perlustrazione=copertura (γ={config.COVERAGE_IMPORTANCE_EXPONENT:g})")
        if config.IGNITION_RATE_PER_S > 0.0:
            notes.append(f"accensioni={config.IGNITION_RATE_PER_S:g}/s")
        if not config.COLLISION_DAMAGE:
            notes.append("urti senza danni")
        return notes

    # --- Primitive di disegno -------------------------------------------------

    def _filled_circle(self, color, center, radius: int) -> None:
        """Cerchio semitrasparente (Pygame lo sa fare solo su una superficie a parte)."""
        radius = max(1, int(radius))
        patch = self.pygame.Surface((radius * 2, radius * 2), self.pygame.SRCALPHA)
        self.pygame.draw.circle(patch, color, (radius, radius), radius)
        self.screen.blit(patch, (center[0] - radius, center[1] - radius))

    def _dashed_line(self, color, start, end, dash: int, gap: int) -> None:
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = max(1.0, float(np.hypot(dx, dy)))
        for segment in range(int(length // (dash + gap))):
            begin = segment * (dash + gap) / length
            finish = min(1.0, (segment * (dash + gap) + dash) / length)
            self.pygame.draw.line(self.transparent_layer, color,
                                  (start[0] + dx * begin, start[1] + dy * begin),
                                  (start[0] + dx * finish, start[1] + dy * finish), 1)

    def _force_arrow(self, origin: np.ndarray, vector: np.ndarray, color, scale: float) -> None:
        if np.linalg.norm(vector) < 1e-8:
            return
        tip = origin + clamp_magnitude(vector, self.config.MAX_FORCE_ON_TARGET) * scale
        self.pygame.draw.line(self.screen, color, self.to_screen(origin), self.to_screen(tip), 1)


def run_with_window(simulation: Simulation, show_forces: bool = False,
                    show_radio: bool = False) -> MissionResult:
    """Apre la finestra e fa girare la simulazione finché l'utente non chiude (tasto Q)."""
    renderer = Renderer(simulation, show_forces, show_radio)
    try:
        return simulation.run(watchers=[renderer])
    finally:
        renderer.close()
