"""
IL MONDO: i parametri, il terreno, gli incendi, la radio.

Questo file contiene tutto ciò che esiste indipendentemente dai droni. Si legge dall'alto in basso:

    1. Parametri        SimConfig: ogni numero della simulazione, con il suo commento.
    2. Vettori          quattro funzioni di supporto usate ovunque.
    3. Incendio         un fuoco che cresce finché qualcuno non lo spegne.
    4. Terreno          la mappa di importanza: quanto vale tenere d'occhio ogni zona.
    5. Clock            il battito condiviso: scandisce il tempo di tutti i droni.
    6. Radio            il messaggio che un drone trasmette e la cassetta postale di chi lo riceve.
    7. Mondo            sensori e attuatori che il mondo mette a disposizione dei droni.

Il mondo conosce tutto perché È la realtà fisica simulata. I droni no: vedono solo ciò che il
mondo risponde ai loro sensori e ciò che arriva dalla radio. Il mondo non decide mai nulla al
posto loro: qui non c'è una riga che assegni un incendio a un drone.

Chi legge il progetto per la prima volta: dopo questo file, drone.py.
"""

import dataclasses
import itertools
import math
import random
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

import numpy as np

if TYPE_CHECKING:
    from drone import Drone

# Posizione di un incendio, usata come chiave nelle memorie dei droni: (x, y) in metri.
FirePos = Tuple[float, float]

# Angolo aureo: serve a distribuire direzioni "di ripiego" diverse per ogni drone.
GOLDEN_ANGLE = 2.399963


class DroneStatus(Enum):
    """In che stato è la macchina. Un drone danneggiato non torna mai a volare.

    I tre modi di rompersi non sono ordinati per gravità del danno, ma per quanto l'avaria è
    ONESTA verso gli altri — ed è quella, non il danno, a decidere quanto costa allo sciame:

        GROUNDED   è caduto e lo dice. Gli altri gli passano sopra e non contano più su di lui.
        DESTROYED  è caduto e tace. Per lo sciame è come se non fosse mai esistito.
        SILENT     è caduto e continua a dire che sta volando. È il caso peggiore: gli altri lo
                   scansano, aspettano il suo turno, gli lasciano l'incendio che aveva preso in
                   carico, e continuano a ripetersi le notizie che aveva in memoria nell'istante
                   dell'urto, come se fossero appena state confermate.
    """
    FLYING = "in volo"
    GROUNDED = "a terra"      # motori distrutti: è precipitato, ma la radio funziona ancora
    DESTROYED = "distrutto"   # anche la radio è spenta: per lo sciame è come se non esistesse
    SILENT = "guasto silenzioso"   # è precipitato ma continua a trasmettere l'ultimo stato


# Valori ammessi per i parametri che scelgono tra comportamenti alternativi.
AVOIDANCE_MODES = ("full", "emergency-only", "none")
EXPLORATION_MODES = ("random", "coverage")


# ============================================================
# 1. PARAMETRI
# ============================================================

@dataclass(frozen=True)
class SimConfig:
    """Tutti i numeri della simulazione, in un oggetto che non si può modificare dopo averlo creato.

    Viene passato esplicitamente a mondo, droni e simulazione, invece di stare in variabili globali:
    così due simulazioni con parametri diversi possono convivere nello stesso processo, ed è
    possibile lanciare in parallelo decine di varianti (vedi experiments.py).

        SimConfig()                          i valori di default qui sotto
        SimConfig(NUM_DRONES=20)             uno cambiato
        config.with_overrides(NUM_DRONES=20) copia di una configurazione, con una modifica
    """

    # --- Simulazione -------------------------------------------------------
    NUM_DRONES: int = 12
    AREA_WIDTH: float = 20.0
    AREA_HEIGHT: float = 12.0
    SIM_TIME_STEP: float = 0.01        # durata di un passo di simulazione [s]
    RANDOM_SEED: int = 42

    # --- Incendi -----------------------------------------------------------
    NUM_FIRES: int = 3                 # incendi iniziali quando si chiedono posizioni casuali
    FIRE_HEALTH: float = 200.0         # "vita" di un incendio: quanta acqua serve per spegnerlo
    FIRE_GROWTH_RATE: float = 0.5      # vita guadagnata al secondo se nessuno lo spegne
    FIRE_GENERATION_MARGIN: float = 1.5           # nessun incendio nasce più vicino di così al bordo
    FIRE_SPAWN_THRESHOLD: float = 1.20            # oltre questa frazione di FIRE_HEALTH l'incendio si propaga
    FIRE_SPAWN_PROB_PER_STEP: float = 0.003       # probabilità, a ogni passo, che generi un figlio
    FIRE_SPAWN_OFFSET_MAX: float = 3.0            # a che distanza al massimo nasce il figlio [m]
    FIRE_SPAWN_INITIAL_HEALTH: float = 0.15       # vita del figlio, in frazione di FIRE_HEALTH
    # Accensioni spontanee: incendi che nascono da soli, lontano da quelli già noti. Sono ciò che
    # rende la RICERCA un problema; con 0 gli incendi nascono solo accanto a fuochi già osservati.
    IGNITION_RATE_PER_S: float = 0.0
    IGNITION_FOLLOWS_IMPORTANCE: bool = True      # nascono più spesso dove il terreno vale di più

    # --- Percezione e comunicazione ----------------------------------------
    FIRE_DETECTION_RADIUS: float = 2.0   # entro questo raggio un drone vede un incendio
    COMMUNICATION_RADIUS: float = 2.5    # entro questo raggio due droni si parlano
    PACKET_LOSS: float = 0.0             # probabilità che un singolo messaggio non arrivi
    FIRE_MEMORY_TTL_S: float = 15.0      # per quanto un drone ricorda un incendio senza riconferme

    # --- Spegnimento -------------------------------------------------------
    DRONE_WATER_CAPACITY: float = 20.0
    DRONE_WATER_FLOW_RATE: float = 5.0            # acqua erogata al secondo
    FIRE_EXTINGUISH_RADIUS: float = 1.2           # entro questo raggio l'acqua arriva sul fuoco
    FIRE_WORK_RADIUS: float = 0.8                 # dove si ferma il drone per spegnere (anello di lavoro)
    MAX_DRONES_ON_FIRE: int = 3                   # quanti droni al massimo su uno stesso incendio
    FIRE_SATURATION_BOUNCE_DISTANCE: float = 3.5  # di quanto si allontana chi trova l'incendio già presidiato [m]
    FIRE_SATURATION_MEMORY_S: float = 5.0         # per quanto lo considera "pieno" e lo esclude

    # --- Rifornimento ------------------------------------------------------
    NUM_WATER_STATIONS: int = 3
    LOW_WATER_FRACTION: float = 0.1               # sotto questa frazione di serbatoio si va a fare acqua
    WATER_STATION_REFILL_RATE: float = 10.0       # acqua caricata al secondo
    WATER_STATION_CAPACITY: int = 2               # droni riforniti contemporaneamente (un posto ciascuno)
    WATER_STATION_SERVICE_RADIUS: float = 0.8     # area in cui si può caricare acqua
    WATER_STATION_SLOT_RADIUS: float = 0.55       # distanza dei posti dal centro della stazione
    WATER_STATION_WAIT_RADIUS: float = 1.9        # anello dove aspetta chi trova i posti occupati
    WATER_STATION_MIN_SEPARATION: float = 5.0     # distanza minima tra stazioni piazzate a caso
    # Dopo quanto un drone che non riesce a rifornirsi rinuncia e riprova da capo, magari altrove.
    # Serve contro i blocchi: per esempio un relitto caduto proprio sul posto di rifornimento.
    REFUEL_GIVE_UP_S: float = 45.0
    WATER_STATION_GENERATION_MARGIN: float = 2.0  # nessuna stazione più vicina di così al bordo

    # --- Movimento del drone -----------------------------------------------
    MAX_DRONE_SPEED: float = 1.0                  # [m/s]
    K_DESIRED_VEL_TO_TARGET: float = 0.70         # quanto forte punta al proprio obiettivo
    PID_KP: float = 3.4                           # il PID trasforma l'errore di velocità in accelerazione
    PID_KI: float = 0.6
    PID_KD: float = 0.25
    PID_INTEGRAL_LIMIT: float = 3.0
    PID_MAX_OUTPUT_ACCEL: float = 4.0             # [m/s²]
    MAX_JERK: float = 8.0                         # quanto può cambiare l'accelerazione al secondo [m/s³]

    # --- Obiettivo mobile (il "target virtuale" inseguito in perlustrazione) -----
    TARGET_SEPARATION: float = 2.0                # sotto questa distanza due obiettivi si respingono
    TARGET_REACHED_DISTANCE: float = 0.2          # a che distanza l'obiettivo si considera raggiunto
    MAX_TARGET_SPEED: float = 20.0
    MAX_FORCE_ON_TARGET: float = 10.0
    TARGET_VEL_DAMPING_RATE: float = 10.5         # attrito dell'obiettivo [1/s]: frenata = exp(-rate·dt)
    ANCHOR_TO_TARGET_INTENSITY: float = 0.0075    # quanto lentamente l'ancora insegue l'obiettivo [1/s]
    K_REPULSION_BETWEEN_TARGETS: float = 1.0
    K_ANCHOR_DRAGGING: float = 0.2
    K_BOUNDARY_REPULSION: float = 1.0
    MARGIN_REPULSION_BOUNDARY: float = 1.5        # entro questa distanza dal bordo l'obiettivo viene respinto
    MAX_IDLE_STEPS: int = 30                      # fermo sull'obiettivo per tanti passi -> se ne sceglie un altro

    # --- Distanze di sicurezza ---------------------------------------------
    # Gerarchia (verificata da validate_config): collisione < emergenza < distanza operativa < radio.
    #   0.1 m si toccano  <  0.8 m allarme  <  1.0 m distanza voluta  <  2.5 m orizzonte
    DRONE_IMPACT_RADIUS: float = 0.1              # sotto questa distanza i droni si urtano davvero
    EMERGENCY_AVOID_DISTANCE: float = 0.8         # sotto questa, scansarsi conta più del compito
    AVOID_MIN_DISTANCE: float = 1.0               # distanza che l'evitamento cerca di mantenere
    SAFE_DISTANCE_K_VEL: float = 0.6              # quanto si allarga quel margine con la velocità [s]
    AVOID_LOOKAHEAD: float = 1.5                  # quanto avanti nel tempo guarda l'evitamento [s]
    K_AVOID_REPULSION: float = 7.2
    K_AVOID_DAMPING: float = 2.2
    AVOID_MAX_CORRECTION: float = 3.0             # correzione massima di velocità dovuta all'evitamento [m/s]
    EMERGENCY_AVOID_GAIN: float = 6.0
    EMERGENCY_DAMPING: float = 0.8
    EMERGENCY_TARGET_WEIGHT: float = 0.25         # quanto resta del compito mentre ci si scansa

    # --- Conseguenze di un urto --------------------------------------------
    # Un urto non è più solo un numero in una statistica: i droni si rompono.
    COLLISION_DAMAGE: bool = True                 # False = gli urti si contano ma non rompono niente
    # Velocità RELATIVA d'impatto oltre la quale si rompe anche la radio. Sotto, si rompono solo
    # i motori: il drone precipita dove si trovava ma continua a trasmettere quello che sa.
    COLLISION_TOTAL_LOSS_SPEED: float = 0.6
    # Un relitto caduto dentro un incendio brucia: dopo questi secondi tace anche lui.
    WRECK_BURN_TIME_S: float = 20.0
    # Un drone precipitato resta un ostacolo per chi vola ancora?
    # False (default): no. I droni volano a quota di crociera e un rottame sta a terra, quindi ci
    #   passano sopra. È anche la situazione realistica: riconoscere di essere a terra è banale per
    #   un multirotore (assetto ribaltato, accelerometro fermo a 1 g, giri dei motori a zero), e
    #   dichiararlo agli altri costa un bit nel messaggio che già trasmette.
    # True: sì, come se fosse rimasto sospeso dove si è rotto. Serve per misurare quanto costa allo
    #   sciame un ostacolo fisso e inutile: vedi la variante "relitti ingombranti" in experiments.py.
    WRECK_BLOCKS_FLIGHT: bool = False
    # Quota dei guasti in cui l'avaria NON viene dichiarata: il drone precipita ma la sua radio
    # continua a ripetere l'ultimo stato, "sto volando" compreso. Con 0 non succede mai.
    # È il guasto bizantino del povero: non mente di proposito, semplicemente si è fermato con
    # l'ultima verità in bocca. Vedi l'esperimento "guasti".
    SILENT_FAILURE_PROBABILITY: float = 0.0
    # Guasti provocati a tavolino, per studiare la resilienza senza aspettare che accada un urto:
    # a FAILURE_INJECTION_TIME_S si rompono FAILURE_INJECTION_COUNT droni scelti a caso.
    FAILURE_INJECTION_COUNT: int = 0
    FAILURE_INJECTION_TIME_S: float = 30.0
    # Come si rompono i droni scelti: False = si guastano ma la radio resta viva (e allora
    # SILENT_FAILURE_PROBABILITY decide se dichiarano l'avaria); True = perdita totale, radio spenta.
    FAILURE_INJECTION_RADIO_OFF: bool = False

    # --- Perlustrazione ----------------------------------------------------
    # "random": quando non ha compiti, il drone sceglie un punto a caso nell'area.
    # "coverage": sceglie dove non si guarda da più tempo, pesando quanto vale quella zona.
    EXPLORATION_MODE: str = "random"
    COVERAGE_CELL_SIZE: float = 0.5               # lato della cella della griglia [m]
    # Oltre questa anzianità una zona non "peggiora" più: evita che una zona dimenticata da dieci
    # minuti valga dieci volte una dimenticata da un minuto, e che i droni la raggiungano in massa.
    COVERAGE_STALENESS_CAP_S: float = 120.0
    COVERAGE_HALF_LIFE_S: float = 30.0            # usato solo dalla visualizzazione della freschezza
    COVERAGE_REPLAN_S: float = 1.0                # ogni quanto si ricalcola la meta
    COVERAGE_SYNC_PERIOD_S: float = 0.5           # ogni quanto la mappa di copertura viaggia via radio
    COVERAGE_W_DIST: float = 0.05                 # quanto pesa la distanza nella scelta della meta [1/m]
    # Quanto ci si concentra dove il terreno vale di più: valore di una cella = importanza^γ.
    # γ = 0 tutte le zone uguali, γ = 1 proporzionale all'importanza, γ > 1 ancora più concentrati.
    COVERAGE_IMPORTANCE_EXPONENT: float = 1.0

    # --- Terreno -----------------------------------------------------------
    # Ogni hotspot è (x in frazione della larghezza, y in frazione dell'altezza, raggio [m], valore).
    IMPORTANCE_HOTSPOTS: Tuple[Tuple[float, float, float, float], ...] = (
        (0.25, 0.72, 2.5, 1.00),
        (0.72, 0.30, 2.0, 0.90),
        (0.88, 0.80, 1.6, 0.75),
    )
    IMPORTANCE_FLOOR: float = 0.05                # valore minimo ovunque: nessuna zona vale zero

    # --- Interruttori per gli esperimenti ----------------------------------
    # "full": evitamento predittivo + emergenza; "emergency-only": solo la reazione ravvicinata;
    # "none": nessun evitamento (serve come limite inferiore di confronto).
    AVOIDANCE_MODE: str = "full"
    SATURATION_BOUNCE: bool = True                # allontanarsi dagli incendi già presidiati

    # --- Valori calcolati dai precedenti (non si impostano) ----------------

    @property
    def LOW_WATER_THRESHOLD(self) -> float:
        return self.DRONE_WATER_CAPACITY * self.LOW_WATER_FRACTION

    @property
    def FIRE_MEMORY_TTL_STEPS(self) -> int:
        return int(round(self.FIRE_MEMORY_TTL_S / self.SIM_TIME_STEP))

    @property
    def EXTINGUISHED_FIRE_MEMORY_TTL_STEPS(self) -> int:
        # Più lunga della memoria degli incendi attivi: altrimenti una vecchia notizia "c'è un
        # incendio" potrebbe sopravvivere alla smentita "è spento" e farlo resuscitare.
        return self.FIRE_MEMORY_TTL_STEPS * 3

    @property
    def FIRE_SATURATION_MEMORY_STEPS(self) -> int:
        return int(round(self.FIRE_SATURATION_MEMORY_S / self.SIM_TIME_STEP))

    @property
    def COVERAGE_HALF_LIFE_STEPS(self) -> float:
        return self.COVERAGE_HALF_LIFE_S / self.SIM_TIME_STEP

    @property
    def COVERAGE_REPLAN_STEPS(self) -> int:
        return max(1, int(round(self.COVERAGE_REPLAN_S / self.SIM_TIME_STEP)))

    @property
    def COVERAGE_SYNC_STEPS(self) -> int:
        return max(1, int(round(self.COVERAGE_SYNC_PERIOD_S / self.SIM_TIME_STEP)))

    @property
    def REFUEL_GIVE_UP_STEPS(self) -> int:
        return int(round(self.REFUEL_GIVE_UP_S / self.SIM_TIME_STEP))

    @property
    def WRECK_BURN_STEPS(self) -> int:
        return int(round(self.WRECK_BURN_TIME_S / self.SIM_TIME_STEP))

    # --- Utilità -----------------------------------------------------------

    def with_overrides(self, **changes: Any) -> "SimConfig":
        """Copia di questa configurazione con alcuni parametri cambiati."""
        return dataclasses.replace(self, **changes)

    def changes_from_default(self) -> Dict[str, Any]:
        """Solo i parametri diversi dai valori di default: serve a descrivere uno scenario."""
        return {field_.name: getattr(self, field_.name) for field_ in dataclasses.fields(self)
                if getattr(self, field_.name) != getattr(DEFAULT_CONFIG, field_.name)}


DEFAULT_CONFIG = SimConfig()


def validate_config(config: SimConfig = DEFAULT_CONFIG) -> List[str]:
    """Controlla le ipotesi geometriche su cui si regge la logica dei droni.

    Molti meccanismi funzionano solo se certe distanze stanno in un certo ordine: per esempio, due
    droni che spengono lo stesso incendio devono potersi sentire via radio, altrimenti non possono
    accorgersi di essere già in troppi. Questa funzione rende quei vincoli espliciti e controllabili
    invece di lasciarli impliciti nel codice. Restituisce la lista dei problemi trovati (vuota = tutto a posto).
    """
    problems = []

    if not config.DRONE_IMPACT_RADIUS < config.EMERGENCY_AVOID_DISTANCE < config.AVOID_MIN_DISTANCE < config.COMMUNICATION_RADIUS:
        problems.append("le distanze devono crescere: urto < emergenza < distanza operativa < raggio radio")

    if 2.0 * config.FIRE_EXTINGUISH_RADIUS > config.COMMUNICATION_RADIUS:
        problems.append("due droni sullo stesso incendio non riuscirebbero a sentirsi: "
                        "2 × FIRE_EXTINGUISH_RADIUS deve stare dentro COMMUNICATION_RADIUS")

    if config.FIRE_WORK_RADIUS + config.TARGET_REACHED_DISTANCE > config.FIRE_EXTINGUISH_RADIUS:
        problems.append("un drone fermo sull'anello di lavoro potrebbe trovarsi troppo lontano per spegnere")

    if config.MAX_DRONES_ON_FIRE > 1:
        spacing = 2.0 * config.FIRE_WORK_RADIUS * math.sin(math.pi / config.MAX_DRONES_ON_FIRE)
        if spacing < config.AVOID_MIN_DISTANCE:
            problems.append("sull'anello di lavoro non c'è posto per MAX_DRONES_ON_FIRE droni distanziati")

    if config.WATER_STATION_CAPACITY > 1:
        spacing = 2.0 * config.WATER_STATION_SLOT_RADIUS * math.sin(math.pi / config.WATER_STATION_CAPACITY)
        if spacing < config.AVOID_MIN_DISTANCE:
            problems.append("i posti di rifornimento sono troppo vicini tra loro")

    if config.WATER_STATION_SLOT_RADIUS + config.TARGET_REACHED_DISTANCE > config.WATER_STATION_SERVICE_RADIUS:
        problems.append("un drone arrivato al proprio posto potrebbe risultare fuori dall'area di servizio")

    if config.WATER_STATION_WAIT_RADIUS + config.WATER_STATION_SLOT_RADIUS > config.COMMUNICATION_RADIUS:
        problems.append("chi aspetta il proprio turno non sentirebbe i droni che stanno caricando acqua")

    if config.WATER_STATION_WAIT_RADIUS - config.WATER_STATION_SLOT_RADIUS < config.AVOID_MIN_DISTANCE:
        problems.append("l'anello di attesa è così vicino ai posti da spingere via chi sta caricando")

    if config.EXTINGUISHED_FIRE_MEMORY_TTL_STEPS <= config.FIRE_MEMORY_TTL_STEPS:
        problems.append("la memoria degli incendi spenti deve durare più di quella degli incendi attivi")

    if config.COVERAGE_CELL_SIZE > config.FIRE_DETECTION_RADIUS / 2.0:
        problems.append("le celle della mappa di copertura sono troppo grandi rispetto a quanto vede un drone")

    if config.AVOIDANCE_MODE not in AVOIDANCE_MODES:
        problems.append(f"AVOIDANCE_MODE deve essere uno di {AVOIDANCE_MODES}")
    if config.EXPLORATION_MODE not in EXPLORATION_MODES:
        problems.append(f"EXPLORATION_MODE deve essere uno di {EXPLORATION_MODES}")
    if not 0.0 <= config.PACKET_LOSS <= 1.0:
        problems.append("PACKET_LOSS è una probabilità: deve stare tra 0 e 1")
    if not 0.0 <= config.IMPORTANCE_FLOOR <= 1.0:
        problems.append("IMPORTANCE_FLOOR deve stare tra 0 e 1")
    if config.COVERAGE_IMPORTANCE_EXPONENT < 0.0:
        problems.append("COVERAGE_IMPORTANCE_EXPONENT non può essere negativo")
    if config.IGNITION_RATE_PER_S < 0.0:
        problems.append("IGNITION_RATE_PER_S non può essere negativo")
    if not 0.0 <= config.SILENT_FAILURE_PROBABILITY <= 1.0:
        problems.append("SILENT_FAILURE_PROBABILITY è una probabilità: deve stare tra 0 e 1")
    return problems


# ============================================================
# 2. VETTORI
# ============================================================

def clamp_magnitude(vector: np.ndarray, limit: float) -> np.ndarray:
    """Lo stesso vettore, accorciato se supera la lunghezza massima (la direzione non cambia)."""
    length = magnitude(vector)
    if length <= 1e-12:
        return vector.copy()
    if length > limit:
        return vector * (limit / length)
    return vector.copy()


def magnitude(vector: np.ndarray) -> float:
    """Lunghezza di un vettore a due componenti.

    Calcola esattamente ciò che calcola np.linalg.norm, ma cinque volte più in fretta: su vettori
    così corti il tempo se ne va tutto nella macchinosità di numpy, non nel calcolo. Il risultato è
    identico bit per bit, perché le operazioni sono le stesse e nello stesso ordine — se ne accorge
    subito il test di regressione, che confronta le traiettorie con un'impronta digitale.

    Vale la pena perché è la funzione più chiamata del progetto: quasi un milione di volte ogni
    trenta secondi simulati, un terzo del tempo di calcolo totale.
    """
    x = vector[0]
    y = vector[1]
    return math.sqrt(x * x + y * y)


def normalize(vector: np.ndarray) -> np.ndarray:
    """Vettore di lunghezza 1 nella stessa direzione (vettore nullo se l'originale è nullo)."""
    length = magnitude(vector)
    if length <= 1e-12:
        return np.zeros_like(vector)
    return vector / length


def unit_from_angle(angle: float) -> np.ndarray:
    """Vettore di lunghezza 1 che punta nella direzione dell'angolo dato (radianti)."""
    return np.array([math.cos(angle), math.sin(angle)], dtype=float)


def vec_to_tuple(vector: np.ndarray) -> FirePos:
    """Da array numpy a tupla: serve per usare una posizione come chiave di un dizionario."""
    return (float(vector[0]), float(vector[1]))


# ============================================================
# 3. INCENDIO
# ============================================================

_fire_numbering = itertools.count()


@dataclass(eq=False)
class Fire:
    """Un incendio: sta fermo, cresce da solo, si spegne solo se riceve abbastanza acqua."""
    pos: np.ndarray
    health: float          # quanta acqua serve ancora per spegnerlo
    growth_rate: float     # quanta vita guadagna al secondo
    # Numero progressivo, per chi tiene la storia di ogni incendio. Non si usa id() perché Python
    # ricicla gli indirizzi: un incendio nuovo erediterebbe la storia di uno appena spento.
    uid: int = field(default_factory=lambda: next(_fire_numbering))

    @property
    def active(self) -> bool:
        return self.health > 0.0

    def grow(self, dt: float) -> None:
        if self.active:
            self.health += self.growth_rate * dt

    def extinguish(self, water: float) -> float:
        """Applica l'acqua e restituisce quanta ne è servita davvero (l'eccesso resta nel serbatoio)."""
        applied = min(max(water, 0.0), self.health)
        self.health -= applied
        return applied


# ============================================================
# 4. TERRENO: la mappa di importanza
# ============================================================

class ImportanceMap:
    """Quanto vale tenere d'occhio ogni punto dell'area, da 0 a 1.

    È fatta di "hotspot" gaussiani su un fondo minimo: rappresenta il valore del terreno (case,
    bosco fitto, quello che si vuole) ed è nota a priori a tutti i droni, come le stazioni idriche.
    Non è stato dinamico: non cambia mai durante la simulazione.

    Serve a due cose:
        - ai droni, per decidere dove perlustrare (vedi drone.py);
        - al mondo, per estrarre dove scoppiano le accensioni spontanee.

    L'area è divisa in celle quadrate di lato COVERAGE_CELL_SIZE; `grid[i, j]` è il valore della
    cella di centro `(X[i, j], Y[i, j])`.
    """

    def __init__(self, config: SimConfig):
        self.config = config
        self.columns = max(1, int(round(config.AREA_WIDTH / config.COVERAGE_CELL_SIZE)))
        self.rows = max(1, int(round(config.AREA_HEIGHT / config.COVERAGE_CELL_SIZE)))
        self.cell_width = config.AREA_WIDTH / self.columns
        self.cell_height = config.AREA_HEIGHT / self.rows

        x_centers = (np.arange(self.columns) + 0.5) * self.cell_width
        y_centers = (np.arange(self.rows) + 0.5) * self.cell_height
        self.X, self.Y = np.meshgrid(x_centers, y_centers, indexing="ij")

        importance = np.full(self.shape, config.IMPORTANCE_FLOOR, dtype=float)
        for x_fraction, y_fraction, radius, peak_value in config.IMPORTANCE_HOTSPOTS:
            center_x = x_fraction * config.AREA_WIDTH
            center_y = y_fraction * config.AREA_HEIGHT
            distance_sq = (self.X - center_x) ** 2 + (self.Y - center_y) ** 2
            hotspot = peak_value * np.exp(-distance_sq / (2.0 * radius ** 2))
            importance = np.maximum(importance, hotspot)
        self.grid = np.clip(importance, 0.0, 1.0)

        self._ignition_cdf = self._build_ignition_cdf()

    @property
    def shape(self) -> Tuple[int, int]:
        return (self.columns, self.rows)

    def value_at(self, x: float, y: float) -> float:
        """Il valore del terreno nel punto indicato."""
        column = int(np.clip(x / self.cell_width, 0, self.columns - 1))
        row = int(np.clip(y / self.cell_height, 0, self.rows - 1))
        return float(self.grid[column, row])

    def weight(self, exponent: float) -> np.ndarray:
        """La griglia elevata a `exponent`: con 0 tutte le celle valgono uguale, con 1 vale l'importanza."""
        if exponent == 0.0:
            return np.ones_like(self.grid)
        if exponent == 1.0:
            return self.grid
        return self.grid ** exponent

    def cells_within(self, position: np.ndarray, radius: float) -> Optional[Tuple[slice, slice, np.ndarray]]:
        """Le celle che stanno entro `radius` dal punto, come (colonne, righe, maschera).

        Restituisce un rettangolo di celle più una maschera booleana che dice quali, dentro quel
        rettangolo, sono davvero nel cerchio. Si lavora solo sul rettangolo che contiene il cerchio,
        quindi il costo dipende da quanto vede il sensore, non da quanto è grande la mappa.
        """
        first_column = max(0, int(math.floor((position[0] - radius) / self.cell_width)))
        last_column = min(self.columns, int(math.ceil((position[0] + radius) / self.cell_width)) + 1)
        first_row = max(0, int(math.floor((position[1] - radius) / self.cell_height)))
        last_row = min(self.rows, int(math.ceil((position[1] + radius) / self.cell_height)) + 1)
        if first_column >= last_column or first_row >= last_row:
            return None
        columns = slice(first_column, last_column)
        rows = slice(first_row, last_row)
        dx = self.X[columns, rows] - position[0]
        dy = self.Y[columns, rows] - position[1]
        return columns, rows, (dx * dx + dy * dy) <= radius * radius

    def _build_ignition_cdf(self) -> np.ndarray:
        """Prepara la tabella che serve a estrarre a sorte dove scoppia un incendio.

        È la somma cumulativa dei "pesi" di tutte le celle: estraendo un numero a caso tra 0 e 1 e
        cercando dove cade nella tabella si ottiene una cella con probabilità proporzionale al suo peso.
        """
        margin = self.config.FIRE_GENERATION_MARGIN
        far_enough_from_the_edge = ((self.X >= margin) & (self.X <= self.config.AREA_WIDTH - margin)
                                    & (self.Y >= margin) & (self.Y <= self.config.AREA_HEIGHT - margin))
        weights = self.grid if self.config.IGNITION_FOLLOWS_IMPORTANCE else np.ones_like(self.grid)
        weights = np.where(far_enough_from_the_edge, weights, 0.0).ravel()
        if weights.sum() <= 0.0:                    # margine così grande da escludere tutta l'area
            weights = np.ones_like(weights)
        return np.cumsum(weights) / weights.sum()

    def sample_position(self, rng: random.Random) -> np.ndarray:
        """Un punto a caso, più probabile dove il terreno vale di più."""
        cell_index = int(np.searchsorted(self._ignition_cdf, rng.random(), side="right"))
        cell_index = min(cell_index, self._ignition_cdf.size - 1)
        column, row = divmod(cell_index, self.rows)
        return np.array([self.X[column, row] + (rng.random() - 0.5) * self.cell_width,
                         self.Y[column, row] + (rng.random() - 0.5) * self.cell_height], dtype=float)


# ============================================================
# 5. IL CLOCK
# ============================================================

class Clock:
    """Il battito della simulazione: l'unica cosa che i droni hanno in comune oltre alla radio.

    Nessuno comanda i droni. Ciascuno, quando nasce, si iscrive al clock lasciandogli due cose da
    richiamare a ogni battito, e da quel momento nel resto del programma non esiste più un modo per
    farlo agire dall'esterno: i suoi metodi sono privati, e l'unico che li conosce è il clock.

    Ogni battito ha tre fasi, e l'ordine è la regola più importante di tutta la simulazione:

        fase 1 — TUTTI trasmettono ciò che sanno
        fase 2 — TUTTI leggono la posta e decidono dove andare
        fase 3 — TUTTI agiscono sul mondo (spruzzano acqua, caricano)

    Se le fasi fossero mescolate, il drone che ragiona per primo deciderebbe conoscendo già le mosse
    appena fatte dagli altri, e chi ragiona per ultimo sarebbe sistematicamente avvantaggiato.
    Separare la fase 3 serve allo stesso scopo verso il mondo: nessuno decide guardando un incendio
    che un altro ha appena spento nello stesso battito.
    """

    def __init__(self, time_step: float):
        self.time_step = time_step
        self.step_count = 0
        self._broadcast_phase: List[Tuple[int, Any]] = []   # (indice, funzione da chiamare)
        self._thinking_phase: List[Tuple[int, Any]] = []
        self._acting_phase: List[Tuple[int, Any]] = []

    @property
    def now_s(self) -> float:
        """Secondi trascorsi dall'inizio della simulazione."""
        return self.step_count * self.time_step

    def subscribe(self, subscriber_id: int, broadcast, think, act) -> None:
        """Un agente chiede di essere svegliato a ogni battito, in tutte e tre le fasi."""
        self._broadcast_phase.append((subscriber_id, broadcast))
        self._thinking_phase.append((subscriber_id, think))
        self._acting_phase.append((subscriber_id, act))

    def unsubscribe(self, subscriber_id: int) -> None:
        """Un agente smette di rispondere al clock: per un drone vuol dire essere distrutto."""
        self._broadcast_phase = [entry for entry in self._broadcast_phase if entry[0] != subscriber_id]
        self._thinking_phase = [entry for entry in self._thinking_phase if entry[0] != subscriber_id]
        self._acting_phase = [entry for entry in self._acting_phase if entry[0] != subscriber_id]

    def tick(self) -> None:
        """Un battito: prima parlano tutti, poi ragionano tutti, infine agiscono tutti."""
        for _, broadcast in self._broadcast_phase:
            broadcast()
        for _, think in self._thinking_phase:
            think()
        for _, act in self._acting_phase:
            act()
        self.step_count += 1


# ============================================================
# 6. RADIO
# ============================================================

@dataclass
class DroneMessage:
    """Quello che un drone dice di sé agli altri, una volta per passo di simulazione.

    È l'UNICO modo in cui un drone sa qualcosa degli altri: nel codice non esiste un punto in cui
    un drone legga direttamente lo stato di un altro oggetto Drone.
    """
    position: np.ndarray
    velocity: np.ndarray
    target: np.ndarray                          # dove sta andando
    reloading: bool                             # sta andando a fare acqua
    water_station_idx: Optional[int]            # a quale stazione
    refuel_claim_age: int                       # da quanti passi aspetta il proprio turno
    station_slot: Optional[int]                 # quale posto di rifornimento occupa
    flying: bool                                # è ancora in volo (un relitto trasmette ma sta a terra)
    extinguishing: bool                         # sta spruzzando acqua
    fire_target: Optional[FirePos]              # di quale incendio si sta occupando
    known_fires: Dict[FirePos, int]             # incendi che crede accesi -> quanto è vecchia la notizia
    extinguished_fires: Dict[FirePos, int]      # incendi che sa spenti -> quanto è vecchia la notizia
    # La mappa di dove si è già guardato: è il dato più pesante, quindi viaggia di rado (None = "non stavolta").
    coverage: Optional[np.ndarray] = None


class Mailbox:
    """La cassetta postale di un drone: i messaggi ricevuti dai vicini, con doppio scomparto.

    Perché due scomparti. In un passo di simulazione prima TUTTI i droni trasmettono, poi TUTTI
    ragionano. I messaggi che arrivano finiscono nello scomparto "in arrivo" e diventano leggibili
    solo al passo successivo, quando `begin_round()` li sposta in "vicini". Così il drone 0, che
    ragiona per primo, e il drone 11, che ragiona per ultimo, usano informazioni dello stesso istante:
    nessuno ha il vantaggio di sapere cosa hanno appena deciso gli altri.
    """

    def __init__(self, drone: 'Drone'):
        self.drone = drone
        self.neighbors: Dict[int, DroneMessage] = {}    # leggibili adesso
        self._incoming: Dict[int, DroneMessage] = {}    # arrivati adesso, leggibili al prossimo passo

    def begin_round(self) -> None:
        self.neighbors = self._incoming
        self._incoming = {}

    def deliver(self, sender_idx: int, message: DroneMessage) -> None:
        self._incoming[sender_idx] = message

    def merge_fire_knowledge(self) -> None:
        """Unisce ciò che dicono i vicini alla propria memoria degli incendi.

        Ogni notizia porta con sé la propria età (in passi di simulazione): tra due versioni della
        stessa notizia vince la più fresca. È il meccanismo che fa arrivare un avvistamento anche a
        droni lontanissimi, passando di vicino in vicino.
        """
        drone = self.drone

        # 1. Prima le smentite: "quell'incendio è spento" vince sempre su "c'è un incendio".
        for message in self.neighbors.values():
            for fire_pos, age in message.extinguished_fires.items():
                if fire_pos not in drone.extinguished_fires or age < drone.extinguished_fires[fire_pos]:
                    drone.extinguished_fires[fire_pos] = age

        # 2. Poi gli avvistamenti, ignorando quelli già smentiti.
        for message in self.neighbors.values():
            for fire_pos, age in message.known_fires.items():
                if fire_pos in drone.extinguished_fires:
                    continue
                if fire_pos not in drone.known_fires or age < drone.known_fires[fire_pos]:
                    drone.known_fires[fire_pos] = age

        # 3. Infine si ripulisce la memoria da ciò che nel frattempo si è saputo spento.
        for fire_pos in list(drone.known_fires):
            if fire_pos in drone.extinguished_fires:
                del drone.known_fires[fire_pos]


class RadioChannel:
    """Chi sente chi, e quali messaggi si perdono per strada.

    Due droni si sentono se sono entro COMMUNICATION_RADIUS. Ogni messaggio può perdersi con
    probabilità PACKET_LOSS, indipendentemente per ogni coppia e per ogni passo. Un drone
    distrutto non trasmette e non riceve più nulla.
    """

    def __init__(self, config: SimConfig, seed: int):
        self.radius = config.COMMUNICATION_RADIUS
        self.loss_probability = config.PACKET_LOSS
        # Generatore separato da quello degli incendi e da quelli dei droni: così attivare le
        # perdite non sposta la sequenza casuale di tutto il resto, e gli scenari restano confrontabili.
        self.rng = random.Random(f"channel-{seed}")
        self.neighbor_map: Dict[int, List['Drone']] = {}
        self.messages_attempted = 0     # solo statistica, i droni non la vedono
        self.messages_delivered = 0

    @property
    def messages_dropped(self) -> int:
        return self.messages_attempted - self.messages_delivered

    def refresh_neighbors(self, drones: List['Drone']) -> Dict[int, List['Drone']]:
        """Ricalcola chi è nel raggio di chi. Da rifare a ogni passo, perché i droni si muovono."""
        neighbor_map: Dict[int, List['Drone']] = {drone.idx: [] for drone in drones}
        with_working_radio = [drone for drone in drones if drone.status is not DroneStatus.DESTROYED]
        if len(with_working_radio) > 1:
            positions = np.array([drone.position for drone in with_working_radio])
            difference = positions[:, None, :] - positions[None, :, :]
            distance = np.sqrt(np.einsum("ijk,ijk->ij", difference, difference))
            in_range = distance <= self.radius
            np.fill_diagonal(in_range, False)
            for i, drone in enumerate(with_working_radio):
                neighbor_map[drone.idx] = [with_working_radio[j] for j in np.flatnonzero(in_range[i])]
        self.neighbor_map = neighbor_map
        return neighbor_map

    def broadcast(self, sender_idx: int, message: DroneMessage) -> None:
        """Consegna il messaggio a tutti quelli nel raggio del mittente, tranne quelli che si perdono."""
        receivers = self.neighbor_map.get(sender_idx, [])
        self.messages_attempted += len(receivers)
        for receiver in receivers:
            if self.loss_probability > 0.0 and self.rng.random() < self.loss_probability:
                continue
            receiver.mailbox.deliver(sender_idx, message)
            self.messages_delivered += 1


# ============================================================
# 7. MONDO
# ============================================================

class SimulationWorld:
    """L'ambiente: tiene gli incendi e offre ai droni sensori, attuatori e radio.

    Tutto ciò che un drone può sapere o fare passa da uno di questi metodi. Sono pochi di proposito:
        sense_fires()        "quali incendi vedo da qui?"
        has_active_fire_near() "c'è fuoco entro questa distanza?"
        request_extinguish() "spruzzo acqua: cosa succede?"
        broadcast()          "dico agli altri come sto"
    """

    def __init__(self, config: SimConfig, drones: List['Drone'], fires: List[Fire],
                 water_stations: List[np.ndarray], terrain: ImportanceMap, seed: int):
        self.config = config
        self._drones = drones
        self._fires = fires
        self.area_width = config.AREA_WIDTH
        self.area_height = config.AREA_HEIGHT
        self.water_stations = water_stations     # infrastruttura fissa, nota a tutti
        self.terrain = terrain
        self.channel = RadioChannel(config, seed)
        # Il tempo comune a tutti: i droni vi si iscrivono da soli e nessun altro li fa agire.
        self.clock = Clock(config.SIM_TIME_STEP)
        # Generatore dedicato alle accensioni spontanee: con IGNITION_RATE_PER_S = 0 non estrae
        # nemmeno un numero, quindi accenderle non cambia il resto dello scenario.
        self.ignition_rng = random.Random(f"ignition-{seed}")

        self.extinguished_count = 0    # incendi spenti dai droni
        self.spawned_count = 0         # incendi nati per propagazione
        self.ignited_count = 0         # incendi nati da soli
        self.water_delivered = 0.0     # acqua finita davvero sugli incendi

    @property
    def fires(self) -> List[Fire]:
        return self._fires

    @property
    def step_counter(self) -> int:
        """Quanti battiti sono passati: il tempo è quello del clock, non un contatore a parte."""
        return self.clock.step_count

    # --- Radio -------------------------------------------------------------

    def refresh_neighbors(self) -> Dict[int, List['Drone']]:
        return self.channel.refresh_neighbors(self._drones)

    def broadcast(self, sender_idx: int, message: DroneMessage) -> None:
        self.channel.broadcast(sender_idx, message)

    # --- Sensori -----------------------------------------------------------

    def sense_fires(self, position: np.ndarray) -> List[Fire]:
        """Gli incendi accesi che un drone in questa posizione riesce a vedere."""
        radius = self.config.FIRE_DETECTION_RADIUS
        return [fire for fire in self._fires
                if fire.active and magnitude(position - fire.pos) <= radius]

    def has_active_fire_near(self, position: np.ndarray, radius: float) -> bool:
        return any(fire.active and magnitude(position - fire.pos) <= radius for fire in self._fires)

    # --- Attuatore ---------------------------------------------------------

    def request_extinguish(self, position: np.ndarray, water_available: float,
                           flow_rate: float) -> Tuple[float, List[FirePos]]:
        """Spruzza acqua sugli incendi a portata.

        Restituisce quanta acqua è servita e quali incendi si sono spenti con questo getto: così il
        drone viene a sapere l'esito della propria azione senza guardare lo stato globale del mondo.
        """
        remaining = min(max(water_available, 0.0), flow_rate * self.config.SIM_TIME_STEP)
        total_used = 0.0
        extinguished: List[FirePos] = []
        for fire in self._fires:
            if remaining <= 1e-12:
                break
            if not fire.active or magnitude(position - fire.pos) > self.config.FIRE_EXTINGUISH_RADIUS:
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

    # --- Evoluzione degli incendi ------------------------------------------

    def update_fires(self, rng: random.Random) -> None:
        """Fa crescere gli incendi, li fa propagare e ne accende di nuovi. Una volta per passo."""
        self._grow_and_spread(rng)
        self._maybe_ignite_new_fire()
        self._remove_dead_fires()

    def _grow_and_spread(self, rng: random.Random) -> None:
        config = self.config
        newborn: List[Fire] = []
        for fire in self._fires:
            fire.grow(config.SIM_TIME_STEP)
            too_big = fire.health > config.FIRE_HEALTH * config.FIRE_SPAWN_THRESHOLD
            if too_big and rng.random() < config.FIRE_SPAWN_PROB_PER_STEP:
                angle = rng.uniform(0.0, 2.0 * math.pi)
                distance = rng.uniform(0.5, config.FIRE_SPAWN_OFFSET_MAX)
                newborn.append(self._new_fire(fire.pos + distance * unit_from_angle(angle)))
        self._fires.extend(newborn)
        self.spawned_count += len(newborn)

    def _maybe_ignite_new_fire(self) -> None:
        """Accensione spontanea: un incendio che nasce dove nessuno lo sta guardando."""
        rate = self.config.IGNITION_RATE_PER_S
        if rate <= 0.0 or self.ignition_rng.random() >= rate * self.config.SIM_TIME_STEP:
            return
        self._fires.append(self._new_fire(self.terrain.sample_position(self.ignition_rng)))
        self.ignited_count += 1

    def _new_fire(self, position: np.ndarray) -> Fire:
        """Un incendio appena nato, riportato dentro i bordi dell'area."""
        margin = self.config.FIRE_GENERATION_MARGIN
        position = position.copy()
        position[0] = float(np.clip(position[0], margin, self.area_width - margin))
        position[1] = float(np.clip(position[1], margin, self.area_height - margin))
        return Fire(pos=position,
                    health=self.config.FIRE_HEALTH * self.config.FIRE_SPAWN_INITIAL_HEALTH,
                    growth_rate=self.config.FIRE_GROWTH_RATE)

    def _remove_dead_fires(self) -> None:
        self._fires[:] = [fire for fire in self._fires if fire.active]
