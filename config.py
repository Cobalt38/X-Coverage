"""
Configurazione della simulazione.

Tutti i parametri stanno in un unico oggetto immutabile (SimConfig) passato esplicitamente a mondo,
droni e simulazione. Niente variabili globali mutabili: così
    - due simulazioni con parametri diversi possono convivere nello stesso processo;
    - gli esperimenti in parallelo (multiprocessing, che su Windows reimporta i moduli da zero)
      ricevono esattamente la configurazione della variante che devono simulare.

I nomi restano in MAIUSCOLO, come le costanti del vecchio script.
Per cambiare un valore: SimConfig(NUM_DRONES=20) oppure cfg.with_overrides(NUM_DRONES=20).
"""

import dataclasses
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

AVOIDANCE_MODES = ("full", "emergency-only", "none")

GOLDEN_ANGLE = 2.399963  # rad, distribuisce direzioni di fallback senza sovrapposizioni

FirePos = Tuple[float, float]


@dataclass(frozen=True)
class SimConfig:
    # SIM
    NUM_DRONES: int = 12
    AREA_WIDTH: float = 20.0
    AREA_HEIGHT: float = 12.0
    SIM_TIME_STEP: float = 0.01
    RANDOM_SEED: int = 42

    FIRE_GROWTH_RATE: float = 0.5  # Health points per second

    # DRONI
    COMMUNICATION_RADIUS: float = 2.5
    TARGET_SEPARATION: float = 2.0
    TARGET_REACHED_DISTANCE: float = 0.2
    MAX_TARGET_SPEED: float = 20.0
    MAX_FORCE_ON_TARGET: float = 10.0
    # Smorzamento esponenziale della velocità del target [1/s]: decay = exp(-rate * dt).
    # exp(-10.5 * 0.01) ≈ 0.90 -> identico al vecchio "TARGET_VEL_AGING_FACTOR = 0.1 per step",
    # ma ora il valore ha un significato temporale e non dipende da SIM_TIME_STEP.
    TARGET_VEL_DAMPING_RATE: float = 10.5
    MAX_DRONE_SPEED: float = 1.0
    ANCHOR_TO_TARGET_INTENSITY: float = 0.0075  # [1/s] L'ancora segue lentamente il target (costante di tempo ≈ 133 s)

    K_DESIRED_VEL_TO_TARGET: float = 0.70
    K_REPULSION_BETWEEN_TARGETS: float = 1.0
    K_ANCHOR_DRAGGING: float = 0.2
    K_BOUNDARY_REPULSION: float = 1.0

    FIRE_DETECTION_RADIUS: float = 2.0
    FIRE_GENERATION_MARGIN: float = 1.5
    WATER_STATION_GENERATION_MARGIN: float = 2.0  # >= WATER_STATION_WAIT_RADIUS: l'anello di attesa resta nell'area
    MARGIN_REPULSION_BOUNDARY: float = 1.5  # Distanza minima dal bordo per la repulsione del target

    MAX_IDLE_STEPS: int = 30  # Step d'inattività prima di riprendere l'esplorazione

    # Collision avoidance & Safety parameters
    # Gerarchia esplicita delle distanze (controllata da validate_config()):
    #   DRONE_IMPACT_RADIUS < EMERGENCY_AVOID_DISTANCE < AVOID_MIN_DISTANCE < COMMUNICATION_RADIUS
    #   0.1 m collisione  <  0.8 m emergenza  <  1.0 m distanza operativa  <  2.5 m orizzonte di percezione
    DRONE_IMPACT_RADIUS: float = 0.1       # Soglia reale di collisione fisica (solo rilevamento statistico)
    EMERGENCY_AVOID_DISTANCE: float = 0.8  # Soglia locale di emergenza: l'evitamento prevale sul compito
    AVOID_MIN_DISTANCE: float = 1.0        # Distanza di separazione minima desiderata (metri)
    SAFE_DISTANCE_K_VEL: float = 0.6       # [s] Moltiplicatore ellisse di sicurezza su velocità: margine = AVOID_MIN_DISTANCE + K * |v|
    AVOID_LOOKAHEAD: float = 1.5           # Orizzonte temporale predittivo per evitamento (secondi)
    K_AVOID_REPULSION: float = 7.2         # Guadagno repulsivo campo potenziale
    K_AVOID_DAMPING: float = 2.2           # Smorzamento velocità relativa di avvicinamento
    AVOID_MAX_CORRECTION: float = 3.0      # [m/s] Correzione massima di velocità dovuta all'evitamento predittivo
    EMERGENCY_AVOID_GAIN: float = 6.0      # Guadagno repulsivo del livello di emergenza
    EMERGENCY_DAMPING: float = 0.8         # Smorzamento della velocità relativa nel livello di emergenza
    EMERGENCY_TARGET_WEIGHT: float = 0.25  # Quota di navigazione verso il target mantenuta durante l'emergenza
    MAX_JERK: float = 8.0                  # Variazione massima accelerazione (m/s^3)

    # Fire extinguishing & Saturation Behavior
    FIRE_HEALTH: float = 200.0
    FIRE_SPAWN_THRESHOLD: float = 1.20        # Soglia vita per propagazione incendio
    FIRE_SPAWN_PROB_PER_STEP: float = 0.003   # Probabilità per passo di generare un nuovo incendio
    FIRE_SPAWN_OFFSET_MAX: float = 3.0        # Raggio massimo offset incendio figlio
    FIRE_SPAWN_INITIAL_HEALTH: float = 0.15   # Vita iniziale incendio figlio
    DRONE_WATER_CAPACITY: float = 20.0
    DRONE_WATER_FLOW_RATE: float = 5.0
    FIRE_EXTINGUISH_RADIUS: float = 1.2
    FIRE_WORK_RADIUS: float = 0.8             # Distanza dal centro a cui il drone si posiziona per spegnere (anello di lavoro)
    MAX_DRONES_ON_FIRE: int = 3               # Max droni sullo stesso incendio (stima LOCALE di ciascun drone)
    FIRE_SATURATION_BOUNCE_DISTANCE: float = 3.5  # Distanza (m) di allontanamento radiale se il fuoco è saturo (non è una forza)
    FIRE_SATURATION_MEMORY_S: float = 5.0     # Per quanti secondi un incendio saturo viene escluso dalla scelta

    # Stazione idrica / rifornimento
    LOW_WATER_FRACTION: float = 0.1           # Soglia di rifornimento come frazione di DRONE_WATER_CAPACITY
    WATER_STATION_REFILL_RATE: float = 10.0
    NUM_WATER_STATIONS: int = 3
    WATER_STATION_CAPACITY: int = 2           # Droni riforniti contemporaneamente (uno per slot di servizio)
    WATER_STATION_SERVICE_RADIUS: float = 0.8
    WATER_STATION_SLOT_RADIUS: float = 0.55   # Distanza degli slot di servizio dal centro della stazione
    WATER_STATION_WAIT_RADIUS: float = 1.9    # Anello di attesa: abbastanza vicino da "sentire" chi è in servizio
    WATER_STATION_MIN_SEPARATION: float = 5.0  # Separazione minima tra stazioni casuali

    # PID velocity control
    PID_KP: float = 3.4
    PID_KI: float = 0.6
    PID_KD: float = 0.25
    PID_INTEGRAL_LIMIT: float = 3.0
    PID_MAX_OUTPUT_ACCEL: float = 4.0

    # TTL memoria incendi (in secondi, convertiti in step dalle proprietà sotto).
    # Un incendio non si sposta e non scompare da solo: può essere ricordato a lungo. Se nel frattempo
    # viene spento, la notizia arriva via gossip (extinguished_fires) oppure il drone lo verifica
    # arrivando sul posto.
    FIRE_MEMORY_TTL_S: float = 15.0

    # Numero incendi generati con --random-fires
    NUM_FIRES: int = 3

    # --------------------------------------------------------
    # Ablation studies (i default riproducono esattamente il comportamento originale)
    # --------------------------------------------------------
    # "full": CPA predittivo + emergenza; "emergency-only": solo il livello di emergenza
    # (repulsione reattiva sulla distanza attuale); "none": nessun evitamento (limite inferiore).
    AVOIDANCE_MODE: str = "full"
    # Probabilità di perdita di ciascun messaggio su ciascun link (Bernoulli i.i.d., link per link).
    PACKET_LOSS: float = 0.0
    # Rimbalzo radiale dagli incendi saturi (euristica anti-livelock).
    SATURATION_BOUNCE: bool = True

    # --------------------------------------------------------
    # Grandezze derivate (non impostabili: seguono sempre i parametri da cui dipendono)
    # --------------------------------------------------------

    @property
    def LOW_WATER_THRESHOLD(self) -> float:
        return self.DRONE_WATER_CAPACITY * self.LOW_WATER_FRACTION

    @property
    def FIRE_MEMORY_TTL_STEPS(self) -> int:
        return int(round(self.FIRE_MEMORY_TTL_S / self.SIM_TIME_STEP))

    @property
    def EXTINGUISHED_FIRE_MEMORY_TTL_STEPS(self) -> int:
        # Deve essere > FIRE_MEMORY_TTL_STEPS: una vecchia voce "attivo" non deve sopravvivere alla notizia "spento".
        return self.FIRE_MEMORY_TTL_STEPS * 3

    @property
    def FIRE_SATURATION_MEMORY_STEPS(self) -> int:
        return int(round(self.FIRE_SATURATION_MEMORY_S / self.SIM_TIME_STEP))

    # --------------------------------------------------------
    # Utilità
    # --------------------------------------------------------

    def with_overrides(self, **changes: Any) -> "SimConfig":
        """Copia con alcuni parametri cambiati, es. cfg.with_overrides(NUM_DRONES=20)."""
        return dataclasses.replace(self, **changes)

    def changes_from_default(self) -> Dict[str, Any]:
        """Solo i parametri diversi dal default (per descrivere uno scenario nel report)."""
        return {f.name: getattr(self, f.name) for f in dataclasses.fields(self)
                if getattr(self, f.name) != getattr(DEFAULT_CONFIG, f.name)}


DEFAULT_CONFIG = SimConfig()


def validate_config(cfg: SimConfig = DEFAULT_CONFIG) -> List[str]:
    """Controlla che i parametri rispettino le ipotesi geometriche su cui si basa la logica."""
    problems = []
    if not cfg.DRONE_IMPACT_RADIUS < cfg.EMERGENCY_AVOID_DISTANCE < cfg.AVOID_MIN_DISTANCE < cfg.COMMUNICATION_RADIUS:
        problems.append("gerarchia distanze violata: IMPACT < EMERGENCY < AVOID_MIN < COMMUNICATION")
    # La saturazione è stimata solo dai messaggi dei vicini: due droni che spengono lo stesso incendio
    # (entrambi entro FIRE_EXTINGUISH_RADIUS) devono potersi sentire.
    if 2.0 * cfg.FIRE_EXTINGUISH_RADIUS > cfg.COMMUNICATION_RADIUS:
        problems.append("2*FIRE_EXTINGUISH_RADIUS > COMMUNICATION_RADIUS: la saturazione non è stimabile localmente")
    if cfg.FIRE_WORK_RADIUS + cfg.TARGET_REACHED_DISTANCE > cfg.FIRE_EXTINGUISH_RADIUS:
        problems.append("FIRE_WORK_RADIUS troppo grande: il drone sull'anello potrebbe non spegnere")
    # L'anello di lavoro deve ospitare MAX_DRONES_ON_FIRE droni a distanza >= AVOID_MIN_DISTANCE.
    if cfg.MAX_DRONES_ON_FIRE > 1 and 2.0 * cfg.FIRE_WORK_RADIUS * math.sin(math.pi / cfg.MAX_DRONES_ON_FIRE) < cfg.AVOID_MIN_DISTANCE:
        problems.append("anello di lavoro troppo piccolo per MAX_DRONES_ON_FIRE droni separati")
    if cfg.WATER_STATION_CAPACITY > 1 and 2.0 * cfg.WATER_STATION_SLOT_RADIUS * math.sin(math.pi / cfg.WATER_STATION_CAPACITY) < cfg.AVOID_MIN_DISTANCE:
        problems.append("slot della stazione troppo vicini tra loro")
    if cfg.WATER_STATION_SLOT_RADIUS + cfg.TARGET_REACHED_DISTANCE > cfg.WATER_STATION_SERVICE_RADIUS:
        problems.append("uno slot raggiunto potrebbe cadere fuori dall'area di servizio")
    if cfg.WATER_STATION_WAIT_RADIUS + cfg.WATER_STATION_SLOT_RADIUS > cfg.COMMUNICATION_RADIUS:
        problems.append("chi attende non riesce a sentire i droni in servizio")
    if cfg.WATER_STATION_WAIT_RADIUS - cfg.WATER_STATION_SLOT_RADIUS < cfg.AVOID_MIN_DISTANCE:
        problems.append("anello di attesa troppo vicino agli slot di servizio")
    if cfg.EXTINGUISHED_FIRE_MEMORY_TTL_STEPS <= cfg.FIRE_MEMORY_TTL_STEPS:
        problems.append("la memoria 'spento' deve durare più della memoria 'attivo'")
    if cfg.AVOIDANCE_MODE not in AVOIDANCE_MODES:
        problems.append(f"AVOIDANCE_MODE deve essere uno di {AVOIDANCE_MODES}")
    if not 0.0 <= cfg.PACKET_LOSS <= 1.0:
        problems.append("PACKET_LOSS deve essere in [0, 1]")
    return problems
