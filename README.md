x-coverage-readme README file per la simulazione X-Coverage
Instructions
X-Coverage — Simulazione di Sciame per il Monitoraggio e la Soppressione di Incendi

"Un sistema multi-agente decentralizzato basato su campi di forza emergenti e meccanismi di gossip locale per la soppressione coordinata di incendi dinamici."


Indice

Panoramica del sistema
Architettura generale
Ciclo di esecuzione principale
La pipeline del singolo agente
Modello di comunicazione localizzata
Propagazione della conoscenza ed epidemica degli incendi
Dinamica degli incendi e propagazione spontanea
Logistica idrica e gestione delle stazioni
Controllo del moto e dinamica dei campi di forza
Collisioni e sicurezza operativa
Parametri chiave del sistema
Interfaccia utente e rendering
Avvio e opzioni da riga di comando


1. Panoramica del sistema
X-Coverage è un framework di simulazione per sciami di droni autonomi $N$ operanti in un dominio bidimensionale per la ricerca, il contenimento e l'estinzione di eventi d'incendio dinamici.
Il sistema si fonda sui principi dei sistemi complessi decentralizzati: ciascun agente prende decisioni esclusivamente sulla base di percezioni sensoriali locali e dello scambio di messaggi a corto raggio con i vicini entro un raggio di comunicazione definito ($R_{\text{comm}}$). Non esiste un'infrastruttura di controllo centrale, un registro globale condiviso o un coordinatore della flotta.
L'ambiente è caratterizzato da eventi stocastici e dinamici: i focolai aumentano la propria intensità nel tempo e, al superamento di soglie critiche di energia, possono propagarsi generando focolai secondari adiacenti.

2. Architettura generale
graph TD
    subgraph SwarmSimulation["SwarmSimulation (Orchestratore)"]
        STEP["step()"]
        DRAW["draw_scene()"]
        COL["_resolve_collisions()"]
    end

    subgraph SimulationWorld["SimulationWorld (Ambiente)"]
        REFRESH["refresh_neighbors()"]
        FIRES["update_fires(rng)"]
        SENSE["sense_fires()"]
        EXT["request_extinguish()"]
    end

    subgraph Drone["Drone (Agente × N)"]
        PRE_STEP["pre_step()"]
        SENSE_ENV["sense_environment()"]
        MERGE["merge_neighbor_knowledge()"]
        DECIDE["decide_and_move()"]
        EXTINGUISH["try_extinguish()"]
        RELOAD["try_reload()"]
    end

    STEP --> REFRESH
    STEP --> PRE_STEP
    PRE_STEP --> SENSE_ENV
    STEP --> MERGE
    STEP --> DECIDE
    STEP --> EXTINGUISH
    STEP --> RELOAD
    STEP --> FIRES
    STEP --> COL
    SENSE_ENV --> SENSE
    EXTINGUISH --> EXT


3. Ciclo di esecuzione principale
La simulazione avanza a passi temporali discreti $\Delta t = 0.01\text{ s}$ gestiti dall'orchestratore. Per garantire la rigorosa coerenza causale ed evitare disparità di aggiornamento nello stato degli agenti, ogni passo di simulazione si articola in fasi ben distinte:
sequenceDiagram
    participant S as SwarmSimulation
    participant W as SimulationWorld
    participant D as Drone × N

    S->>W: refresh_neighbors()
    Note over W: Calcola le adiacenze topologiche per raggio COMMUNICATION_RADIUS
    loop Fase di Broadcast (Pre-Step)
        S->>D: pre_step()
        Note over D: Genera il pacchetto DroneMessage e lo trasmette al doppio buffer dei vicini
    end
    loop Fase di Esecuzione (Step)
        S->>D: step()
        Note over D: Promuove i messaggi, aggiorna la memoria,<br/>calcola le forze PID e aggiorna la posizione fisica
    end
    S->>W: update_fires(rng)
    S->>S: _resolve_collisions()


4. La pipeline del singolo agente
Ogni unità dello sciame esegue una sequenza deterministica di elaborazione ad ogni ciclo logico:
flowchart LR
    A["sense_environment()"] --> B["merge_neighbor_knowledge()"]
    B --> C["decide_and_move()"]
    C --> D["try_extinguish()"]
    D --> E["try_reload()"]

4.1 Percezione (sense_environment)
Il drone aggiorna lo stato dei propri sensori fisici:

Incrementa il tempo di permanenza (in step) delle informazioni memorizzate in known_fires ed extinguished_fires, applicando un decadimento temporale (TTL).
Verifica la presenza fisica dei fuochi noti entro il raggio di sensore (FIRE_DETECTION_RADIUS). Se un focolaio precedentemente registrato non è più rilevato dall'osservazione diretta, viene classificato come estinto ed inserito nella memoria di soppressione.
Registra i nuovi focolai scoperti nel proprio raggio percettivo.

4.2 Fusione della Conoscenza (merge_neighbor_knowledge)
Il drone consolida le informazioni ricevute dai vicini mediante l'analisi del buffer di comunicazione. Le informazioni di estinzione hanno priorità su quelle di rilevamento attivo per prevenire ri-allocazioni obsolete.
4.3 Decisione e Cinematica (decide_and_move)
L'agente determina il proprio vettore di accelerazione combinando la selezione comportamentale dell'obiettivo con un sistema dinamico di target atteso e vincoli di evitamento delle collisioni. Il controllo di velocità è affidato a un controllore PID dedicato.
4.4 Inibizione della Saturazione ed Emergenza Comportamentale
Qualora un incendio risulti già servito dalla capacità operativa massima ($N_{\max} = \text{MAX_DRONES_ON_FIRE}$), l'agente applica una deviazione radiale guidata dalla forza FIRE_SATURATION_BOUNCE_FORCE. Questa strategia impedisce l'affollamento e favorisce l'esplorazione del territorio circostante.
4.5 Soppressione dell'Incendio (try_extinguish)
Se il drone si trova all'interno del raggio di estinzione (FIRE_EXTINGUISH_RADIUS) ed è agganciato all'incendio, applica il getto d'acqua con una portata nominale sottraendo salute al fuoco in proporzione a $\Delta t$.

5. Modello di comunicazione localizzata
Il modello di comunicazione adotta una trasmissione push basata su un doppio buffer per garantire che le comunicazioni avvengano su uno snapshot temporale coerente.
Struttura del Pacchetto (DroneMessage)

CampoDescrizione ScientificapositionVettore di posizione espresso in coordinate mondo $\mathbb{R}^2$velocityVettore di velocità cinematica istantanea $\mathbb{R}^2$targetPosizione attrattore del punto obiettivo $\mathbb{R}^2$reloadingFlag booleano di stato critico della risorsa idricawater_station_idxIdentificatore della risorsa logistica prenotatarefuel_claim_ageTimestamp locale di accodamento per la ricaricaextinguishingFlag booleano di erogazione idrica attivafire_targetCoordinate dell'evento d'incendio ingaggiatoknown_firesMappa topologica locale degli incendi attiviextinguished_firesMappa topologica locale degli incendi soppressi


6. Propagazione della conoscenza ed epidemica degli incendi
La distribuzione dell'informazione nello sciame segue dinamiche tipiche dei protocolli epidemici (gossip protocols).
flowchart TD
    R1["1. Priorità di Estinzione:\nSe un vicino notifica l'estinzione di un fuoco con timestamp più recente,\nla memoria dell'agente si aggiorna e cancella il fuoco dalla lista attiva."]
    R2["2. Propagazione degli Incendi Attivi:\nI nuovi incendi vengono integrati solo se non figurano tra quelli già estinti."]
    R3["3. Bonifica della Memoria Locale:\nRimozione immediata delle posizioni estinte dai vettori di puntamento."]
    R1 --> R2 --> R3


7. Dinamica degli incendi e propagazione spontanea
L'incendio è modellato come un sistema termodinamico aperto caratterizzato da una variabile di stato $H(t)$ (salute/energia del fuoco).
7.1 Crescita Erogata
$$H(t + \Delta t) = H(t) + r_{\text{growth}} \cdot \Delta t$$
7.2 Stocasticità di Propagazione
Quando $H(t) > H_{\text{initial}} \times 1.20$, il focolaio presenta una probabilità ad ogni passo temporale di generare un focolaio secondario $H_{\text{child}} = 0.15 \times H_{\text{initial}}$ a una distanza casuale compresa tra $0.5\text{ m}$ e $2.0\text{ m}$.

8. Logistica idrica e gestione delle stazioni
Ciascun drone trasporta una risorsa idrica limitata $W_{\text{cap}} = 20.0$. Qualora il livello scenda sotto la soglia $W_{\text{low}} = 2.0$, l'agente attiva la procedura di rientro logistico.
Gestione delle Accodamenti Distribuiti
Le stazioni di ricarica ospitano un numero massimo fisso di unità in servizio contemporaneo ($\text{WATER_STATION_CAPACITY} = 2$). Gli agenti in eccesso si dispongono in configurazioni d'attesa circularizzate calcolate in funzione della priorità di richiesta (refuel_claim_age).

9. Controllo del moto e dinamica dei campi di forza
Il moto degli agenti è guidato da un modello a campi di forza potenziali applicato al punto attrattore (target), la cui dinamica è successivamente inseguita dall'agente mediante un controllore PID sulla velocità.
9.1 Composizione del Campo Potenziale
Il vettore di forza totale agisce sul target del drone ed è definito dalla somma ponderata dei seguenti contributi:

Repulsione tra Target: Mantiene la spaziatura di esplorazione tra gli agenti.
Forza dell'Ancora: Richiama il target verso la posizione originale d'esplorazione.
Attrattore dell'Incendio: Tira il target verso il focolaio ingaggiato.
Repulsione dei Bordi: Mantiene il target entro i limiti geografici del dominio.

9.2 Modello PID e Vincoli di Jerk
La velocità desiderata $v_{\text{des}}$ viene convertita in accelerazione dal controllore PID:
$$a(t) = K_p e_v(t) + K_i \int e_v(t) dt + K_d \frac{de_v(t)}{dt}$$
Inoltre, il sistema limita la variazione temporale dell'accelerazione per rispettare i vincoli cinematici del mezzo ($J_{\max} = 8.0\text{ m/s}^3$).

10. Collisioni e sicurezza operativa
Evitamento Predittivo
Gli agenti valutano le traiettorie estrapolate dei vicini nell'orizzonte temporale AVOID_LOOKAHEAD. Se la distanza minima prevista scende sotto la soglia di sicurezza, viene applicata una forza repulsiva ortogonale alla traiettoria di collisione. In condizioni di prossimità critica ($d < \text{EMERGENCY_AVOID_DISTANCE}$), si attiva una forza di emergenza ad alta priorità.

11. Parametri chiave del sistema

CategoriaParametroValore NominalUnità di MisuraAmbienteDimensioni Dominio$20.0 \times 12.0$$\text{m}$SciameDimensione Flotta ($N$)$12$UnitàComunicazioneRaggio di Copertura ($R_{\text{comm}}$)$2.5$$\text{m}$IncendiEnergia Iniziale ($H_{\text{initial}}$)$100.0$HPIncendiTasso di Crescita$0.5$$\text{HP/s}$IncendiCapacità Max Attacco$3$Droni/FuocoLogisticaRaggio di Servizio Stazione$0.65$$\text{m}$CinematicaVelocità Massima Agente$1.0$$\text{m/s}$CinematicaAccelerazione Massima$4.0$$\text{m/s}^2$


12. Interfaccia utente e rendering
La visualizzazione grafica in Pygame mostra in tempo reale:

La posizione degli agenti e il loro stato di carica idrica.
I focolai d'incendio e il relativo livello energetico residuale.
Le righe di connessione della rete ad-hoc di comunicazione.
I vettori del campo potenziale applicati ai target d'inseguimento.


13. Avvio e opzioni da riga di comando
Sintassi di esecuzione:
python pymain.py [OPZIONI]

Parametri CLI

--show-vectors: Abilita il rendering sovrapposto dei vettori di forza.
--comm: Visualizza la topologia della rete di comunicazione locale.
--random-fires: Inizializza gli incendi in posizioni casuali.
--random-stations: Dispone le stazioni idriche secondo una distribuzione stocastica vincolata.
--log: Genera il registro analitico delle intersezioni di sicurezza fisiche.


Documentazione teorico-operativa — Revisione: Settembre 2026