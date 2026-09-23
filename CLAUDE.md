# Istruzioni per chi lavora su questo progetto

Simulazione 2D di uno sciame decentralizzato di droni antincendio. Il [README](README.md) spiega
cosa fa il sistema; [docs/stato-del-lavoro.md](docs/stato-del-lavoro.md) contiene le decisioni di
progetto, i risultati già misurati e le questioni aperte. **Leggi entrambi prima di modificare
qualcosa**: molte scelte che sembrano arbitrarie hanno una ragione misurata.

## Come si parla qui

Il progetto è scritto e commentato **in italiano**, e va mantenuto così: nomi di funzioni e
variabili in inglese, tutto il resto (commenti, docstring, documentazione, messaggi a schermo) in
italiano.

Il destinatario finale è un professore che leggerà il codice senza poter fare domande. Quindi:
commenti che spiegano **perché**, non cosa; nessuna abbreviazione nei nomi; un flusso leggibile
dall'alto in basso.

## Regole di scrittura

**Mai riferimenti a versioni precedenti del codice.** Niente "prima era così", "ex NOME_VECCHIO",
"(prima bastava X)", "dopo questa correzione". Chi legge vede solo la versione attuale: la
cronologia è rumore. Se serve motivare una scelta, usa l'alternativa in forma ipotetica ("se il
guadagno fosse limitato a 1, i droni si fermerebbero dopo pochi metri"), mai storica. La storia sta
in git.

**Documentazione asciutta.** Il README è già stato snellito due volte: prima di aggiungere una
sezione, chiediti se sostituisce qualcosa. Le tabelle dei risultati valgono più dei paragrafi.

**Delega i compiti meccanici.** Cancellare file, rinominare, controlli ripetitivi: vanno a un
sotto-agente con modello economico, non al lavoro principale.

## Struttura

Sei file, catena di import lineare, un solo eseguibile:

    world.py        parametri (SimConfig) e mondo fisico: incendi, terreno, radio, sensori, clock
    drone.py        l'agente: percezione, memoria, decisioni, volo, copertura
    simulation.py   il giro di ogni passo, gli urti con i loro danni, la finestra Pygame
    experiments.py  misure, esperimenti, statistica, report
    tests.py        verifiche automatiche
    main.py         unico punto d'ingresso: run / experiment / test

    python main.py                          finestra
    python main.py test                     ~1 minuto, deve restare verde
    python main.py experiment NOME --runs N  15-20 minuti con N=30

## Cose che costano tempo se non le sai

**Le impronte di regressione.** `tests.py::Regressione` confronta le traiettorie con un hash. Se
tocchi la logica dei droni il test fallisce: è il suo mestiere. Aggiorna l'hash solo se il
cambiamento è voluto, e sappi che i risultati raccolti prima non sono confrontabili con i nuovi.

**Gli invarianti geometrici.** `validate_config()` in `world.py` controlla che le distanze stiano
nell'ordine giusto (urto < emergenza < distanza operativa < raggio radio) e che la geometria regga
le ipotesi della logica decentralizzata. Cambiare un raggio senza guardare gli altri rompe
ragionamenti che sembrano indipendenti: leggi i messaggi che stampa all'avvio.

**Ogni cosa casuale ha il suo generatore.** Scenario, droni, radio, accensioni spontanee e guasti
provocati usano generatori separati, così cambiare una variante non sposta lo scenario in cui viene
confrontata. Se aggiungi qualcosa di casuale, dagli un generatore suo, altrimenti rompi
l'accoppiamento per seed su cui si regge tutta la statistica.

**Quante ripetizioni servono.** Con N simulazioni la probabilità più piccola ottenibile dal test di
permutazione è 2/2^N, e va moltiplicata per il numero di confronti del report (correzione di Holm).
Sotto le ~16 ripetizioni nessun risultato può risultare significativo, per quanto grande sia la
differenza. Per conclusioni serie: `--runs 30`.

**Misure e durata.** Le grandezze che si accumulano (danno, distanza, sforzo) crescono con la durata
della missione, e una variante che collassa prima ne accumula meno. Per confrontare varianti si
usano le versioni divise per il tempo; le metriche condizionate (il ritardo di avvistamento media
solo gli incendi trovati) vanno lette insieme al conteggio di quelli mai visti.

## Cosa non fare

- Non far leggere a un drone lo stato di un altro drone: tutto passa dai messaggi. C'è un test che
  lo verifica (`tests.py::Autonomia`).
- Non introdurre decisioni nel mondo o nella simulazione: `SimulationWorld` offre sensori, attuatori
  e radio, non sceglie mai per conto dei droni.
- Non committare senza che sia stato chiesto.
