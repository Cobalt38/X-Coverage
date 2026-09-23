# Stato del lavoro

Appunti per riprendere il progetto da un'altra macchina o dopo una pausa. Il punto d'ingresso per
capire **cosa fa** il sistema resta il [README](../README.md); qui c'è quello che il README non dice:
le decisioni prese, cosa è già stato misurato e cosa è ancora aperto.

Aggiornato al 23 settembre 2026.

---

## Dove siamo

Sei file Python, un solo eseguibile (`main.py`), 41 verifiche automatiche che passano in un minuto
(`python main.py test`). La simulazione gira a ~1700 passi al secondo con la perlustrazione casuale,
~1400 con la copertura.

Sette esperimenti pronti in `experiments.py`: `ablation`, `urti`, `guasti`, `radio`, `difficolta`,
`flotta`, `copertura`, `importanza`. I report validi stanno in `results/`.

## Decisioni di progetto che non si deducono dal codice

**Il limite di droni per incendio è una stima locale, non un contatore.** Vale perché la geometria lo
garantisce: due droni sullo stesso incendio stanno entro il raggio radio. Questo vincolo, e gli altri
della stessa famiglia, sono controllati all'avvio da `validate_config()`. Cambiare un raggio senza
guardare gli altri rompe il ragionamento, e il controllo lo dice.

**L'ambiente è centralizzato, il controllo no.** Il simulatore conosce tutto perché è la fisica; i
droni no. La verifica che regge questa affermazione è in `tests.py::Autonomia`: un drone non tiene
riferimenti ad altri droni, e le sue decisioni cambiano solo in base ai messaggi che riceve.

**Le tre fasi del clock** (trasmettere, ragionare, agire) servono a non dare vantaggi di ordine.
Toglierne una fa tornare l'asimmetria: chi agisce prima toglie acqua agli altri, chi ragiona dopo
decide su un mondo già cambiato.

**Un drone che precipita dichiara di non volare più.** Su un multirotore reale è un'informazione
gratuita (assetto, accelerometro, giri dei motori). L'alternativa è `WRECK_BLOCKS_FLIGHT`.

**Il guasto silenzioso** (`SILENT_FAILURE_PROBABILITY`) è il terzo modo di rompersi: la radio si
blocca sull'ultima fotografia e continua a dichiarare "sto volando". È la modalità più interessante
dal punto di vista scientifico, vedi sotto.

## Che cosa dicono le misure

Tutto riproducibile con `python main.py experiment NOME --runs N`; i report completi sono in
`results/`.

**Evitamento collisioni** (`urti`, 12 run): con l'evitamento attivo zero urti in tutte le
simulazioni. Senza, sei urti mettono a terra undici droni su dodici e la riuscita passa da 92% a 8%.

**Modalità di guasto** (`guasti`, 20 run, tre droni su dodici rotti al secondo 30): perderli in modo
onesto non cambia la probabilità di riuscita (70%); perderli in silenzio la porta al 50%, con gli
incendi fantasma per drone che triplicano (0.10 → 0.31). L'effetto sui fantasmi supera il test
statistico, quello sulla riuscita no: servono più ripetizioni.

**Perlustrazione** (`copertura`, 30 run): la copertura uniforme non migliora il caso tipico (vince in
13 casi su 30, mediana identica alla ricerca casuale) ma elimina la coda — le tre simulazioni
peggiori passano da ~900 a ~170 di fuoco acceso medio, e le tre missioni fuori controllo diventano
zero. Sulle medie il test dà p ≈ 0.2: il vantaggio sta negli eventi rari.

**Concentrazione sulle zone di valore** (`importanza`, 12 run): ogni aumento di γ peggiora danno,
incendi mai visti e obsolescenza complessiva, comprando solo qualche secondo di freschezza sugli
hotspot. γ = 0 è il migliore dei quattro valori provati.

## Questioni aperte

**1. La legge di allocazione dello sforzo.** La teoria classica della ricerca (Koopman) dà, per un
sensore con rivelazione esponenziale, uno sforzo ottimo proporzionale a `max(ln p − α, 0)`:
logaritmico e con una soglia sotto la quale non si cerca affatto. Qui l'allocazione è `importanza^γ`,
e le misure dicono che γ = 0 batte γ = 1. Implementare la regola logaritmica come terza strategia e
confrontarla con la famiglia γ darebbe al risultato un aggancio teorico.
Attenzione: la "regola della radice quadrata" che si trova citata nei sistemi di polling **non è
stata verificata** — il paper di riferimento (Boxma, Levy, Weststrate 1991) è dietro paywall, e il
√3 del lavoro sul patrolling di Chuangpishit et al. è un rapporto di approssimazione, non una radice
di un peso.

**2. Misure costruite sulla coda.** Il risultato vero della perlustrazione sistematica è la sparizione
dei disastri, ma le misure attuali sono medie, che quel fenomeno lo nascondono. Serve qualcosa come
"probabilità di perdere il controllo", con abbastanza ripetizioni per stimarla.

**3. Guasti parziali contro guasti totali.** Il guasto silenzioso danneggia lo sciame più di una
perdita totale, e il meccanismo è misurabile (memoria collettiva avvelenata da notizie che non
invecchiano più). È il contributo più originale emerso finora; per sostenerlo servono 50-100
ripetizioni invece di 20.

**4. Quante ripetizioni servono davvero.** Con N simulazioni la probabilità più piccola ottenibile dal
test di permutazione è 2/2^N; moltiplicata per il numero di confronti del report può restare sopra la
soglia. Sotto le ~16 ripetizioni nessun risultato può risultare significativo, per quanto grande sia
la differenza. Il report lo segnala da solo, ma va tenuto presente quando si progetta un esperimento.

## Come si lavora su questo progetto

Due preferenze già concordate, valide per chiunque (o qualunque assistente) ci metta mano:

- **niente riferimenti a versioni passate del codice**: i commenti spiegano perché il codice è come
  è, non contro cosa si contrappone. La storia sta in git;
- i compiti meccanici (cancellare, rinominare, controllare) si delegano, non occupano il lavoro
  principale.
