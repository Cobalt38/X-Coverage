# X-Coverage

Uno sciame di droni autonomi sorveglia un'area, trova gli incendi, li spegne e va a rifornirsi
d'acqua. Nessuno li comanda: non esiste una centrale che assegni gli incendi o i turni alla
stazione idrica. Ogni drone decide da solo con quello che vede, quello che ricorda e quello che si
sente dire dai vicini via radio.

![lo sciame al lavoro](docs/screenshot.png)

```bash
pip install -r requirements.txt
python main.py                      # guarda lo sciame lavorare
python main.py test                 # verifica che tutto funzioni
python main.py experiment ablation  # misura quanto serve ciascun meccanismo
```

---

## Il problema, in una riga

Dodici droni, un'area di 20 × 12 metri, tre incendi che crescono se nessuno li spegne e si
propagano se crescono troppo. Ogni drone vede solo entro 2 metri e parla solo entro 2.5 metri.
Porta 20 unità d'acqua, che bastano per 4 secondi di getto, mentre un incendio ne richiede 200:
servono dieci viaggi, e le stazioni idriche hanno solo due posti ciascuna.

Da questi vincoli nascono tutti i problemi interessanti: come si accorda uno sciame su chi va dove,
se nessuno ha una visione d'insieme e le informazioni arrivano vecchie e incomplete?

## Come leggere il codice

Cinque file, da leggere in quest'ordine. Ognuno comincia con un commento che spiega cosa c'è dentro
e perché.

| File | Cosa contiene |
|---|---|
| [`world.py`](world.py) | I parametri (tutti, in un posto solo) e il mondo fisico: incendi, terreno, radio, sensori. |
| [`drone.py`](drone.py) | L'agente: cosa sa, come decide, come vola. È il cuore del progetto. |
| [`simulation.py`](simulation.py) | Il giro di ogni passo, le conseguenze degli urti e la finestra grafica. |
| [`experiments.py`](experiments.py) | Le misure, gli esperimenti e i report con la statistica. |
| [`tests.py`](tests.py) | Le verifiche automatiche. |

C'è un solo file eseguibile, [`main.py`](main.py), che li mette insieme.

## Cosa fa un drone, un passo alla volta

Ogni centesimo di secondo simulato, ciascun drone ripete sempre la stessa sequenza
(`Drone.step()` in fondo a [`drone.py`](drone.py)):

```
   legge i messaggi arrivati
        ↓
   guarda intorno a sé (2 m) e invecchia i ricordi
        ↓
   unisce quello che dicono i vicini a quello che sa
        ↓
   decide cosa fare  →  acqua quasi finita?   vado alla stazione
        ↓                conosco un incendio libero?   ci vado
        ↓                nessuno dei due?   perlustro
   vola: obiettivo + evitamento → PID → accelerazione → posizione
        ↓
   se ha un fuoco a portata, spruzza; se è al suo posto alla stazione, carica
```

E l'intero sciame, sempre nello stesso ordine (`Simulation._advance_one_step()`):

```
chi sente chi  →  TUTTI trasmettono  →  TUTTI ragionano e si muovono
               →  gli incendi crescono  →  si controllano gli urti
```

Il fatto che *tutti* trasmettano prima che *chiunque* ragioni non è un dettaglio: è ciò che rende
la simulazione equa. Altrimenti il drone numero 0, che ragiona per primo, deciderebbe sapendo già
cosa hanno fatto gli altri nello stesso istante.

## Le quattro idee che tengono in piedi il sistema

**1. Le notizie invecchiano.** Un drone non memorizza "qui c'è un incendio", ma "qui c'è un
incendio, e questa notizia ha 37 centesimi di secondo". Quando due droni si incontrano, per ogni
incendio tengono la versione più fresca. Così un avvistamento si propaga di vicino in vicino fino
a droni che sono a venti metri di distanza, e una notizia che nessuno conferma più scade da sola.
Chi passa sul posto e non vede nulla genera la smentita, che viaggia allo stesso modo e ha la
precedenza sull'avvistamento.

**2. Le regole di precedenza sostituiscono l'accordo.** Nessuno assegna gli incendi. Quando un
drone deve decidere se occuparsi di un fuoco, ordina i pretendenti con una regola: prima chi ci sta
già lavorando, poi chi è più vicino, e a parità l'indice più basso. Se davanti a sé ne trova già
tre, si tira indietro da solo. Tutti applicano la stessa regola agli stessi dati e arrivano alla
stessa conclusione, senza scambiarsi una parola in più. La stessa idea regola la coda alla stazione
idrica.

Funziona però solo se chi deve confrontarsi si sente: due droni sullo stesso incendio distano al
massimo 2 × 1.2 metri, quindi stanno dentro i 2.5 metri della radio. Vincoli di questo tipo sono
scritti in `validate_config()` in [`world.py`](world.py), che li controlla all'avvio invece di
lasciarli impliciti.

**3. La geometria fa il lavoro del coordinatore.** Un drone che arriva su un incendio non punta al
centro, ma a un anello di 80 centimetri di raggio, dal lato da cui arriva. L'obiettivo fissa solo
la distanza dal fuoco; *dove* disporsi lungo l'anello lo decide da sé l'evitamento collisioni, che
spinge i droni di lato finché non sono abbastanza distanti. Tre droni si distribuiscono attorno al
fuoco senza che nessuno lo decida. Lo stesso vale per chi aspetta il proprio turno all'acqua.

**4. Scansarsi guardando avanti, non indietro.** Per ogni vicino, il drone calcola *quando* e *con
quanto scarto* si mancheranno, se entrambi proseguono dritti, e spinge nella direzione di
quello scarto: si sposta di lato invece di frenare. Guardare solo "dove saremo tra un secondo e
mezzo" non basterebbe, perché due droni che si incrociano possono sfiorarsi a metà strada ed essere
di nuovo lontani alla fine. Il dettaglio matematico è nel commento di `_avoidance_correction()`.

## Quando qualcosa va storto

Gli urti non sono un numero in una statistica: due droni che si toccano si rompono, e non tornano
più a volare.

- **Impatto leggero** (sotto 0.6 m/s di velocità relativa): si rompono i motori. Il drone precipita
  dov'era, ma la radio continua a funzionare: resta un ripetitore fermo che invecchia e ritrasmette
  quello che sapeva, quindi la sua conoscenza non va persa.
- **Impatto forte**: perdita totale, radio compresa. Per lo sciame è come se non fosse mai esistito.
- Un relitto caduto **dentro un incendio** brucia, e dopo venti secondi tace anche lui.

Nella finestra i relitti sono delle X: grigia se la radio è viva, rossa se è tutto spento.

Serve a misurare quanto valga davvero l'evitamento collisioni. L'esperimento `urti` (12 simulazioni
per variante, scenario «critico») dice questo:

| | con evitamento | senza evitamento |
|---|---|---|
| Urti | **0** | 5.4 |
| Droni fuori uso su 12 | **0** | 10.8 |
| Missione riuscita | **75%** | 0% |
| Incendi mai avvistati | 2.9 | 21.9 |

Con l'evitamento attivo i droni non si toccano mai, quindi rendere gli urti letali non cambia
nulla: le tre varianti con evitamento danno risultati identici fino all'ultima cifra. Senza, cinque
urti bastano a mettere a terra dieci droni su dodici, e da lì gli incendi dilagano.

## Cercare, non solo spegnere

Quando non c'è niente da spegnere, un drone perlustra. Ci sono due strategie, scelte con il
parametro `EXPLORATION_MODE`:

- `"random"`: sceglie un punto a caso nell'area. Semplice, e sorprendentemente difficile da battere
  quando i droni sono tanti e l'area è piccola.
- `"coverage"`: va dove non si guarda da più tempo, pesando quanto vale quella zona. Ogni drone
  tiene una mappa dell'area con l'istante dell'ultima occhiata per ogni cella, la scambia via radio
  con i vicini (unirle è un massimo cella per cella, quindi l'ordine e i messaggi persi non
  contano) e sceglie la meta nella propria zona di competenza, cioè le celle più vicine a lui che
  a qualunque vicino di cui abbia notizie.

Il tasto `H` nella finestra mostra la mappa del terreno e l'obsolescenza: le zone blu sono quelle
che nessuno guarda da un minuto.

Questa parte ha prodotto il risultato più interessante del progetto, che si può rifare con
`python main.py experiment importanza`. Concentrarsi sulle zone di valore rende, ma solo fino a un
certo punto: spingendo troppo, lo sciame presidia benissimo gli hotspot e abbandona tutto il resto,
dove comunque nascono incendi. La via di mezzo (γ = 0.5) batte entrambi gli estremi.

## Misurare invece di guardare

Un esperimento risponde a una domanda del tipo "questo meccanismo serve davvero?", confrontando
varianti del sistema nella stessa identica situazione:

```bash
python main.py experiment              # elenca le domande disponibili
python main.py experiment ablation --runs 30
```

Ogni variante viene simulata trenta volte, sempre con gli stessi trenta seed. Con lo stesso seed
gli incendi nascono negli stessi punti e i droni partono dagli stessi posti: le simulazioni si
confrontano quindi **a coppie**, e resta solo l'effetto della modifica in esame. Il risultato è una
cartella in `results/` con un report leggibile e i dati grezzi in CSV.

Il report segnala con ▲ e ▼ le differenze che superano un test statistico, spiegato per esteso nei
commenti di [`experiments.py`](experiments.py): in sostanza si chiede quanto sarebbe facile
ottenere per puro caso una differenza grande come quella osservata.

Domande già pronte: `ablation` (a cosa serve ogni meccanismo), `urti` (quanto costa un incidente),
`radio` (quanto pesa perdere messaggi), `difficolta`, `flotta` (quanti droni servono), `copertura`
e `importanza` (come conviene cercare).

## Onestà sui limiti

- **L'ambiente è centralizzato, il controllo no.** Il simulatore conosce tutto perché *è* la realtà
  fisica: calcola chi sente chi, fa crescere gli incendi, rileva gli urti. Ma non decide mai al
  posto dei droni: nel codice non c'è una riga che assegni un incendio a qualcuno. La formula
  corretta è "controllo decentralizzato simulato in un ambiente centralizzato".
- **Nessuna risposta fisica all'urto**: i droni non rimbalzano, si rompono e cadono.
- **Il limite di tre droni per incendio vale sull'incendio, non lungo il tragitto**: droni che
  convergono da lontano non si sentono finché non si avvicinano.
- **Il modello di incendio è un'astrazione**: un punto con una "vita" che cresce, non un fronte di
  fiamma con vento e combustibile.
- I droni volano su un piano, con una dinamica semplificata (velocità e accelerazione limitate,
  niente assetto né quota).

## Requisiti

Python 3.9 o successivo, `numpy` e `pygame` (quest'ultimo serve solo per la finestra: gli
esperimenti e i test girano senza).
