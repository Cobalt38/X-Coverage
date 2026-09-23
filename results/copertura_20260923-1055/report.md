# Esperimento «copertura»

**Domanda.** Perlustrare dove non si guarda da più tempo, e dove conta di più, fa trovare prima gli incendi?

**Scenario «ricerca».** Area quattro volte più grande (40x24 m), un solo incendio iniziale e accensioni spontanee distribuite secondo la mappa di importanza (una ogni ~33 s). Qui il problema non è l'acqua ma TROVARE gli incendi: i droni passano l'80% del tempo a perlustrare, contro il 6% dello scenario 'critico'.
Parametri diversi dal default: AREA_WIDTH = 40.0, AREA_HEIGHT = 24.0, NUM_FIRES = 1, IGNITION_RATE_PER_S = 0.03. Ogni simulazione dura al massimo 300 s ed è fermata (e conta come fallita) se gli incendi accesi superano 40.

**Varianti.** «ricerca casuale» (nessuno); «copertura persistente» (EXPLORATION_MODE = coverage); «copertura uniforme» (EXPLORATION_MODE = coverage, COVERAGE_IMPORTANCE_EXPONENT = 0.0). Il riferimento è «ricerca casuale».

**Metodo.** 16 simulazioni per variante, con i seed 1–16, uguali per tutte le varianti. Eseguito il 23/09/2026 alle 10:55 in 16.9 minuti (codice: commit 9b738fc con modifiche non salvate).

## In breve

⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare significative. Per conclusioni affidabili usare almeno 20 simulazioni (--runs 20).

- **copertura persistente**: nessun cambiamento tra le metriche principali. Cambiano in modo significativo anche altre 13 metriche (vedi tabelle).
- **copertura uniforme**: nessun cambiamento tra le metriche principali. Cambiano in modo significativo anche altre 7 metriche (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «ricerca casuale»; ◆ = significativamente diverso (per metriche senza un valore "migliore").

### Missione

| Metrica | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Missione riuscita | 88% (64%–97%) | 62% (39%–82%) | 94% (72%–99%) |
| Incendi fuori controllo | 6.2% (1.1%–28%) | 25% (10%–49%) | 0% (0%–19%) |
| Tempo per spegnere tutto [s] | 123 (77–168) | 76.3 (59.4–93.2) | 130 (89.2–171) |
| Danno degli incendi [vita·s] | 33 268 (10 178–56 359) | 65 206 (26 204–104 209) | 23 675 (20 260–27 090) |
| Incendi nati dalla propagazione | 2.69 (0–8.41) | 10.1 (0.447–19.8) | 0 (0–0) |
| Incendi nati da soli | 6.88 (5.42–8.33) | 6.5 (4.82–8.18) | 7.12 (5.67–8.58) |
| Picco di incendi accesi | 5.62 (0.56–10.7) | 12.6 (3.6–21.7) | 3.5 (2.85–4.15) |
| Ritardo di avvistamento [s] | 21.3 (16.4–26.1) | 5.92 (3.85–8) ▲ | 19.6 (15.3–23.9) |
| Incendi mai avvistati | 2.25 (0–5.66) | 9.88 (1.45–18.3) | 0.688 (0.183–1.19) |
| Ritardo di intervento [s] | 2.03 (1.38–2.69) | 2.48 (0–5.65) | 1.85 (1.41–2.29) |

### Sicurezza

| Metrica | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Collisioni | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Quasi-collisioni | 0.0625 (0–0.196) | 0.0625 (0–0.196) | 0.0625 (0–0.196) |
| Distanza minima tra due droni [m] | 0.986 (0.881–1.09) | 0.996 (0.88–1.11) | 0.997 (0.89–1.1) |
| Tempo in emergenza | 0.0% (0%–0.0%) | 0.0% (0%–0.0%) | 0.0% (0%–0.0%) |

### Uso del tempo

| Metrica | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Esplorazione | 80% (77%–83%) | 85% (80%–89%) ◆ | 80% (77%–83%) |
| In volo verso un incendio | 7.4% (6.1%–8.7%) | 6.2% (4.3%–8.1%) | 7.4% (6.1%–8.7%) |
| Spegnimento | 3.1% (2.7%–3.5%) | 2.3% (1.7%–2.8%) ▼ | 3.0% (2.7%–3.4%) |
| In volo verso una stazione | 7.8% (6.3%–9.4%) | 5.9% (3.8%–8.1%) ▲ | 8.0% (6.7%–9.2%) |
| In coda alla stazione | 0.1% (0.1%–0.2%) | 0.1% (0.0%–0.1%) | 0.1% (0.0%–0.3%) |
| Rifornimento | 1.4% (1.2%–1.6%) | 1.0% (0.7%–1.2%) ◆ | 1.4% (1.2%–1.6%) |

### Lavoro ed efficienza

| Metrica | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Equità del lavoro | 0.675 (0.593–0.758) | 0.619 (0.506–0.733) | 0.699 (0.643–0.754) |
| Sovraffollamento sugli incendi [s] | 0 (0–0) | 0.0119 (0–0.0372) | 0 (0–0) |
| Attesa media in coda [s] | 0.188 (0.0831–0.294) | 0.163 (0.0222–0.305) | 0.197 (0.0452–0.35) |
| Attesa massima in coda [s] | 2.68 (1.23–4.13) | 1.66 (0.631–2.69) | 2.39 (0.884–3.89) |
| Distanza percorsa [m] | 2 953 (2 829–3 077) | 1 171 (1 032–1 310) ▲ | 3 072 (3 040–3 103) ▼ |
| Sforzo di controllo [m²/s³] | 1e+03 (940–1 060) | 1 135 (1 019–1 251) ▼ | 2 632 (2 567–2 697) ▼ |
| Rimbalzi da incendi pieni | 0.688 (0.113–1.26) | 1.44 (0.568–2.31) | 0.438 (0–1.11) |

### Comunicazione e conoscenza

| Metrica | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 0.588 (0.523–0.653) | 1.32 (1.19–1.46) ▲ | 0.516 (0.46–0.572) ▼ |
| Sciame tutto connesso | 0.1% (0%–0.4%) | 0.7% (0%–1.7%) | 0% (0%–0%) |
| Incendi noti ai droni | 15% (12%–18%) | 13% (7.9%–18%) | 17% (15%–19%) |
| Incendi fantasma per drone | 0.0104 (0.0069–0.014) | 0.0145 (0.00203–0.0269) | 0.013 (0.00851–0.0175) |
| Età delle informazioni [s] | 1.11 (0.716–1.5) | 0.642 (0.195–1.09) ▲ | 1.14 (0.602–1.67) |

### Perlustrazione

| Metrica | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Obsolescenza del terreno [s] | 30.4 (27.7–33.1) | 46.8 (43.1–50.5) ▼ | 27 (25–29) ▲ |
| Obsolescenza delle zone importanti [s] | 21.7 (18.5–24.9) | 5.81 (4.24–7.37) ▲ | 26.3 (23.5–29.1) ▼ |
| Obsolescenza del resto dell'area [s] | 38.6 (35.7–41.5) | 96.7 (88.9–105) ▼ | 27.6 (26–29.3) ▲ |
| Obsolescenza della zona peggiore [s] | 191 (163–219) | 96.5 (72.9–120) ▲ | 139 (128–150) ▲ |

## Come leggere questi numeri

- **Intervallo di confidenza al 95%**: se si ripetesse l'esperimento con infinite simulazioni, la media cadrebbe quasi certamente in quell'intervallo. Intervalli larghi = servono più simulazioni.
- **▲ ▼ ◆**: la differenza dal riferimento supera il test statistico (p < 0.05), cioè difficilmente è dovuta al caso. Le simulazioni sono confrontate a coppie con lo stesso seed (test di permutazione dei segni; test di McNemar per le metriche sì/no). Significativo non vuol dire grande: guardare anche di quanto cambia la media.
- **Tempo per spegnere tutto** è calcolato solo sulle missioni riuscite: va letto insieme a «Missione riuscita».
- Le simulazioni si fermano quando la missione riesce o gli incendi vanno fuori controllo, quindi hanno durate diverse: danno, distanza e sforzo accumulati vanno confrontati con cautela tra varianti con esiti molto diversi.

## Significato delle metriche

- **Missione riuscita** (`mission_complete`): Tutti gli incendi sono stati spenti entro la durata massima.
- **Incendi fuori controllo** (`fire_overrun`): La simulazione è stata interrotta perché gli incendi accesi erano troppi.
- **Tempo per spegnere tutto** (`extinction_time_s`): Secondi fino allo spegnimento dell'ultimo incendio (solo missioni riuscite).
- **Danno degli incendi** (`fire_damage`): Somma nel tempo della vita di tutti gli incendi accesi: premia chi spegne presto.
- **Incendi nati dalla propagazione** (`fires_spawned`): Un incendio lasciato crescere ne genera altri vicino a sé.
- **Incendi nati da soli** (`fires_ignited`): Accensioni spontanee, indipendenti dagli incendi già presenti (IGNITION_RATE_PER_S). Dipendono solo dal seed: a parità di seed sono le stesse per tutte le varianti.
- **Picco di incendi accesi** (`peak_active_fires`): Massimo numero di incendi accesi contemporaneamente.
- **Ritardo di avvistamento** (`detection_delay_s`): Tempo medio tra la nascita di un incendio e il primo drone che lo vede. Attenzione: media solo sugli incendi effettivamente avvistati, quindi va letta insieme alla metrica seguente: una strategia che ignora del tutto certe zone migliora questa media.
- **Incendi mai avvistati** (`fires_never_seen`): Incendi ancora accesi a fine simulazione che nessun drone ha mai visto.
- **Ritardo di intervento** (`response_delay_s`): Tempo medio tra l'avvistamento di un incendio e la prima acqua che riceve.
- **Collisioni** (`collisions`): Coppie di droni entrate a contatto (distanza < DRONE_IMPACT_RADIUS).
- **Quasi-collisioni** (`near_misses`): Coppie di droni scese sotto la distanza di emergenza (EMERGENCY_AVOID_DISTANCE).
- **Distanza minima tra due droni** (`min_distance_m`): La distanza più piccola mai registrata tra due droni.
- **Tempo in emergenza** (`emergency_fraction`): Quota del tempo in cui i droni si scansano invece di lavorare.
- **Esplorazione** (`time_explore`): Nessun compito: il drone perlustra l'area.
- **In volo verso un incendio** (`time_to_fire`): Ha scelto un incendio e ci sta andando.
- **Spegnimento** (`time_extinguishing`): Sta spruzzando acqua su un incendio.
- **In volo verso una stazione** (`time_to_station`): Acqua quasi finita: sta andando a rifornirsi.
- **In coda alla stazione** (`time_queue`): Arrivato alla stazione, aspetta che si liberi un posto.
- **Rifornimento** (`time_refill`): Sta caricando acqua.
- **Equità del lavoro** (`water_fairness`): Indice di Jain sull'acqua erogata da ogni drone: 1 = tutti lavorano uguale, verso 0 = pochi fanno tutto.
- **Sovraffollamento sugli incendi** (`overcrowding_s`): Secondi in cui su un incendio lavoravano più di MAX_DRONES_ON_FIRE droni.
- **Attesa media in coda** (`queue_wait_mean_s`): Attesa media alla stazione idrica per ogni rifornimento.
- **Attesa massima in coda** (`queue_wait_max_s`): L'attesa più lunga registrata a una stazione idrica.
- **Distanza percorsa** (`distance_m`): Metri percorsi da tutti i droni insieme.
- **Sforzo di controllo** (`control_effort`): Quanto i droni hanno accelerato e frenato (∫|a|² dt): approssima il consumo di batteria.
- **Rimbalzi da incendi pieni** (`saturation_bounces`): Volte in cui un drone si è allontanato da un incendio già occupato da abbastanza droni.
- **Messaggi persi** (`message_loss`): Quota dei messaggi radio che non sono arrivati.
- **Vicini radio per drone** (`neighbors`): Quanti altri droni sente in media ogni drone (messaggi effettivamente ricevuti).
- **Sciame tutto connesso** (`connected_fraction`): Quota del tempo in cui ogni drone può raggiungere ogni altro, anche passando per altri.
- **Incendi noti ai droni** (`fire_awareness`): In media, quale quota degli incendi accesi conosce ogni drone.
- **Incendi fantasma per drone** (`phantom_fires`): Incendi che un drone crede accesi ma sono già stati spenti.
- **Età delle informazioni** (`info_age_s`): Quanto sono vecchie, in media, le notizie che i droni hanno sugli incendi accesi.
- **Obsolescenza del terreno** (`coverage_staleness_s`): Da quanto tempo, in media, una zona non viene guardata da nessun drone, pesando ogni zona per la sua importanza. È l'obiettivo classico della copertura persistente e si misura allo stesso modo con qualunque strategia di perlustrazione.
- **Obsolescenza delle zone importanti** (`coverage_staleness_hot_s`): Come sopra, ma solo sulle zone a importanza alta (I >= 0.5).
- **Obsolescenza del resto dell'area** (`coverage_staleness_cold_s`): Come sopra, sulle zone a importanza bassa. Insieme alla precedente mostra come una strategia distribuisce l'attenzione: concentrarsi sulle zone importanti peggiora questa.
- **Obsolescenza della zona peggiore** (`coverage_staleness_max_s`): Obsolescenza della zona importante (I >= 0.5) lasciata più a lungo senza controllo.

I dati di ogni singola simulazione sono in `runs.csv` (una riga per simulazione, colonne = nomi tra parentesi qui sopra).
