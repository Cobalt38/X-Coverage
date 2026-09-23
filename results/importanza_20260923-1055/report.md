# Esperimento «importanza»

**Domanda.** Quanto conviene concentrare la perlustrazione dove gli incendi sono più probabili?

**Scenario «ricerca».** Area quattro volte più grande (40x24 m), un solo incendio iniziale e accensioni spontanee distribuite secondo la mappa di importanza (una ogni ~33 s). Qui il problema non è l'acqua ma TROVARE gli incendi: i droni passano l'80% del tempo a perlustrare, contro il 6% dello scenario 'critico'.
Parametri diversi dal default: AREA_WIDTH = 40.0, AREA_HEIGHT = 24.0, NUM_FIRES = 1, IGNITION_RATE_PER_S = 0.03. Ogni simulazione dura al massimo 300 s ed è fermata (e conta come fallita) se gli incendi accesi superano 40.

**Varianti.** «concentrazione γ=0» (EXPLORATION_MODE = coverage, COVERAGE_IMPORTANCE_EXPONENT = 0.0); «concentrazione γ=0.5» (EXPLORATION_MODE = coverage, COVERAGE_IMPORTANCE_EXPONENT = 0.5); «concentrazione γ=1» (EXPLORATION_MODE = coverage, COVERAGE_IMPORTANCE_EXPONENT = 1.0); «concentrazione γ=2» (EXPLORATION_MODE = coverage, COVERAGE_IMPORTANCE_EXPONENT = 2.0). Il riferimento è «concentrazione γ=0».

**Metodo.** 12 simulazioni per variante, con i seed 1–12, uguali per tutte le varianti. Eseguito il 23/09/2026 alle 10:55 in 17.1 minuti (codice: commit 9b738fc con modifiche non salvate).

## In breve

⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare significative. Per conclusioni affidabili usare almeno 20 simulazioni (--runs 20).

- **concentrazione γ=0.5**: nessun cambiamento tra le metriche principali. Cambiano in modo significativo anche altre 6 metriche (vedi tabelle).
- **concentrazione γ=1**: danno degli incendi 24 007 → 78 867 (peggiore). Cambiano in modo significativo anche altre 11 metriche (vedi tabelle).
- **concentrazione γ=2**: missione riuscita 92% → 17% (peggiore); danno degli incendi 24 007 → 128 569 (peggiore); incendi noti ai droni 17% → 8.3% (peggiore). Cambiano in modo significativo anche altre 14 metriche (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «concentrazione γ=0»; ◆ = significativamente diverso (per metriche senza un valore "migliore").

### Missione

| Metrica | concentrazione γ=0 | concentrazione γ=0.5 | concentrazione γ=1 | concentrazione γ=2 |
|---|---|---|---|---|
| Missione riuscita | 92% (65%–99%) | 92% (65%–99%) | 58% (32%–81%) | 17% (4.7%–45%) ▼ |
| Incendi fuori controllo | 0% (0%–24%) | 0% (0%–24%) | 33% (14%–61%) | 58% (32%–81%) ▼ |
| Tempo per spegnere tutto [s] | 146 (92.5–200) | 114 (85.3–143) | 81.2 (56.2–106) | 99.5 (0–640) |
| Danno degli incendi [vita·s] | 24 007 (19 443–28 570) | 20 974 (15 691–26 257) | 78 867 (27 856–129 877) ▼ | 128 569 (72 750–184 388) ▼ |
| Incendi nati dalla propagazione | 0 (0–0) | 0 (0–0) | 13.5 (0.788–26.2) | 23.1 (9.97–36.2) ▼ |
| Incendi nati da soli | 6.83 (5.1–8.56) | 6.83 (5.1–8.56) | 6 (3.97–8.03) | 5.08 (3.3–6.87) ◆ |
| Picco di incendi accesi | 3.25 (2.64–3.86) | 2.83 (2.13–3.54) | 15.6 (3.65–27.5) | 25.3 (13–37.6) ▼ |
| Ritardo di avvistamento [s] | 20.1 (14.4–25.9) | 12.4 (8.83–15.9) ▲ | 4.85 (2.83–6.87) ▲ | 20.7 (5.8–35.7) |
| Incendi mai avvistati | 0.75 (0.137–1.36) | 0.333 (0–0.747) | 12.6 (1.42–23.7) ▼ | 21.2 (8.91–33.6) ▼ |
| Ritardo di intervento [s] | 2.07 (1.54–2.59) | 1.51 (1.09–1.92) | 3.06 (0–7.54) | 1.95 (0.0919–3.81) |

### Sicurezza

| Metrica | concentrazione γ=0 | concentrazione γ=0.5 | concentrazione γ=1 | concentrazione γ=2 |
|---|---|---|---|---|
| Collisioni | 0 (0–0) | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Quasi-collisioni | 0 (0–0) | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Distanza minima tra due droni [m] | 1.04 (1–1.08) | 1.04 (0.997–1.08) | 1.04 (0.966–1.11) | 1.09 (0.958–1.21) |
| Tempo in emergenza | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |

### Uso del tempo

| Metrica | concentrazione γ=0 | concentrazione γ=0.5 | concentrazione γ=1 | concentrazione γ=2 |
|---|---|---|---|---|
| Esplorazione | 80% (76%–84%) | 80% (76%–84%) | 84% (78%–90%) | 86% (76%–95%) |
| In volo verso un incendio | 7.4% (5.8%–8.9%) | 7.5% (5.6%–9.4%) | 6.6% (4.0%–9.1%) | 5.3% (2.2%–8.4%) |
| Spegnimento | 3.0% (2.6%–3.4%) | 3.0% (2.5%–3.4%) | 2.2% (1.4%–2.9%) ▼ | 1.9% (0.9%–3.0%) ▼ |
| In volo verso una stazione | 8.0% (6.4%–9.7%) | 8.0% (6.4%–9.5%) | 6.1% (3.2%–9.1%) | 5.9% (1.4%–10%) |
| In coda alla stazione | 0.1% (0%–0.3%) | 0.2% (0.0%–0.4%) | 0.1% (0.0%–0.2%) | 0.5% (0%–1.5%) |
| Rifornimento | 1.4% (1.2%–1.6%) | 1.4% (1.1%–1.6%) | 0.9% (0.6%–1.3%) ◆ | 0.8% (0.3%–1.2%) ◆ |

### Lavoro ed efficienza

| Metrica | concentrazione γ=0 | concentrazione γ=0.5 | concentrazione γ=1 | concentrazione γ=2 |
|---|---|---|---|---|
| Equità del lavoro | 0.704 (0.632–0.777) | 0.715 (0.649–0.781) | 0.678 (0.559–0.796) | 0.572 (0.429–0.716) |
| Sovraffollamento sugli incendi [s] | 0 (0–0) | 0 (0–0) | 0 (0–0) | 0.0625 (0–0.2) |
| Attesa media in coda [s] | 0.184 (0–0.373) | 0.34 (0.0861–0.593) | 0.197 (0.00259–0.391) | 0.587 (0–1.51) |
| Attesa massima in coda [s] | 2.51 (0.575–4.45) | 2.87 (1.02–4.72) | 1.97 (0.608–3.34) | 2.33 (0–4.72) |
| Distanza percorsa [m] | 3 071 (3 031–3 111) | 2 893 (2 831–2 955) ▲ | 1 146 (958–1 334) ▲ | 558 (366–751) ▲ |
| Sforzo di controllo [m²/s³] | 2 648 (2 562–2 734) | 2 187 (2 127–2 248) ▲ | 1 124 (970–1 277) ▲ | 537 (353–721) ▲ |
| Rimbalzi da incendi pieni | 0.5 (0–1.42) | 0.25 (0–0.537) | 1.33 (0.6–2.07) | 2.17 (0.712–3.62) |

### Comunicazione e conoscenza

| Metrica | concentrazione γ=0 | concentrazione γ=0.5 | concentrazione γ=1 | concentrazione γ=2 |
|---|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 0.518 (0.454–0.582) | 0.533 (0.463–0.603) | 1.34 (1.17–1.51) ▲ | 1.91 (1.71–2.12) ▲ |
| Sciame tutto connesso | 0% (0%–0%) | 0% (0%–0%) | 0.9% (0%–2.3%) | 1.4% (0%–3.5%) |
| Incendi noti ai droni | 17% (14%–19%) | 19% (16%–21%) | 13% (6.5%–19%) | 8.3% (1.3%–15%) ▼ |
| Incendi fantasma per drone | 0.0144 (0.00848–0.0203) | 0.0182 (0.0102–0.0262) | 0.0156 (0–0.032) | 0.00649 (0–0.0159) |
| Età delle informazioni [s] | 0.964 (0.546–1.38) | 1.22 (0.667–1.78) | 0.714 (0.094–1.33) | 1.01 (0.0473–1.96) |

### Perlustrazione

| Metrica | concentrazione γ=0 | concentrazione γ=0.5 | concentrazione γ=1 | concentrazione γ=2 |
|---|---|---|---|---|
| Obsolescenza del terreno [s] | 27.4 (25–29.9) | 17.8 (16.3–19.3) ▲ | 45.9 (41–50.9) ▼ | 58.2 (49.9–66.6) ▼ |
| Obsolescenza delle zone importanti [s] | 26.9 (23.8–30.1) | 5.88 (5.21–6.55) ▲ | 5.85 (3.85–7.85) ▲ | 12.7 (8.4–17) ▲ |
| Obsolescenza del resto dell'area [s] | 28.1 (25.9–30.2) | 30.7 (28.2–33.3) | 94.6 (84.1–105) ▼ | 102 (88.3–115) ▼ |
| Obsolescenza della zona peggiore [s] | 138 (123–153) | 68 (58.1–77.9) ▲ | 101 (71.8–131) ▲ | 206 (154–258) ▼ |

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
