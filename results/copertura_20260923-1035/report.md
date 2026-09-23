# Esperimento «copertura»

**Domanda.** Perlustrare dove non si guarda da più tempo, e dove conta di più, fa trovare prima gli incendi?

**Scenario «ricerca».** Area quattro volte più grande (40x24 m), un solo incendio iniziale e accensioni spontanee distribuite secondo la mappa di importanza (una ogni ~33 s). Qui il problema non è l'acqua ma TROVARE gli incendi: i droni passano l'80% del tempo a perlustrare, contro il 6% dello scenario 'critico'.
Parametri diversi dal default: AREA_WIDTH = 40.0, AREA_HEIGHT = 24.0, NUM_FIRES = 1, IGNITION_RATE_PER_S = 0.03. Ogni simulazione dura al massimo 300 s ed è fermata (e conta come fallita) se gli incendi accesi superano 40.

**Varianti.** «ricerca casuale» (nessuno); «copertura persistente» (EXPLORATION_MODE = coverage); «copertura senza importanza» (EXPLORATION_MODE = coverage, COVERAGE_USE_IMPORTANCE = False). Il riferimento è «ricerca casuale».

**Metodo.** 12 simulazioni per variante, con i seed 1–12, uguali per tutte le varianti. Eseguito il 23/09/2026 alle 10:35 in 5.8 minuti (codice: commit 9b738fc con modifiche non salvate).

## In breve

⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare significative. Per conclusioni affidabili usare almeno 20 simulazioni (--runs 20).

- **copertura persistente**: nessun cambiamento tra le metriche principali. Cambiano in modo significativo anche altre 10 metriche (vedi tabelle).
- **copertura senza importanza**: nessun cambiamento tra le metriche principali. Cambiano in modo significativo anche altre 6 metriche (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «ricerca casuale»; ◆ = significativamente diverso (per metriche senza un valore "migliore").

### Missione

| Metrica | ricerca casuale | copertura persistente | copertura senza importanza |
|---|---|---|---|
| Missione riuscita | 83% (55%–95%) | 58% (32%–81%) | 92% (65%–99%) |
| Incendi fuori controllo | 8.3% (1.5%–35%) | 33% (14%–61%) | 0% (0%–24%) |
| Tempo per spegnere tutto [s] | 141 (78–203) | 81.2 (56.2–106) | 146 (92.5–200) |
| Danno degli incendi [vita·s] | 37 510 (5 883–69 137) | 78 867 (27 856–129 877) | 24 007 (19 443–28 570) |
| Incendi nati dalla propagazione | 3.58 (0–11.5) | 13.5 (0.788–26.2) | 0 (0–0) |
| Incendi nati da soli | 6.5 (4.8–8.2) | 6 (3.97–8.03) | 6.83 (5.1–8.56) |
| Picco di incendi accesi | 6.17 (0–13.2) | 15.6 (3.65–27.5) | 3.25 (2.64–3.86) |
| Ritardo di avvistamento [s] | 23.3 (17.3–29.4) | 4.85 (2.83–6.87) ▲ | 20.1 (14.4–25.9) |
| Ritardo di intervento [s] | 2.11 (1.22–3.01) | 3.06 (0–7.54) | 2.07 (1.54–2.59) |

### Sicurezza

| Metrica | ricerca casuale | copertura persistente | copertura senza importanza |
|---|---|---|---|
| Collisioni | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Quasi-collisioni | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Distanza minima tra due droni [m] | 1.03 (0.998–1.07) | 1.04 (0.966–1.11) | 1.04 (1–1.08) |
| Tempo in emergenza | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |

### Uso del tempo

| Metrica | ricerca casuale | copertura persistente | copertura senza importanza |
|---|---|---|---|
| Esplorazione | 80% (75%–84%) | 84% (78%–90%) | 80% (76%–84%) |
| In volo verso un incendio | 7.5% (5.8%–9.2%) | 6.6% (4.0%–9.1%) | 7.4% (5.8%–8.9%) |
| Spegnimento | 3.2% (2.6%–3.7%) | 2.2% (1.4%–2.9%) ▼ | 3.0% (2.6%–3.4%) |
| In volo verso una stazione | 8.2% (6.4%–10%) | 6.1% (3.2%–9.1%) ▲ | 8.0% (6.4%–9.7%) |
| In coda alla stazione | 0.2% (0.0%–0.3%) | 0.1% (0.0%–0.2%) | 0.1% (0%–0.3%) |
| Rifornimento | 1.4% (1.2%–1.6%) | 0.9% (0.6%–1.3%) ◆ | 1.4% (1.2%–1.6%) |

### Lavoro ed efficienza

| Metrica | ricerca casuale | copertura persistente | copertura senza importanza |
|---|---|---|---|
| Equità del lavoro | 0.688 (0.613–0.763) | 0.678 (0.559–0.796) | 0.704 (0.632–0.777) |
| Sovraffollamento sugli incendi [s] | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Attesa media in coda [s] | 0.217 (0.0822–0.352) | 0.197 (0.00259–0.391) | 0.184 (0–0.373) |
| Attesa massima in coda [s] | 2.83 (1.14–4.53) | 1.97 (0.608–3.34) | 2.51 (0.575–4.45) |
| Distanza percorsa [m] | 2 929 (2 762–3 097) | 1 146 (958–1 334) ▲ | 3 071 (3 031–3 111) ▼ |
| Sforzo di controllo [m²/s³] | 1 012 (939–1 085) | 1 124 (970–1 277) | 2 648 (2 562–2 734) ▼ |
| Rimbalzi da incendi pieni | 0.583 (0.159–1.01) | 1.33 (0.6–2.07) | 0.5 (0–1.42) |

### Comunicazione e conoscenza

| Metrica | ricerca casuale | copertura persistente | copertura senza importanza |
|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 0.596 (0.511–0.682) | 1.34 (1.17–1.51) ▲ | 0.518 (0.454–0.582) ▼ |
| Sciame tutto connesso | 0.2% (0%–0.6%) | 0.9% (0%–2.3%) | 0% (0%–0%) |
| Incendi noti ai droni | 15% (12%–18%) | 13% (6.5%–19%) | 17% (14%–19%) |
| Incendi fantasma per drone | 0.012 (0.00784–0.0162) | 0.0156 (0–0.032) | 0.0144 (0.00848–0.0203) |
| Età delle informazioni [s] | 0.945 (0.49–1.4) | 0.714 (0.094–1.33) | 0.964 (0.546–1.38) |

### Perlustrazione

| Metrica | ricerca casuale | copertura persistente | copertura senza importanza |
|---|---|---|---|
| Obsolescenza del terreno [s] | 30.9 (27.3–34.6) | 45.9 (41–50.9) ▼ | 27.4 (25–29.9) |
| Obsolescenza delle zone importanti [s] | 21.7 (17.3–26.2) | 5.85 (3.85–7.85) ▲ | 26.9 (23.8–30.1) ▼ |
| Obsolescenza del resto dell'area [s] | 39.5 (35.7–43.3) | 94.6 (84.1–105) ▼ | 28.1 (25.9–30.2) ▲ |
| Obsolescenza della zona peggiore [s] | 190 (151–229) | 101 (71.8–131) ▲ | 138 (123–153) ▲ |

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
- **Ritardo di avvistamento** (`detection_delay_s`): Tempo medio tra la nascita di un incendio e il primo drone che lo vede.
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
