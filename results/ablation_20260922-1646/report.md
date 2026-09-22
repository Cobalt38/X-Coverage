# Esperimento «ablation»

**Domanda.** Quanto contribuisce ciascun meccanismo al successo e alla sicurezza dello sciame?

**Scenario «critico».** 5 incendi in posizioni casuali che crescono più in fretta (0.6 vita/s). Lo sciame riesce circa 2 volte su 3: è in questa zona che si vedono le differenze.
Parametri diversi dal default: NUM_FIRES = 5, FIRE_GROWTH_RATE = 0.6. Ogni simulazione dura al massimo 600 s ed è fermata (e conta come fallita) se gli incendi accesi superano 40.

**Varianti.** «completo» (nessuno); «senza predizione delle collisioni» (AVOIDANCE_MODE = emergency-only); «senza evitamento delle collisioni» (AVOIDANCE_MODE = none); «radio con 50% di messaggi persi» (PACKET_LOSS = 0.5); «senza rimbalzo dagli incendi pieni» (SATURATION_BOUNCE = False). Il riferimento è «completo».

**Metodo.** 8 simulazioni per variante, con i seed 1–8, uguali per tutte le varianti. Eseguito il 22/09/2026 alle 16:47 in 5.1 minuti (codice: commit 77da2b5 con modifiche non salvate).

## In breve

⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare significative. Per conclusioni affidabili usare almeno 20 simulazioni (--runs 20).

- **senza predizione delle collisioni**: tempo in emergenza 0.1% → 70% (peggiore); equità del lavoro 0.956 → 0.828 (peggiore); incendi noti ai droni 74% → 47% (peggiore). Cambiano in modo significativo anche altre 12 metriche (vedi tabelle).
- **senza evitamento delle collisioni**: collisioni 0 → 95.5 (peggiore). Cambiano in modo significativo anche altre 13 metriche (vedi tabelle).
- **radio con 50% di messaggi persi**: collisioni 0 → 2.5 (peggiore); tempo in emergenza 0.1% → 8.1% (peggiore); equità del lavoro 0.956 → 0.746 (peggiore); incendi noti ai droni 74% → 53% (peggiore). Cambiano in modo significativo anche altre 13 metriche (vedi tabelle).
- **senza rimbalzo dagli incendi pieni**: nessun cambiamento tra le metriche principali. Cambia in modo significativo anche un'altra metrica (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «completo»; ◆ = significativamente diverso (per metriche senza un valore "migliore").

### Missione

| Metrica | completo | senza predizione delle collisioni | senza evitamento delle collisioni | radio con 50% di messaggi persi | senza rimbalzo dagli incendi pieni |
|---|---|---|---|---|---|
| Missione riuscita | 62% (31%–86%) | 0% (0%–32%) | 88% (53%–98%) | 0% (0%–32%) | 62% (31%–86%) |
| Incendi fuori controllo | 38% (14%–69%) | 100% (68%–100%) | 0% (0%–32%) | 88% (53%–98%) | 38% (14%–69%) |
| Tempo per spegnere tutto [s] | 160 (124–195) | – | 127 (61–192) | – | 159 (123–195) |
| Danno degli incendi [vita·s] | 127 948 (58 740–197 157) | 210 806 (159 158–262 453) | 81 324 (18 867–143 781) | 200 474 (154 159–246 789) | 127 921 (58 444–197 399) |
| Incendi nati dalla propagazione | 17.8 (0–36.6) | 38.1 (37.2–39.1) | 7.62 (0–17.9) | 34.8 (22.9–46.6) | 18.4 (0–38) |
| Picco di incendi accesi | 18.9 (3.53–34.2) | 41 (41–41) | 8.12 (3.34–12.9) | 36.5 (25.9–47.1) | 18.9 (3.53–34.2) |
| Ritardo di avvistamento [s] | 9.39 (2.26–16.5) | 8.05 (0.472–15.6) | 6.46 (1.99–10.9) | 10 (0–21.5) | 9.52 (2.23–16.8) |
| Ritardo di intervento [s] | 6.68 (2.03–11.3) | 4.97 (1.48–8.46) | 10.2 (0–20.9) | 8.08 (4.1–12.1) | 7.54 (1.24–13.8) |

### Sicurezza

| Metrica | completo | senza predizione delle collisioni | senza evitamento delle collisioni | radio con 50% di messaggi persi | senza rimbalzo dagli incendi pieni |
|---|---|---|---|---|---|
| Collisioni | 0 (0–0) | 0.5 (0–1.13) | 95.5 (23.5–167) ▼ | 2.5 (0.0931–4.91) ▼ | 0 (0–0) |
| Quasi-collisioni | 1 (0–2.09) | 179 (142–216) ▼ | 396 (82.4–709) ▼ | 229 (65.2–394) ▼ | 1 (0–2.09) |
| Distanza minima tra due droni [m] | 0.728 (0.526–0.931) | 0.147 (0.0872–0.206) ▼ | 0.00163 (0–0.00334) ▼ | 0.0499 (0.0166–0.0831) ▼ | 0.729 (0.526–0.933) |
| Tempo in emergenza | 0.1% (0%–0.4%) | 70% (68%–72%) ▼ | 0% (0%–0%) | 8.1% (7.2%–8.9%) ▼ | 0.1% (0%–0.4%) |

### Uso del tempo

| Metrica | completo | senza predizione delle collisioni | senza evitamento delle collisioni | radio con 50% di messaggi persi | senza rimbalzo dagli incendi pieni |
|---|---|---|---|---|---|
| Esplorazione | 6.2% (2.3%–10%) | 2.6% (1.5%–3.6%) ◆ | 20% (9.7%–30%) ◆ | 4.2% (1.1%–7.3%) | 6.2% (2.2%–10%) |
| In volo verso un incendio | 31% (28%–34%) | 30% (25%–35%) | 25% (21%–28%) ◆ | 10% (6.4%–14%) ◆ | 31% (27%–35%) |
| Spegnimento | 13% (12%–15%) | 7.5% (7.1%–8.0%) ▼ | 18% (15%–22%) ▲ | 7.3% (5.8%–8.7%) ▼ | 13% (12%–15%) |
| In volo verso una stazione | 38% (36%–39%) | 47% (44%–50%) ▼ | 26% (23%–28%) ▲ | 58% (54%–63%) ▼ | 38% (36%–39%) |
| In coda alla stazione | 5.8% (3.3%–8.2%) | 9.5% (6.3%–13%) | 2.4% (1.3%–3.5%) ▲ | 16% (12%–21%) ▼ | 5.7% (3.3%–8.0%) |
| Rifornimento | 6.2% (5.6%–6.8%) | 3.4% (3.1%–3.8%) ◆ | 8.9% (7.2%–11%) ◆ | 3.4% (2.7%–4.0%) ◆ | 6.2% (5.6%–6.8%) |

### Lavoro ed efficienza

| Metrica | completo | senza predizione delle collisioni | senza evitamento delle collisioni | radio con 50% di messaggi persi | senza rimbalzo dagli incendi pieni |
|---|---|---|---|---|---|
| Equità del lavoro | 0.956 (0.921–0.99) | 0.828 (0.733–0.922) ▼ | 0.959 (0.938–0.981) | 0.746 (0.562–0.929) ▼ | 0.96 (0.936–0.984) |
| Sovraffollamento sugli incendi [s] | 0.135 (0–0.348) | 0 (0–0) | 8.76 (0–21.6) ▼ | 0 (0–0) | 0.255 (0–0.725) |
| Attesa media in coda [s] | 1.58 (0.948–2.21) | 5.25 (2.82–7.67) ▼ | 0.588 (0.319–0.857) ▲ | 5.73 (1.18–10.3) ▼ | 1.59 (1.02–2.17) |
| Attesa massima in coda [s] | 11.1 (7.24–14.9) | 52 (29.4–74.6) ▼ | 5.1 (3.2–7.01) ▲ | 38.4 (15.1–61.7) ▼ | 11.5 (7.96–15.1) |
| Distanza percorsa [m] | 958 (776–1 140) | 374 (299–450) ▲ | 1 450 (76.9–2 823) | 1 429 (585–2 273) | 958 (771–1 145) |
| Sforzo di controllo [m²/s³] | 1 037 (869–1 206) | 520 (405–634) ▲ | 582 (185–979) ▲ | 2 099 (872–3 326) ▼ | 1 036 (856–1 217) |
| Rimbalzi da incendi pieni | 1.25 (0.384–2.12) | 0.75 (0–1.62) | 3.12 (0.213–6.04) | 0.5 (0–1.13) | 0 (0–0) ◆ |

### Comunicazione e conoscenza

| Metrica | completo | senza predizione delle collisioni | senza evitamento delle collisioni | radio con 50% di messaggi persi | senza rimbalzo dagli incendi pieni |
|---|---|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) | 50% (50%–50%) ▼ | 0% (0%–0%) |
| Vicini radio per drone | 2.94 (2.56–3.31) | 3.53 (2.88–4.18) ▲ | 2.95 (2.5–3.41) | 1.75 (1.49–2.02) ▼ | 2.91 (2.56–3.26) |
| Sciame tutto connesso | 20% (10%–30%) | 9.8% (0%–20%) ▼ | 8.8% (0.3%–17%) ▼ | 11% (0%–26%) | 21% (8.9%–32%) |
| Incendi noti ai droni | 74% (60%–87%) | 47% (37%–58%) ▼ | 65% (53%–77%) | 53% (35%–72%) ▼ | 73% (59%–87%) |
| Incendi fantasma per drone | 0.05 (0.0172–0.0827) | 0.0288 (0–0.0583) | 0.0695 (0.0293–0.11) | 0.056 (0–0.116) | 0.0413 (0.0265–0.056) |
| Età delle informazioni [s] | 0.324 (0.123–0.525) | 0.482 (0.212–0.751) | 0.214 (0.0658–0.361) | 0.724 (0.358–1.09) ▼ | 0.287 (0.0816–0.491) |

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

I dati di ogni singola simulazione sono in `runs.csv` (una riga per simulazione, colonne = nomi tra parentesi qui sopra).
