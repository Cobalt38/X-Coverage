# Esperimento «urti»

**Domanda.** Quanto costa un urto allo sciame, e quanti droni può perdere prima di non farcela più?

**Scenario «critico».** Cinque incendi in posizioni casuali che crescono più in fretta del normale (0.6 vita/s). Lo sciame riesce circa due volte su tre: è in questa zona di difficoltà che le differenze tra varianti si vedono.
Parametri diversi dal default: NUM_FIRES = 5, FIRE_GROWTH_RATE = 0.6. Ogni simulazione dura al massimo 600 s e viene fermata (contando come fallita) se gli incendi accesi superano 40.

**Varianti.** «urti innocui» (COLLISION_DAMAGE = False); «urti con danni» (nessuno); «urti con danni, senza evitamento» (AVOIDANCE_MODE = none); «urti sempre fatali» (COLLISION_TOTAL_LOSS_SPEED = 0.0). Il riferimento è «urti innocui».

**Metodo.** 12 simulazioni per variante, con i seed da 1 a 12, gli stessi per tutte le varianti: ogni variante affronta quindi esattamente le stesse situazioni di partenza. Eseguito il 23/09/2026 alle 13:56 in 3.4 minuti (codice: commit fb98452 con modifiche non salvate).

## In breve

⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare significative. Per conclusioni affidabili servono almeno 20 simulazioni (--runs 20).

- **urti con danni**: nessuna differenza significativa rispetto a «urti innocui».
- **urti con danni, senza evitamento**: nessuna differenza significativa rispetto a «urti innocui».
- **urti sempre fatali**: nessuna differenza significativa rispetto a «urti innocui».

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «urti innocui»; ◆ = significativamente diverso, per le misure senza un verso migliore.

### Missione

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Missione riuscita | 92% (65%–99%) | 92% (65%–99%) | 8.3% (1.5%–35%) | 92% (65%–99%) |
| Incendi fuori controllo | 8.3% (1.5%–35%) | 8.3% (1.5%–35%) | 25% (8.9%–53%) | 8.3% (1.5%–35%) |
| Tempo per spegnere tutto [s] | 142 (126–159) [su 11] | 142 (126–159) [su 11] | 394 [su 1] | 142 (126–159) [su 11] |
| Danno degli incendi [vita·s] | 90 134 (28 932–151 336) | 90 134 (28 932–151 336) | 119 282 (58 234–180 329) | 90 134 (28 932–151 336) |
| Fuoco acceso in media [vita] | 523 (333–713) | 523 (333–713) | 804 (635–973) | 523 (333–713) |
| Incendi nati dalla propagazione | 5.17 (0–16) | 5.17 (0–16) | 13.8 (2.59–25.1) | 5.17 (0–16) |
| Incendi nati da soli | 0 (0–0) | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Picco di incendi accesi | 8.08 (1.49–14.7) | 8.08 (1.49–14.7) | 17.8 (7.38–28.3) | 8.08 (1.49–14.7) |
| Ritardo di avvistamento [s] | 4.62 (0.963–8.27) | 4.62 (0.963–8.27) | 3.51 (1.69–5.32) | 4.62 (0.963–8.27) |
| Incendi mai avvistati | 0 (0–0) | 0 (0–0) | 12.4 (2.34–22.5) | 0 (0–0) |
| Ritardo di intervento [s] | 4.37 (0–9.15) | 4.37 (0–9.15) | 2.33 (0.894–3.77) | 4.37 (0–9.15) |

### Sicurezza

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Urti | 0 (0–0) | 0 (0–0) | 5.67 (5.35–5.98) | 0 (0–0) |
| Droni fuori uso | 0 (0–0) | 0 (0–0) | 11.3 (10.7–12) | 0 (0–0) |
| Quasi-urti | 0.417 (0–0.841) | 0.417 (0–0.841) | 28.2 (21.6–34.8) | 0.417 (0–0.841) |
| Distanza minima tra due droni [m] | 0.803 (0.642–0.963) | 0.803 (0.642–0.963) | 0.0913 (0.0882–0.0944) | 0.803 (0.642–0.963) |
| Tempo in emergenza | 0.0% (0%–0.1%) | 0.0% (0%–0.1%) | 0% (0%–0%) | 0.0% (0%–0.1%) |

### Uso del tempo

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Perlustrazione | 4.5% (3.4%–5.7%) | 4.5% (3.4%–5.7%) | 5.0% (1.8%–8.1%) | 4.5% (3.4%–5.7%) |
| In volo verso un incendio | 33% (31%–35%) | 33% (31%–35%) | 7.7% (6.3%–9.2%) | 33% (31%–35%) |
| Spegnimento | 15% (14%–16%) | 15% (14%–16%) | 11% (7.8%–13%) | 15% (14%–16%) |
| In volo verso una stazione | 21% (18%–24%) | 21% (18%–24%) | 4.2% (3.2%–5.3%) | 21% (18%–24%) |
| In coda alla stazione | 20% (17%–22%) | 20% (17%–22%) | 5.3% (3.8%–6.7%) | 20% (17%–22%) |
| Rifornimento | 7.0% (6.5%–7.5%) | 7.0% (6.5%–7.5%) | 4.5% (3.3%–5.7%) | 7.0% (6.5%–7.5%) |
| Fuori uso | 0% (0%–0%) | 0% (0%–0%) | 63% (54%–72%) | 0% (0%–0%) |

### Lavoro ed efficienza

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Equità del lavoro | 0.944 (0.896–0.991) | 0.944 (0.896–0.991) | 0.986 (0.96–1.01) [su 4] | 0.944 (0.896–0.991) |
| Sovraffollamento sugli incendi [s] | 0 (0–0) | 0 (0–0) | 0.0508 (0–0.111) | 0 (0–0) |
| Attesa media in coda [s] | 5.79 (5.12–6.46) | 5.79 (5.12–6.46) | 2.53 (2.32–2.73) | 5.79 (5.12–6.46) |
| Attesa massima in coda [s] | 19 (15.2–22.9) | 19 (15.2–22.9) | 4.73 (3.61–5.86) | 19 (15.2–22.9) |
| Distanza per drone [m] | 71.6 (56.7–86.4) | 71.6 (56.7–86.4) | 72 (40–104) | 71.6 (56.7–86.4) |
| Sforzo di controllo [m²/s³] | 0.492 (0.469–0.514) | 0.492 (0.469–0.514) | 0.261 (0.251–0.271) | 0.492 (0.469–0.514) |
| Rimbalzi da incendi affollati | 2 (1.23–2.77) | 2 (1.23–2.77) | 0.583 (0.0113–1.16) | 2 (1.23–2.77) |

### Comunicazione e conoscenza

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 2.68 (2.49–2.87) | 2.68 (2.49–2.87) | 0.954 (0.601–1.31) | 2.68 (2.49–2.87) |
| Sciame tutto connesso | 10% (7.4%–13%) | 10% (7.4%–13%) | 0% (0%–0%) | 10% (7.4%–13%) |
| Incendi noti ai droni | 84% (80%–89%) | 84% (80%–89%) | 43% (31%–54%) | 84% (80%–89%) |
| Incendi fantasma per drone | 0.101 (0.0284–0.174) | 0.101 (0.0284–0.174) | 0.0299 (0–0.0647) | 0.101 (0.0284–0.174) |
| Età delle informazioni [s] | 0.343 (0.139–0.546) | 0.343 (0.139–0.546) | 1.15 (0.633–1.66) | 0.343 (0.139–0.546) |

### Perlustrazione

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Obsolescenza del terreno [s] | 27.1 (19.2–35.1) | 27.1 (19.2–35.1) | 45.1 (23.6–66.5) | 27.1 (19.2–35.1) |
| Obsolescenza delle zone importanti [s] | 24.7 (14.8–34.6) | 24.7 (14.8–34.6) | 42.7 (21.3–64) | 24.7 (14.8–34.6) |
| Obsolescenza del resto dell'area [s] | 31.8 (27.1–36.6) | 31.8 (27.1–36.6) | 50.8 (26.8–74.8) | 31.8 (27.1–36.6) |
| Obsolescenza della zona peggiore [s] | 152 (125–179) | 152 (125–179) | 151 (81.3–221) | 152 (125–179) |

## Come leggere questi numeri

- **Intervallo di confidenza al 95%**: ripetendo l'esperimento all'infinito, la media vera cadrebbe quasi sempre dentro quell'intervallo. Se è largo, servono più simulazioni.
- **▲ ▼ ◆**: la differenza rispetto al riferimento supera il test statistico (probabilità che sia fortuna sotto 5%). Le simulazioni sono confrontate a coppie con lo stesso seed, e le probabilità sono corrette con il metodo di Holm per il numero di confronti fatti in questo report: senza quella correzione, una decina di marcatori sarebbero falsi allarmi dovuti al solo numero di test. Attenzione: "reale" non vuol dire "grande" — guardare anche di quanto cambia la media.
- **[su N]** accanto a un valore: quella misura non era definita in tutte le simulazioni (per esempio il tempo di spegnimento esiste solo per le missioni riuscite), quindi media e intervallo si basano su N simulazioni invece che su tutte.
- **Tempo per spegnere tutto** è calcolato solo sulle missioni riuscite: va letto insieme a «Missione riuscita», altrimenti una variante che fallisce spesso sembra veloce.
- Le simulazioni finiscono in momenti diversi (chi riesce prima si ferma prima), quindi le misure che si accumulano nel tempo — danno, distanza, sforzo — vanno confrontate con prudenza tra varianti con esiti molto diversi.

## Che cosa significa ogni misura

- **Missione riuscita** (`mission_complete`): Tutti gli incendi sono stati spenti entro il tempo massimo.
- **Incendi fuori controllo** (`fire_overrun`): La simulazione è stata interrotta perché gli incendi accesi erano troppi.
- **Tempo per spegnere tutto** (`extinction_time_s`): Secondi fino all'ultimo incendio spento. Ha senso solo per le missioni riuscite.
- **Danno degli incendi** (`fire_damage`): Somma, istante per istante, della vita di tutti gli incendi accesi. ATTENZIONE: cresce con la durata, e una missione che collassa presto si ferma prima. Per confrontare varianti usare la misura seguente, che è già divisa per il tempo.
- **Fuoco acceso in media** (`burning_health_mean`): Quanta vita di incendi era accesa in media, istante per istante: il danno diviso per la durata. Confrontabile tra missioni di durata diversa.
- **Incendi nati dalla propagazione** (`fires_spawned`): Un incendio lasciato crescere troppo ne genera altri vicino a sé.
- **Incendi nati da soli** (`fires_ignited`): Accensioni spontanee, indipendenti dagli incendi già presenti. Dipendono solo dal seed, quindi a parità di seed sono le stesse per tutte le varianti.
- **Picco di incendi accesi** (`peak_active_fires`): Il massimo numero di incendi accesi nello stesso momento.
- **Ritardo di avvistamento** (`detection_delay_s`): Tempo medio tra la nascita di un incendio e il primo drone che lo vede. Va letto insieme alla misura seguente: una strategia che ignora del tutto certe zone MIGLIORA questa media, perché gli incendi che non trova mai non entrano nel conto.
- **Incendi mai avvistati** (`fires_never_seen`): Incendi ancora accesi alla fine che nessun drone ha mai visto.
- **Ritardo di intervento** (`response_delay_s`): Tempo medio tra l'avvistamento di un incendio e la prima acqua che riceve.
- **Urti** (`collisions`): Quante volte due droni si sono toccati (distanza sotto DRONE_IMPACT_RADIUS).
- **Droni fuori uso** (`drones_lost`): Droni precipitati in seguito a un urto: non volano più per il resto della missione.
- **Quasi-urti** (`near_misses`): Quante volte due droni sono scesi sotto la distanza di emergenza senza toccarsi.
- **Distanza minima tra due droni** (`min_distance_m`): La distanza più piccola mai registrata tra due droni in volo.
- **Tempo in emergenza** (`emergency_fraction`): Quota di tempo passata a scansarsi invece che a lavorare.
- **Perlustrazione** (`time_patrol`): Nessun compito: il drone sta cercando incendi.
- **In volo verso un incendio** (`time_to_fire`): Ha scelto un incendio e ci sta andando.
- **Spegnimento** (`time_extinguishing`): Sta spruzzando acqua su un incendio: è l'unico momento in cui fa il suo mestiere.
- **In volo verso una stazione** (`time_to_station`): Acqua quasi finita: sta andando a rifornirsi.
- **In coda alla stazione** (`time_queue`): È arrivato alla stazione e aspetta che si liberi un posto.
- **Rifornimento** (`time_refill`): Sta caricando acqua.
- **Fuori uso** (`time_out_of_service`): Tempo passato a terra dopo un urto.
- **Equità del lavoro** (`water_fairness`): Quanto equamente i droni si dividono il lavoro (indice di Jain sull'acqua erogata): 1 = tutti uguali, verso 0 = pochi fanno tutto.
- **Sovraffollamento sugli incendi** (`overcrowding_s`): Secondi in cui su un incendio lavoravano più droni del limite MAX_DRONES_ON_FIRE.
- **Attesa media in coda** (`queue_wait_mean_s`): Quanto si aspetta, in media, prima di poter caricare acqua.
- **Attesa massima in coda** (`queue_wait_max_s`): L'attesa più lunga registrata a una stazione.
- **Distanza per drone** (`distance_per_drone_m`): Metri percorsi in media da ogni drone ancora in volo, al netto della durata (metri per drone al secondo × durata). Non ha un verso migliore: volare di più non è né buono né cattivo di per sé.
- **Sforzo di controllo** (`control_effort_rate`): Quanto ogni drone accelera e frena, al secondo: approssima il consumo di batteria. Diviso per i secondi di volo effettivi, quindi confrontabile tra missioni diverse.
- **Rimbalzi da incendi affollati** (`saturation_bounces`): Quante volte un drone si è allontanato da un incendio dove era di troppo.
- **Messaggi persi** (`message_loss`): Quota dei messaggi radio che non sono arrivati a destinazione.
- **Vicini radio per drone** (`neighbors`): Quanti altri droni sente in media ciascuno.
- **Sciame tutto connesso** (`connected_fraction`): Quota di tempo in cui ogni drone può raggiungere ogni altro, anche passando per altri.
- **Incendi noti ai droni** (`fire_awareness`): In media, quale quota degli incendi accesi conosce ciascun drone.
- **Incendi fantasma per drone** (`phantom_fires`): Incendi che un drone crede accesi ma che sono già stati spenti.
- **Età delle informazioni** (`info_age_s`): Quanto sono vecchie, in media, le notizie che i droni hanno sugli incendi accesi.
- **Obsolescenza del terreno** (`coverage_staleness_s`): Da quanto tempo, in media, una zona non viene guardata da nessuno, pesando ogni zona per la sua importanza. È l'obiettivo classico della copertura persistente.
- **Obsolescenza delle zone importanti** (`coverage_staleness_hot_s`): Come sopra, ma solo sulle zone di valore alto (importanza ≥ 0.5).
- **Obsolescenza del resto dell'area** (`coverage_staleness_cold_s`): Come sopra, sulle zone di valore basso. Insieme alla precedente mostra come una strategia distribuisce l'attenzione: concentrarsi sulle zone importanti peggiora questa.
- **Obsolescenza della zona peggiore** (`coverage_staleness_max_s`): La zona importante lasciata più a lungo senza controllo.

I dati di ogni singola simulazione sono in `runs.csv`: una riga per simulazione, e come nomi di colonna quelli tra parentesi qui sopra.
