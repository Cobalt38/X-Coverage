# Esperimento «urti»

**Domanda.** Quanto costa un urto allo sciame, e quanti droni può perdere prima di non farcela più?

**Scenario «critico».** Cinque incendi in posizioni casuali che crescono più in fretta del normale (0.6 vita/s). Lo sciame riesce circa due volte su tre: è in questa zona di difficoltà che le differenze tra varianti si vedono.
Parametri diversi dal default: NUM_FIRES = 5, FIRE_GROWTH_RATE = 0.6. Ogni simulazione dura al massimo 600 s e viene fermata (contando come fallita) se gli incendi accesi superano 40.

**Varianti.** «urti innocui» (COLLISION_DAMAGE = False); «urti con danni» (nessuno); «urti con danni, senza evitamento» (AVOIDANCE_MODE = none); «urti sempre fatali» (COLLISION_TOTAL_LOSS_SPEED = 0.0). Il riferimento è «urti innocui».

**Metodo.** 12 simulazioni per variante, con i seed da 1 a 12, gli stessi per tutte le varianti: ogni variante affronta quindi esattamente le stesse situazioni di partenza. Eseguito il 23/09/2026 alle 11:36 in 5.2 minuti (codice: commit 9b738fc con modifiche non salvate).

## In breve

⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare significative. Per conclusioni affidabili servono almeno 20 simulazioni (--runs 20).

- **urti con danni**: nessuna differenza significativa rispetto a «urti innocui».
- **urti con danni, senza evitamento**: missione riuscita 75% → 0% (peggiore); urti 0 → 5.42 (peggiore); droni fuori uso 0 → 10.8 (peggiore). Cambiano in modo significativo anche altre 22 misure (vedi tabelle).
- **urti sempre fatali**: nessuna differenza significativa rispetto a «urti innocui».

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «urti innocui»; ◆ = significativamente diverso, per le misure senza un verso migliore.

### Missione

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Missione riuscita | 75% (47%–91%) | 75% (47%–91%) | 0% (0%–24%) ▼ | 75% (47%–91%) |
| Incendi fuori controllo | 25% (8.9%–53%) | 25% (8.9%–53%) | 58% (32%–81%) | 25% (8.9%–53%) |
| Tempo per spegnere tutto [s] | 147 (124–171) | 147 (124–171) | – | 147 (124–171) |
| Danno degli incendi [vita·s] | 105 457 (58 307–152 607) | 105 457 (58 307–152 607) | 146 689 (93 280–200 098) | 105 457 (58 307–152 607) |
| Incendi nati dalla propagazione | 11.9 (0–24.6) | 11.9 (0–24.6) | 26.2 (14.9–37.4) | 11.9 (0–24.6) |
| Incendi nati da soli | 0 (0–0) | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Picco di incendi accesi | 14.2 (3.99–24.5) | 14.2 (3.99–24.5) | 29.2 (18.9–39.6) | 14.2 (3.99–24.5) |
| Ritardo di avvistamento [s] | 6.65 (1.62–11.7) | 6.65 (1.62–11.7) | 4.21 (1.02–7.4) | 6.65 (1.62–11.7) |
| Incendi mai avvistati | 2.92 (0–6.68) | 2.92 (0–6.68) | 21.9 (12.6–31.3) ▼ | 2.92 (0–6.68) |
| Ritardo di intervento [s] | 5.06 (1.84–8.29) | 5.06 (1.84–8.29) | 3.51 (1.77–5.25) | 5.06 (1.84–8.29) |

### Sicurezza

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Urti | 0 (0–0) | 0 (0–0) | 5.42 (5.09–5.74) ▼ | 0 (0–0) |
| Droni fuori uso | 0 (0–0) | 0 (0–0) | 10.8 (10.2–11.5) ▼ | 0 (0–0) |
| Quasi-urti | 0.75 (0.0268–1.47) | 0.75 (0.0268–1.47) | 29.1 (23.3–34.9) ▼ | 0.75 (0.0268–1.47) |
| Distanza minima tra due droni [m] | 0.795 (0.648–0.941) | 0.795 (0.648–0.941) | 0.0882 (0.0846–0.0919) ▼ | 0.795 (0.648–0.941) |
| Tempo in emergenza | 0.1% (0%–0.2%) | 0.1% (0%–0.2%) | 0% (0%–0%) ▲ | 0.1% (0%–0.2%) |

### Uso del tempo

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Perlustrazione | 5.3% (2.7%–7.8%) | 5.3% (2.7%–7.8%) | 3.3% (2.3%–4.4%) | 5.3% (2.7%–7.8%) |
| In volo verso un incendio | 32% (29%–34%) | 32% (29%–34%) | 7.0% (5.0%–8.9%) ◆ | 32% (29%–34%) |
| Spegnimento | 14% (13%–16%) | 14% (13%–16%) | 9.5% (7.5%–11%) ▼ | 14% (13%–16%) |
| In volo verso una stazione | 22% (20%–24%) | 22% (20%–24%) | 3.3% (2.1%–4.6%) ▲ | 22% (20%–24%) |
| In coda alla stazione | 20% (17%–23%) | 20% (17%–23%) | 4.8% (3.9%–5.6%) ▲ | 20% (17%–23%) |
| Rifornimento | 6.7% (6.0%–7.4%) | 6.7% (6.0%–7.4%) | 4.2% (3.4%–5.1%) ◆ | 6.7% (6.0%–7.4%) |
| Fuori uso | 0% (0%–0%) | 0% (0%–0%) | 68% (61%–75%) ▼ | 0% (0%–0%) |

### Lavoro ed efficienza

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Equità del lavoro | 0.961 (0.939–0.983) | 0.961 (0.939–0.983) | 0.422 (0.331–0.513) ▼ | 0.961 (0.939–0.983) |
| Sovraffollamento sugli incendi [s] | 0.0917 (0–0.229) | 0.0917 (0–0.229) | 0.194 (0–0.469) | 0.0917 (0–0.229) |
| Attesa media in coda [s] | 6.11 (5.42–6.81) | 6.11 (5.42–6.81) | 2.36 (2.18–2.53) ▲ | 6.11 (5.42–6.81) |
| Attesa massima in coda [s] | 21.8 (17.2–26.3) | 21.8 (17.2–26.3) | 4.96 (3.61–6.32) ▲ | 21.8 (17.2–26.3) |
| Distanza percorsa [m] | 881 (732–1 030) | 881 (732–1 030) | 226 (193–260) ▲ | 881 (732–1 030) |
| Sforzo di controllo [m²/s³] | 954 (804–1 103) | 954 (804–1 103) | 125 (109–141) ▲ | 954 (804–1 103) |
| Rimbalzi da incendi affollati | 1.42 (0.728–2.11) | 1.42 (0.728–2.11) | 0.667 (0.103–1.23) | 1.42 (0.728–2.11) |

### Comunicazione e conoscenza

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 2.8 (2.5–3.09) | 2.8 (2.5–3.09) | 2.12 (1.8–2.44) ▼ | 2.8 (2.5–3.09) |
| Sciame tutto connesso | 16% (8.3%–23%) | 16% (8.3%–23%) | 0% (0%–0%) ▼ | 16% (8.3%–23%) |
| Incendi noti ai droni | 78% (69%–87%) | 78% (69%–87%) | 38% (26%–50%) ▼ | 78% (69%–87%) |
| Incendi fantasma per drone | 0.0778 (0.0385–0.117) | 0.0778 (0.0385–0.117) | 0.299 (0.134–0.464) ▼ | 0.0778 (0.0385–0.117) |
| Età delle informazioni [s] | 0.391 (0.172–0.609) | 0.391 (0.172–0.609) | 0.613 (0.366–0.86) | 0.391 (0.172–0.609) |

### Perlustrazione

| Misura | urti innocui | urti con danni | urti con danni, senza evitamento | urti sempre fatali |
|---|---|---|---|---|
| Obsolescenza del terreno [s] | 27.5 (23.2–31.7) | 27.5 (23.2–31.7) | 47.2 (35.6–58.9) ▼ | 27.5 (23.2–31.7) |
| Obsolescenza delle zone importanti [s] | 25.1 (20–30.3) | 25.1 (20–30.3) | 47 (35.4–58.6) ▼ | 25.1 (20–30.3) |
| Obsolescenza del resto dell'area [s] | 32.2 (28.5–35.9) | 32.2 (28.5–35.9) | 48.6 (35.3–62) ▼ | 32.2 (28.5–35.9) |
| Obsolescenza della zona peggiore [s] | 159 (137–182) | 159 (137–182) | 142 (109–176) | 159 (137–182) |

## Come leggere questi numeri

- **Intervallo di confidenza al 95%**: ripetendo l'esperimento all'infinito, la media vera cadrebbe quasi sempre dentro quell'intervallo. Se è largo, servono più simulazioni.
- **▲ ▼ ◆**: la differenza rispetto al riferimento supera il test statistico (probabilità che sia fortuna sotto 5%). Le simulazioni sono confrontate a coppie con lo stesso seed. Attenzione: "reale" non vuol dire "grande" — guardare anche di quanto cambia la media.
- **Tempo per spegnere tutto** è calcolato solo sulle missioni riuscite: va letto insieme a «Missione riuscita», altrimenti una variante che fallisce spesso sembra veloce.
- Le simulazioni finiscono in momenti diversi (chi riesce prima si ferma prima), quindi le misure che si accumulano nel tempo — danno, distanza, sforzo — vanno confrontate con prudenza tra varianti con esiti molto diversi.

## Che cosa significa ogni misura

- **Missione riuscita** (`mission_complete`): Tutti gli incendi sono stati spenti entro il tempo massimo.
- **Incendi fuori controllo** (`fire_overrun`): La simulazione è stata interrotta perché gli incendi accesi erano troppi.
- **Tempo per spegnere tutto** (`extinction_time_s`): Secondi fino all'ultimo incendio spento. Ha senso solo per le missioni riuscite.
- **Danno degli incendi** (`fire_damage`): Somma, istante per istante, della vita di tutti gli incendi accesi: premia chi spegne presto e penalizza chi lascia bruciare.
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
- **Distanza percorsa** (`distance_m`): Metri percorsi da tutti i droni insieme.
- **Sforzo di controllo** (`control_effort`): Quanto i droni hanno accelerato e frenato: approssima il consumo di batteria.
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
