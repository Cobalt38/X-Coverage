# Esperimento «copertura»

**Domanda.** Perlustrare dove non si guarda da più tempo fa trovare prima gli incendi?

**Scenario «ricerca».** Area quattro volte più grande (40 × 24 m), un solo incendio iniziale e accensioni spontanee distribuite secondo l'importanza del terreno (una ogni ~33 s). Qui il problema non è l'acqua ma TROVARE gli incendi: i droni passano l'80% del tempo a perlustrare, contro il 6% dello scenario 'critico'.
Parametri diversi dal default: AREA_WIDTH = 40.0, AREA_HEIGHT = 24.0, NUM_FIRES = 1, IGNITION_RATE_PER_S = 0.03. Ogni simulazione dura al massimo 300 s e viene fermata (contando come fallita) se gli incendi accesi superano 40.

**Varianti.** «ricerca casuale» (nessuno); «copertura persistente» (EXPLORATION_MODE = coverage); «copertura uniforme» (EXPLORATION_MODE = coverage, COVERAGE_IMPORTANCE_EXPONENT = 0.0). Il riferimento è «ricerca casuale».

**Metodo.** 12 simulazioni per variante, con i seed da 1 a 12, gli stessi per tutte le varianti: ogni variante affronta quindi esattamente le stesse situazioni di partenza. Eseguito il 23/09/2026 alle 13:53 in 4.1 minuti (codice: commit fb98452 con modifiche non salvate).

## In breve

⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare significative. Per conclusioni affidabili servono almeno 20 simulazioni (--runs 20).

- **copertura persistente**: nessun cambiamento tra le misure principali. Cambiano in modo significativo anche altre 4 misure (vedi tabelle).
- **copertura uniforme**: nessun cambiamento tra le misure principali. Cambiano in modo significativo anche altre 2 misure (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «ricerca casuale»; ◆ = significativamente diverso, per le misure senza un verso migliore.

### Missione

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Missione riuscita | – [su 0] | – [su 0] | – [su 0] |
| Incendi fuori controllo | 17% (4.7%–45%) | 8.3% (1.5%–35%) | 0% (0%–24%) |
| Tempo per spegnere tutto [s] | – [su 0] | – [su 0] | – [su 0] |
| Danno degli incendi [vita·s] | 48 394 (1 741–95 046) | 36 580 (13 463–59 698) | 30 347 (21 072–39 623) |
| Fuoco acceso in media [vita] | 207 (0–428) | 148 (14.9–282) | 101 (70.2–132) |
| Incendi nati dalla propagazione | 7 (0–17.4) | 3.17 (0–10.1) | 0 (0–0) |
| Incendi nati da soli | 6.25 (4.62–7.88) | 6.67 (4.9–8.43) | 6.83 (5.1–8.56) |
| Picco di incendi accesi | 9.17 (0–18.6) | 6.42 (0–13.4) | 3.33 (2.65–4.02) |
| Ritardo di avvistamento [s] | 16.8 (9.27–24.3) | 23.2 (15.6–30.9) | 26.1 (18.6–33.6) |
| Incendi mai avvistati | 4.58 (0–10.3) | 4.33 (0–11.7) | 1.33 (0.551–2.12) |
| Ritardo di intervento [s] | 2.36 (0.914–3.81) | 1.72 (1.27–2.18) | 1.86 (1.24–2.49) |

### Sicurezza

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Urti | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Droni fuori uso | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Quasi-urti | 0.0833 (0–0.267) | 0.0833 (0–0.267) | 0.0833 (0–0.267) |
| Distanza minima tra due droni [m] | 1 (0.919–1.09) | 0.99 (0.912–1.07) | 0.996 (0.916–1.08) |
| Tempo in emergenza | 0.0% (0%–0.0%) | 0.0% (0%–0.0%) | 0.0% (0%–0.0%) |

### Uso del tempo

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Perlustrazione | 82% (77%–86%) | 79% (74%–83%) | 80% (75%–85%) |
| In volo verso un incendio | 6.6% (4.7%–8.4%) | 9.1% (7.3%–11%) | 7.9% (5.9%–9.8%) |
| Spegnimento | 2.9% (2.3%–3.5%) | 2.7% (2.2%–3.2%) | 3.0% (2.5%–3.5%) |
| In volo verso una stazione | 5.0% (3.5%–6.5%) | 5.8% (4.5%–7.2%) | 5.8% (3.8%–7.9%) |
| In coda alla stazione | 2.4% (1.8%–3.0%) | 2.6% (1.9%–3.3%) | 2.4% (1.9%–2.9%) |
| Rifornimento | 1.3% (1.0%–1.5%) | 1.2% (0.9%–1.4%) | 1.4% (1.1%–1.6%) |
| Fuori uso | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |

### Lavoro ed efficienza

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Equità del lavoro | 0.655 (0.55–0.76) | 0.78 (0.706–0.854) | 0.73 (0.654–0.807) |
| Sovraffollamento sugli incendi [s] | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Attesa media in coda [s] | 3.63 (3.24–4.02) | 4.44 (3.43–5.45) | 3.47 (3.26–3.68) |
| Attesa massima in coda [s] | 8.71 (6.48–11) | 10.6 (6.41–14.8) | 7.71 (6.05–9.36) |
| Distanza per drone [m] | 240 (221–259) | 237 (220–253) | 255 (252–257) |
| Sforzo di controllo [m²/s³] | 0.272 (0.259–0.286) | 0.975 (0.929–1.02) ▼ | 0.831 (0.779–0.882) ▼ |
| Rimbalzi da incendi affollati | 0.417 (0–1.05) | 2.58 (0.903–4.26) | 1 (0.187–1.81) |

### Comunicazione e conoscenza

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 0.545 (0.454–0.636) | 1.02 (0.905–1.14) ▲ | 0.589 (0.528–0.65) |
| Sciame tutto connesso | 0% (0%–0%) | 2.2% (1.0%–3.4%) ▲ | 0.1% (0%–0.4%) |
| Incendi noti ai droni | 15% (11%–19%) | 17% (11%–23%) | 15% (12%–19%) |
| Incendi fantasma per drone | 0.0177 (0.0101–0.0253) | 0.0125 (0.0063–0.0187) | 0.0152 (0.00391–0.0265) |
| Età delle informazioni [s] | 1.01 (0.699–1.32) | 0.385 (0.222–0.548) | 1.14 (0.774–1.51) |

### Perlustrazione

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Obsolescenza del terreno [s] | 27.4 (24.5–30.2) | 48.6 (44.7–52.5) | 31.2 (28.4–34) |
| Obsolescenza delle zone importanti [s] | 18 (14.9–21.2) | 17.5 (15.4–19.6) | 27.7 (24.3–31.1) ▼ |
| Obsolescenza del resto dell'area [s] | 36 (32.9–39.1) | 83.2 (76.5–89.9) ▼ | 34.6 (31.7–37.4) |
| Obsolescenza della zona peggiore [s] | 173 (129–217) | 95.2 (80.1–110) | 149 (134–164) |

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
