# Esperimento «guasti»

**Domanda.** Un guasto parziale fa più danni di uno totale?

**Scenario «critico».** Cinque incendi in posizioni casuali che crescono più in fretta del normale (0.6 vita/s). Lo sciame riesce circa due volte su tre: è in questa zona di difficoltà che le differenze tra varianti si vedono.
Parametri diversi dal default: NUM_FIRES = 5, FIRE_GROWTH_RATE = 0.6. Ogni simulazione dura al massimo 600 s e viene fermata (contando come fallita) se gli incendi accesi superano 40.

**Varianti.** «nessun guasto» (nessuno); «perdita totale (radio spenta)» (FAILURE_INJECTION_COUNT = 3, FAILURE_INJECTION_RADIO_OFF = True); «avaria dichiarata» (FAILURE_INJECTION_COUNT = 3); «guasto silenzioso» (FAILURE_INJECTION_COUNT = 3, SILENT_FAILURE_PROBABILITY = 1.0). Il riferimento è «nessun guasto».

**Metodo.** 20 simulazioni per variante, con i seed da 1 a 20, gli stessi per tutte le varianti: ogni variante affronta quindi esattamente le stesse situazioni di partenza. Eseguito il 23/09/2026 alle 15:15 in 7.6 minuti (codice: commit fb98452 con modifiche non salvate).

## In breve

- **perdita totale (radio spenta)**: tempo per spegnere tutto 140 → 169 (peggiore, su 14 simulazioni); droni fuori uso 0 → 3 (peggiore). Cambiano in modo significativo anche altre 9 misure (vedi tabelle).
- **avaria dichiarata**: tempo per spegnere tutto 140 → 158 (peggiore, su 14 simulazioni); droni fuori uso 0 → 3 (peggiore). Cambiano in modo significativo anche altre 10 misure (vedi tabelle).
- **guasto silenzioso**: droni fuori uso 0 → 3 (peggiore). Cambiano in modo significativo anche altre 7 misure (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «nessun guasto»; ◆ = significativamente diverso, per le misure senza un verso migliore.

### Missione

| Misura | nessun guasto | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|---|
| Missione riuscita | 70% (48%–85%) | 70% (48%–85%) | 70% (48%–85%) | 50% (30%–70%) |
| Incendi fuori controllo | 30% (15%–52%) | 30% (15%–52%) | 30% (15%–52%) | 50% (30%–70%) |
| Tempo per spegnere tutto [s] | 140 (127–153) [su 14] | 169 (130–207) [su 14] ▼ | 158 (144–172) [su 14] ▼ | 184 (163–205) [su 10] |
| Danno degli incendi [vita·s] | 157 148 (84 706–229 591) | 144 766 (89 039–200 492) | 141 729 (84 759–198 700) | 186 661 (130 329–242 994) |
| Fuoco acceso in media [vita] | 727 (513–940) | 690 (499–881) | 700 (498–902) | 828 (614–1 041) |
| Incendi nati dalla propagazione | 16.9 (4.5–29.2) | 15 (4.35–25.6) | 14.4 (3.73–25.2) | 22.2 (11.4–33) |
| Incendi nati da soli | 0 (0–0) | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Picco di incendi accesi | 15.8 (7.94–23.8) | 16.1 (8.25–24) | 15.8 (7.88–23.7) | 23 (14.4–31.6) |
| Ritardo di avvistamento [s] | 6.3 (2.53–10.1) | 6.74 (3.11–10.4) | 7.58 (2.7–12.5) | 11.4 (4.73–18) |
| Incendi mai avvistati | 2.4 (0–5.39) | 3.35 (0–7.19) | 3.1 (0–6.4) | 6.6 (2.04–11.2) |
| Ritardo di intervento [s] | 7.84 (3.42–12.3) | 9.37 (3.23–15.5) | 6.23 (2.72–9.75) | 7.3 (3.65–10.9) |

### Sicurezza

| Misura | nessun guasto | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|---|
| Urti | 0 (0–0) | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Droni fuori uso | 0 (0–0) | 3 (3–3) ▼ | 3 (3–3) ▼ | 3 (3–3) ▼ |
| Quasi-urti | 0.55 (0.135–0.965) | 0.55 (0.135–0.965) | 0.55 (0.135–0.965) | 0.55 (0.135–0.965) |
| Distanza minima tra due droni [m] | 0.791 (0.675–0.907) | 0.795 (0.677–0.914) | 0.801 (0.681–0.921) | 0.783 (0.67–0.897) |
| Tempo in emergenza | 0.2% (0%–0.4%) | 0.2% (0%–0.5%) | 0.2% (0%–0.5%) | 0.2% (0%–0.5%) |

### Uso del tempo

| Misura | nessun guasto | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|---|
| Perlustrazione | 4.8% (3.2%–6.5%) | 3.5% (2.4%–4.6%) | 3.5% (2.4%–4.5%) | 2.7% (1.6%–3.9%) ◆ |
| In volo verso un incendio | 31% (29%–33%) | 24% (23%–26%) ◆ | 24% (22%–25%) ◆ | 21% (18%–24%) ◆ |
| Spegnimento | 14% (13%–15%) | 13% (12%–14%) ▼ | 13% (12%–14%) ▼ | 11% (9.6%–12%) ▼ |
| In volo verso una stazione | 23% (20%–25%) | 16% (15%–18%) ▲ | 17% (15%–19%) ▲ | 21% (18%–23%) |
| In coda alla stazione | 21% (19%–22%) | 16% (15%–17%) ▲ | 16% (15%–18%) ▲ | 19% (16%–22%) |
| Rifornimento | 6.8% (6.4%–7.2%) | 6.2% (5.8%–6.6%) ◆ | 6.1% (5.7%–6.5%) ◆ | 5.0% (4.5%–5.5%) ◆ |
| Fuori uso | 0% (0%–0%) | 21% (20%–21%) ▼ | 21% (20%–21%) ▼ | 21% (21%–22%) ▼ |

### Lavoro ed efficienza

| Misura | nessun guasto | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|---|
| Equità del lavoro | 0.958 (0.93–0.987) | 0.966 (0.941–0.991) | 0.966 (0.939–0.992) | 0.943 (0.908–0.978) |
| Sovraffollamento sugli incendi [s] | 0.074 (0–0.229) | 0.074 (0–0.229) | 0.088 (0–0.244) | 0.074 (0–0.229) |
| Attesa media in coda [s] | 6.28 (5.74–6.82) | 5.34 (4.89–5.78) ▲ | 5.49 (5.04–5.93) ▲ | 8.5 (5.48–11.5) |
| Attesa massima in coda [s] | 21.6 (17.8–25.4) | 19.1 (15.9–22.3) | 20.2 (15.8–24.5) | 42.4 (23.8–61.1) |
| Distanza per drone [m] | 80.8 (66.8–94.9) | 87.7 (70.9–105) | 81.5 (71.1–91.8) | 87.8 (78.5–97.2) |
| Sforzo di controllo [m²/s³] | 0.493 (0.478–0.508) | 0.467 (0.451–0.483) | 0.466 (0.448–0.483) ▲ | 0.474 (0.454–0.493) |
| Rimbalzi da incendi affollati | 1.7 (1.09–2.31) | 0.7 (0.325–1.08) | 0.85 (0.363–1.34) | 0.45 (0.167–0.733) ◆ |

### Comunicazione e conoscenza

| Misura | nessun guasto | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 2.89 (2.69–3.09) | 2.32 (2.13–2.5) ▼ | 2.57 (2.34–2.81) ▼ | 2.65 (2.43–2.87) |
| Sciame tutto connesso | 17% (11%–22%) | 0.3% (0%–0.6%) ▼ | 1.9% (0%–4.5%) ▼ | 5.7% (0.3%–11%) |
| Incendi noti ai droni | 81% (74%–88%) | 78% (70%–86%) | 78% (69%–87%) | 73% (63%–82%) |
| Incendi fantasma per drone | 0.082 (0.0385–0.126) | 0.101 (0.0549–0.147) | 0.104 (0.0627–0.146) | 0.311 (0.25–0.372) ▼ |
| Età delle informazioni [s] | 0.299 (0.14–0.457) | 0.395 (0.209–0.582) | 0.357 (0.169–0.544) | 0.373 (0.207–0.539) |

### Perlustrazione

| Misura | nessun guasto | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|---|
| Obsolescenza del terreno [s] | 33.3 (24.6–42.1) | 38.1 (29.9–46.3) | 36.7 (29.7–43.7) | 43.4 (37.4–49.4) |
| Obsolescenza delle zone importanti [s] | 30.6 (20.9–40.4) | 35.4 (26.2–44.7) | 34.3 (26–42.7) | 40.6 (33.4–47.8) |
| Obsolescenza del resto dell'area [s] | 38.6 (31–46.2) | 43.5 (36.3–50.6) | 41.6 (36.2–47) | 49 (43.6–54.4) |
| Obsolescenza della zona peggiore [s] | 172 (141–202) | 184 (153–215) | 176 (156–196) | 209 (188–230) |

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
