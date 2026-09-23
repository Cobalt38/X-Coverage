# Esperimento «guasti»

**Domanda.** Un guasto parziale fa più danni di uno totale?

**Scenario «critico».** Cinque incendi in posizioni casuali che crescono più in fretta del normale (0.6 vita/s). Lo sciame riesce circa due volte su tre: è in questa zona di difficoltà che le differenze tra varianti si vedono.
Parametri diversi dal default: NUM_FIRES = 5, FIRE_GROWTH_RATE = 0.6. Ogni simulazione dura al massimo 600 s e viene fermata (contando come fallita) se gli incendi accesi superano 40.

**Varianti.** «perdita totale (radio spenta)» (AVOIDANCE_MODE = none, COLLISION_TOTAL_LOSS_SPEED = 0.0); «avaria dichiarata» (AVOIDANCE_MODE = none, COLLISION_TOTAL_LOSS_SPEED = 99.0); «guasto silenzioso» (AVOIDANCE_MODE = none, COLLISION_TOTAL_LOSS_SPEED = 99.0, SILENT_FAILURE_PROBABILITY = 1.0). Il riferimento è «perdita totale (radio spenta)».

**Metodo.** 20 simulazioni per variante, con i seed da 1 a 20, gli stessi per tutte le varianti: ogni variante affronta quindi esattamente le stesse situazioni di partenza. Eseguito il 23/09/2026 alle 14:57 in 1.2 minuti (codice: commit fb98452 con modifiche non salvate).

## In breve

- **avaria dichiarata**: nessun cambiamento tra le misure principali. Cambiano in modo significativo anche altre 2 misure (vedi tabelle).
- **guasto silenzioso**: nessun cambiamento tra le misure principali. Cambiano in modo significativo anche altre 2 misure (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «perdita totale (radio spenta)»; ◆ = significativamente diverso, per le misure senza un verso migliore.

### Missione

| Misura | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|
| Missione riuscita | 5.0% (0.9%–24%) | 5.0% (0.9%–24%) | 0% (0%–16%) |
| Incendi fuori controllo | 25% (11%–47%) | 30% (15%–52%) | 45% (26%–66%) |
| Tempo per spegnere tutto [s] | 394 [su 1] | 392 [su 1] | – |
| Danno degli incendi [vita·s] | 133 211 (90 328–176 095) | 127 703 (90 227–165 178) | 140 279 (104 011–176 546) |
| Fuoco acceso in media [vita] | 841 (713–969) | 873 (750–995) | 865 (738–991) |
| Incendi nati dalla propagazione | 17 (9.18–24.8) | 17.4 (9.4–25.3) | 19.2 (10.2–28.3) |
| Incendi nati da soli | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Picco di incendi accesi | 20.5 (13.2–27.8) | 21.4 (13.7–29) | 22.7 (14.2–31.2) |
| Ritardo di avvistamento [s] | 3.46 (1.76–5.16) | 2.74 (1.79–3.7) | 3.79 (2.18–5.4) |
| Incendi mai avvistati | 15.3 (8.28–22.4) | 16.4 (8.9–24) | 15.7 (8.11–23.3) |
| Ritardo di intervento [s] | 2.36 (1.49–3.23) | 2.05 (1.26–2.84) | 3.84 (1.76–5.91) |

### Sicurezza

| Misura | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|
| Urti | 5.7 (5.48–5.92) | 5.65 (5.42–5.88) | 5.5 (5.26–5.74) |
| Droni fuori uso | 11.4 (11–11.8) | 11.3 (10.8–11.8) | 11 (10.5–11.5) |
| Quasi-urti | 29.7 (24.3–35.1) | 28.1 (23.3–33) | 29.8 (24.3–35.2) |
| Distanza minima tra due droni [m] | 0.0913 (0.0891–0.0934) | 0.0911 (0.0889–0.0933) | 0.0903 (0.0877–0.0929) |
| Tempo in emergenza | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |

### Uso del tempo

| Misura | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|
| Perlustrazione | 5.4% (3.0%–7.8%) | 4.8% (2.9%–6.8%) | 4.3% (2.5%–6.0%) |
| In volo verso un incendio | 6.8% (5.7%–7.9%) | 7.4% (6.3%–8.6%) | 6.9% (5.6%–8.2%) |
| Spegnimento | 9.5% (7.8%–11%) | 10% (7.8%–12%) | 8.9% (7.6%–10%) |
| In volo verso una stazione | 3.5% (2.8%–4.3%) | 3.7% (2.9%–4.5%) | 4.3% (3.4%–5.2%) |
| In coda alla stazione | 4.9% (4.0%–5.8%) | 5.4% (3.9%–6.9%) | 5.2% (4.4%–6.0%) |
| Rifornimento | 4.1% (3.4%–4.8%) | 4.2% (3.3%–5.0%) | 3.8% (3.3%–4.4%) |
| Fuori uso | 66% (60%–71%) | 64% (57%–71%) | 67% (61%–72%) |

### Lavoro ed efficienza

| Misura | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|
| Equità del lavoro | 0.98 (0.954–1.01) [su 6] | 0.987 (0.976–0.997) [su 7] | 0.95 (0.903–0.996) [su 10] |
| Sovraffollamento sugli incendi [s] | 0.078 (0.0053–0.151) | 0.078 (0.0053–0.151) | 0.147 (0.00309–0.291) |
| Attesa media in coda [s] | 2.49 (2.36–2.62) | 2.56 (2.42–2.71) | 2.9 (2.33–3.47) |
| Attesa massima in coda [s] | 4.75 (3.92–5.58) | 5.06 (4.05–6.08) | 9.52 (4.36–14.7) |
| Distanza per drone [m] | 77.8 (56.2–99.3) | 72 (51.4–92.7) | 81.9 (53.7–110) |
| Sforzo di controllo [m²/s³] | 0.257 (0.248–0.266) | 0.263 (0.255–0.271) | 0.256 (0.241–0.271) |
| Rimbalzi da incendi affollati | 0.75 (0.324–1.18) | 0.75 (0.324–1.18) | 0.8 (0.381–1.22) |

### Comunicazione e conoscenza

| Misura | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 0.677 (0.48–0.875) | 2.24 (1.78–2.7) ▲ | 1.87 (1.59–2.15) ▲ |
| Sciame tutto connesso | 0% (0%–0%) | 0.0% (0%–0.0%) | 0% (0%–0%) |
| Incendi noti ai droni | 35% (28%–41%) | 44% (35%–53%) | 48% (36%–60%) |
| Incendi fantasma per drone | 0.013 (0–0.0293) | 0.0614 (0.0233–0.0994) | 0.28 (0.089–0.471) ▼ |
| Età delle informazioni [s] | 1.13 (0.788–1.48) | 0.459 (0.239–0.679) ▲ | 0.664 (0.44–0.888) |

### Perlustrazione

| Misura | perdita totale (radio spenta) | avaria dichiarata | guasto silenzioso |
|---|---|---|---|
| Obsolescenza del terreno [s] | 46.4 (32.8–59.9) | 46.4 (32.5–60.3) | 49.7 (34.6–64.9) |
| Obsolescenza delle zone importanti [s] | 43.4 (30.1–56.7) | 44.4 (30.6–58.3) | 45.1 (32.6–57.7) |
| Obsolescenza del resto dell'area [s] | 53 (37.4–68.6) | 51.3 (35.9–66.6) | 59.3 (38–80.6) |
| Obsolescenza della zona peggiore [s] | 159 (114–204) | 149 (106–193) | 173 (115–231) |

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
