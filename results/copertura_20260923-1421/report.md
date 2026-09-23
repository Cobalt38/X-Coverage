# Esperimento «copertura»

**Domanda.** Perlustrare dove non si guarda da più tempo fa trovare prima gli incendi?

**Scenario «ricerca».** Area quattro volte più grande (40 × 24 m), un solo incendio iniziale e accensioni spontanee distribuite secondo l'importanza del terreno (una ogni ~33 s). Qui il problema non è l'acqua ma TROVARE gli incendi: i droni passano l'80% del tempo a perlustrare, contro il 6% dello scenario 'critico'.
Parametri diversi dal default: AREA_WIDTH = 40.0, AREA_HEIGHT = 24.0, NUM_FIRES = 1, IGNITION_RATE_PER_S = 0.03. Ogni simulazione dura al massimo 300 s e viene fermata (contando come fallita) se gli incendi accesi superano 40.

**Varianti.** «ricerca casuale» (nessuno); «copertura persistente» (EXPLORATION_MODE = coverage); «copertura uniforme» (EXPLORATION_MODE = coverage, COVERAGE_IMPORTANCE_EXPONENT = 0.0). Il riferimento è «ricerca casuale».

**Metodo.** 30 simulazioni per variante, con i seed da 1 a 30, gli stessi per tutte le varianti: ogni variante affronta quindi esattamente le stesse situazioni di partenza. Eseguito il 23/09/2026 alle 14:21 in 10.3 minuti (codice: commit fb98452 con modifiche non salvate).

## In breve

- **copertura persistente**: nessun cambiamento tra le misure principali. Cambiano in modo significativo anche altre 10 misure (vedi tabelle).
- **copertura uniforme**: nessun cambiamento tra le misure principali. Cambiano in modo significativo anche altre 5 misure (vedi tabelle).

## Risultati

Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. ▲ / ▼ = significativamente migliore / peggiore di «ricerca casuale»; ◆ = significativamente diverso, per le misure senza un verso migliore.

### Missione

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Missione riuscita | – [su 0] | – [su 0] | – [su 0] |
| Incendi fuori controllo | 10% (3.5%–26%) | 13% (5.3%–30%) | 0% (0%–11%) |
| Tempo per spegnere tutto [s] | – [su 0] | – [su 0] | – [su 0] |
| Danno degli incendi [vita·s] | 40 765 (20 956–60 575) | 58 783 (31 773–85 793) | 27 206 (23 071–31 342) |
| Fuoco acceso in media [vita] | 164 (67–261) | 231 (116–347) | 90.7 (76.9–104) |
| Incendi nati dalla propagazione | 4.23 (0–8.99) | 6.7 (0.96–12.4) | 0 (0–0) |
| Incendi nati da soli | 7.5 (6.46–8.54) | 7.6 (6.56–8.64) | 7.87 (6.9–8.84) |
| Picco di incendi accesi | 7.33 (3.05–11.6) | 9.87 (4.66–15.1) | 3.8 (3.36–4.24) |
| Ritardo di avvistamento [s] | 19.8 (15.9–23.7) | 23.4 (18.8–28.1) | 23 (18.6–27.5) |
| Incendi mai avvistati | 3.6 (0.643–6.56) | 6.83 (2.16–11.5) | 1.37 (0.882–1.85) |
| Ritardo di intervento [s] | 2.31 (1.59–3.04) | 2.71 (1.1–4.33) | 2.04 (1.46–2.61) |

### Sicurezza

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Urti | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Droni fuori uso | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Quasi-urti | 0.1 (0–0.214) | 0.1 (0–0.214) | 0.1 (0–0.214) |
| Distanza minima tra due droni [m] | 0.994 (0.946–1.04) | 0.981 (0.934–1.03) | 0.996 (0.948–1.04) |
| Tempo in emergenza | 0.0% (0%–0.0%) | 0.0% (0%–0.0%) | 0.0% (0%–0.0%) |

### Uso del tempo

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Perlustrazione | 80% (77%–82%) | 76% (73%–80%) | 79% (77%–82%) |
| In volo verso un incendio | 7.5% (6.4%–8.6%) | 10% (8.8%–11%) ◆ | 7.9% (6.8%–8.9%) |
| Spegnimento | 3.2% (2.9%–3.5%) | 2.8% (2.5%–3.1%) | 3.2% (2.9%–3.4%) |
| In volo verso una stazione | 5.7% (4.8%–6.6%) | 6.7% (5.1%–8.2%) | 5.9% (4.8%–6.9%) |
| In coda alla stazione | 2.6% (2.3%–2.9%) | 2.9% (2.3%–3.4%) | 2.5% (2.2%–2.7%) |
| Rifornimento | 1.4% (1.3%–1.6%) | 1.2% (1.1%–1.4%) | 1.4% (1.3%–1.6%) |
| Fuori uso | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |

### Lavoro ed efficienza

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Equità del lavoro | 0.686 (0.631–0.741) | 0.794 (0.749–0.838) ▲ | 0.709 (0.661–0.758) |
| Sovraffollamento sugli incendi [s] | 0 (0–0) | 0 (0–0) | 0 (0–0) |
| Attesa media in coda [s] | 3.59 (3.35–3.82) | 4.44 (3.9–4.99) | 3.44 (3.28–3.6) |
| Attesa massima in coda [s] | 8.53 (7.21–9.85) | 10.8 (8.46–13.1) | 8 (6.98–9.02) |
| Distanza per drone [m] | 243 (234–252) | 234 (224–244) | 254 (252–256) ◆ |
| Sforzo di controllo [m²/s³] | 0.279 (0.269–0.289) | 0.944 (0.911–0.978) ▼ | 0.821 (0.798–0.845) ▼ |
| Rimbalzi da incendi affollati | 0.367 (0.0811–0.652) | 1.93 (1.11–2.75) ◆ | 0.867 (0.42–1.31) |

### Comunicazione e conoscenza

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Messaggi persi | 0% (0%–0%) | 0% (0%–0%) | 0% (0%–0%) |
| Vicini radio per drone | 0.553 (0.505–0.6) | 1.05 (0.966–1.14) ▲ | 0.576 (0.541–0.611) |
| Sciame tutto connesso | 0% (0%–0%) | 1.5% (0.9%–2.2%) ▲ | 0.1% (0%–0.1%) |
| Incendi noti ai droni | 14% (12%–16%) | 16% (12%–20%) | 16% (14%–18%) |
| Incendi fantasma per drone | 0.0168 (0.0125–0.0211) | 0.0188 (0.00766–0.03) | 0.0157 (0.00975–0.0216) |
| Età delle informazioni [s] | 1.34 (1.09–1.59) | 0.427 (0.312–0.542) ▲ | 1.08 (0.857–1.31) |

### Perlustrazione

| Misura | ricerca casuale | copertura persistente | copertura uniforme |
|---|---|---|---|
| Obsolescenza del terreno [s] | 27.9 (26.4–29.4) | 48.8 (46.5–51.1) ▼ | 30.8 (29.3–32.2) ▼ |
| Obsolescenza delle zone importanti [s] | 18.5 (16.8–20.1) | 17.6 (16.2–19) | 28 (26.3–29.7) ▼ |
| Obsolescenza del resto dell'area [s] | 37 (35.3–38.7) | 83.5 (79.9–87.1) ▼ | 33.5 (32–35) ▲ |
| Obsolescenza della zona peggiore [s] | 169 (150–189) | 97.6 (83.5–112) ▲ | 150 (142–157) |

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
