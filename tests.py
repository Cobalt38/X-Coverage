"""
LE VERIFICHE AUTOMATICHE: si lanciano con   python main.py test

Servono a poter cambiare il codice senza paura. Sono divise in sei gruppi, ognuno con uno scopo:

    1. Regressione     le traiettorie corrispondono a un'impronta digitale registrata. Toccare la
                       logica dei droni fa fallire questo test: se il cambiamento è voluto, si
                       aggiorna l'impronta, tenendo presente che i risultati raccolti con l'impronta
                       precedente non sono più confrontabili con i nuovi.
    2. Varianti        ogni interruttore fa davvero qualcosa, e lasciandolo com'è non cambia nulla.
    3. Autonomia       un drone non conosce gli altri droni: decide solo con la posta che riceve.
    4. Urti            un drone che si scontra si rompe, e nel modo giusto a seconda dell'impatto.
    5. Perlustrazione  la copertura persistente e le accensioni spontanee si comportano come descritto.
    6. Misure e statistica   i numeri del report sono calcolati correttamente su casi noti.
"""

import hashlib
import unittest

import numpy as np

import experiments
from drone import CoverageMemory, Drone, choose_patrol_point
from simulation import MissionOutcome, Simulation
from world import DroneMessage, DroneStatus, ImportanceMap, SimConfig, validate_config, vec_to_tuple

GOLDEN_STEPS = 3000

# Impronte calcolate sulla versione di riferimento del codice: riassumono in un'unica stringa
# posizioni, velocità, acqua e incendi dopo GOLDEN_STEPS passi.
GOLDEN = {
    (42, False, False): "69be35783a2f60536f9aa3998846f4bbe5e823b4f96f374ab73799aefec0d3bf",
    (7, True, True): "04fe46bf7ba518858b238061ea7c2405bc3a10e1d9fcffe0f7081251c53906af",
}


def fingerprint(simulation: Simulation) -> str:
    """Un'impronta dello stato completo: cambia se cambia anche un solo bit."""
    digest = hashlib.sha256()
    for drone in simulation.drones:
        digest.update(drone.position.tobytes())
        digest.update(drone.velocity.tobytes())
        digest.update(np.float64(drone.water).tobytes())
    for fire in simulation.world.fires:
        digest.update(fire.pos.tobytes())
        digest.update(np.float64(fire.health).tobytes())
    return digest.hexdigest()


def simulate_steps(steps: int, seed: int = 42, random_fires: bool = False,
                   random_stations: bool = False, **parameters) -> Simulation:
    """Fa girare la simulazione per un numero esatto di passi e la restituisce."""
    config = SimConfig().with_overrides(**parameters)
    simulation = Simulation(seed=seed, random_fires=random_fires, random_stations=random_stations,
                            config=config)
    simulation.run(max_time_s=steps * config.SIM_TIME_STEP)
    return simulation


class Regressione(unittest.TestCase):

    def test_le_traiettorie_non_sono_cambiate(self):
        for (seed, random_fires, random_stations), expected in GOLDEN.items():
            with self.subTest(seed=seed):
                simulation = simulate_steps(GOLDEN_STEPS, seed, random_fires, random_stations)
                self.assertEqual(fingerprint(simulation), expected)

    def test_misurare_non_cambia_il_risultato(self):
        plain = simulate_steps(1500)

        config = SimConfig()
        watched = Simulation(seed=42, config=config)
        measurements = experiments.Measurements(watched)
        watched.run(max_time_s=1500 * config.SIM_TIME_STEP, watchers=[measurements])

        self.assertEqual(fingerprint(watched), fingerprint(plain))

    def test_la_missione_finisce_da_sola_quando_gli_incendi_sono_spenti(self):
        simulation = Simulation(seed=42)
        result = simulation.run(max_time_s=600.0)
        self.assertIs(result.outcome, MissionOutcome.ALL_FIRES_OUT)
        self.assertEqual(result.fires_left, 0)
        self.assertAlmostEqual(result.elapsed_s, 67.29, places=2)


class Varianti(unittest.TestCase):

    def test_i_valori_di_default_non_cambiano_niente(self):
        explicit = simulate_steps(1500, AVOIDANCE_MODE="full", PACKET_LOSS=0.0,
                                  SATURATION_BOUNCE=True, EXPLORATION_MODE="random",
                                  IGNITION_RATE_PER_S=0.0, COLLISION_DAMAGE=True)
        self.assertEqual(fingerprint(explicit), fingerprint(simulate_steps(1500)))

    def test_ogni_variante_cambia_davvero_il_comportamento(self):
        reference = fingerprint(simulate_steps(1500))
        for parameters in ({"AVOIDANCE_MODE": "emergency-only"}, {"AVOIDANCE_MODE": "none"},
                           {"PACKET_LOSS": 0.3}, {"EXPLORATION_MODE": "coverage"}):
            with self.subTest(**parameters):
                self.assertNotEqual(fingerprint(simulate_steps(1500, **parameters)), reference)

    def test_i_messaggi_si_perdono_nella_proporzione_richiesta(self):
        channel = simulate_steps(500, PACKET_LOSS=0.25).world.channel
        self.assertAlmostEqual(channel.messages_dropped / channel.messages_attempted, 0.25, delta=0.02)

    def test_cambiare_variante_non_cambia_la_situazione_di_partenza(self):
        # Tutto ciò che è casuale ha un generatore separato, così due varianti con lo stesso seed
        # affrontano gli stessi incendi partendo dagli stessi posti: il confronto resta onesto.
        for parameters in ({"PACKET_LOSS": 0.5}, {"EXPLORATION_MODE": "coverage"},
                           {"IGNITION_RATE_PER_S": 0.1}):
            with self.subTest(**parameters):
                variant = Simulation(seed=5, random_fires=True, config=SimConfig(**parameters))
                plain = Simulation(seed=5, random_fires=True)
                self.assertTrue(all(np.array_equal(one.pos, other.pos)
                                    for one, other in zip(variant.fires, plain.fires)))
                self.assertTrue(all(np.array_equal(one.position, other.position)
                                    for one, other in zip(variant.drones, plain.drones)))

    def test_i_parametri_incoerenti_vengono_segnalati(self):
        self.assertEqual(validate_config(SimConfig()), [])
        self.assertTrue(validate_config(SimConfig(AVOIDANCE_MODE="boh")))
        self.assertTrue(validate_config(SimConfig(PACKET_LOSS=1.5)))
        self.assertTrue(validate_config(SimConfig(EXPLORATION_MODE="boh")))
        self.assertTrue(validate_config(SimConfig(COVERAGE_CELL_SIZE=5.0)))
        self.assertTrue(validate_config(SimConfig(IGNITION_RATE_PER_S=-1.0)))
        # Raggio radio troppo corto: due droni sullo stesso incendio non si sentirebbero più.
        self.assertTrue(validate_config(SimConfig(COMMUNICATION_RADIUS=1.5)))


class Autonomia(unittest.TestCase):
    """Un drone non viene comandato da nessuno: risponde al tempo e alla posta, e basta."""

    def test_non_tiene_riferimenti_agli_altri_droni(self):
        # Se un drone avesse in pancia un altro drone potrebbe leggerne lo stato vero, aggirando
        # la radio: la decentralizzazione sarebbe finta. Qui si controlla che non accada.
        simulation = Simulation(seed=1)
        drone = simulation.drones[0]
        for name, value in vars(drone).items():
            if name in ("world", "mailbox"):        # l'ambiente e la propria cassetta postale
                continue
            self.assertNotIsInstance(value, Drone, f"{name} punta a un altro drone")
            items = value.values() if isinstance(value, dict) else value
            if isinstance(items, (list, tuple, set)) or isinstance(value, dict):
                for item in items:
                    self.assertNotIsInstance(item, Drone, f"{name} contiene un altro drone")

    def test_decide_in_base_ai_messaggi_che_riceve(self):
        """Un drone solo al mondo, a cui si consegna a mano della posta inventata.

        Da solo si prende l'incendio. Se gli arrivano tre messaggi che dicono "ci stiamo lavorando
        noi", si fa da parte: la decisione dipende SOLO da quello che gli è stato detto.
        """
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=1))
        drone = simulation.drones[0]
        fire = simulation.world.fires[0]
        fire_position = vec_to_tuple(fire.pos)
        drone.position[:] = fire.pos + np.array([1.5, 0.0])     # abbastanza vicino da vederlo

        simulation._advance_one_step()
        self.assertEqual(drone.fire_target, fire_position, "da solo dovrebbe occuparsene lui")

        for neighbour_idx in (1, 2, 3):
            drone.mailbox.deliver(neighbour_idx, self._busy_neighbour(fire.pos, neighbour_idx))
        simulation._advance_one_step()
        self.assertIsNone(drone.fire_target, "con tre droni già sul posto dovrebbe farsi da parte")
        self.assertIn(fire_position, drone.saturated_fires)

    @staticmethod
    def _busy_neighbour(fire_position: np.ndarray, idx: int) -> DroneMessage:
        """Un messaggio inventato: un drone fermo sul fuoco che dice di starlo spegnendo."""
        angle = idx * 2.0
        position = fire_position + 0.8 * np.array([np.cos(angle), np.sin(angle)])
        return DroneMessage(position=position, velocity=np.zeros(2), target=position.copy(),
                            reloading=False, water_station_idx=None, refuel_claim_age=0,
                            station_slot=None, flying=True, extinguishing=True,
                            fire_target=vec_to_tuple(fire_position), known_fires={}, extinguished_fires={})

    def test_un_drone_distrutto_smette_di_rispondere_al_clock(self):
        simulation = Simulation(seed=1)
        clock = simulation.world.clock
        subscribers_before = len(clock._thinking_phase)
        simulation.drones[0].break_down(radio_destroyed=True)
        self.assertEqual(len(clock._thinking_phase), subscribers_before - 1)
        # Chi è solo a terra continua invece a farsi svegliare, perché la radio funziona ancora.
        simulation.drones[1].break_down(radio_destroyed=False)
        self.assertEqual(len(clock._thinking_phase), subscribers_before - 1)


class Urti(unittest.TestCase):
    """Le conseguenze di uno scontro. Si preparano a mano due droni a contatto e si chiede alla
    simulazione di valutare gli urti, senza aspettare che accada per caso."""

    def _crash(self, first_velocity, second_velocity, **parameters) -> Simulation:
        simulation = Simulation(seed=1, config=SimConfig(**parameters))
        one, other = simulation.drones[0], simulation.drones[1]
        one.position[:] = (5.0, 5.0)
        other.position[:] = (5.0 + simulation.config.DRONE_IMPACT_RADIUS / 2, 5.0)
        one.velocity[:] = first_velocity
        other.velocity[:] = second_velocity
        simulation._handle_collisions()
        return simulation

    def test_uno_scontro_frontale_distrugge_anche_la_radio(self):
        simulation = self._crash((1.0, 0.0), (-1.0, 0.0))      # 2 m/s di velocità relativa
        self.assertEqual(simulation.collisions, 1)
        for drone in simulation.drones[:2]:
            self.assertIs(drone.status, DroneStatus.DESTROYED)
            self.assertFalse(drone.radio_works)

    def test_uno_sfioramento_lascia_la_radio_funzionante(self):
        simulation = self._crash((0.1, 0.0), (0.0, 0.0))       # 0.1 m/s: molto sotto la soglia
        for drone in simulation.drones[:2]:
            self.assertIs(drone.status, DroneStatus.GROUNDED)
            self.assertTrue(drone.radio_works)
            self.assertFalse(drone.is_flying)
            self.assertTrue(np.array_equal(drone.velocity, np.zeros(2)))

    def test_si_puo_tornare_agli_urti_innocui(self):
        simulation = self._crash((1.0, 0.0), (-1.0, 0.0), COLLISION_DAMAGE=False)
        self.assertEqual(simulation.collisions, 1)             # l'urto si conta comunque
        self.assertTrue(all(drone.is_flying for drone in simulation.drones))

    def test_un_drone_distrutto_sparisce_dalla_radio_uno_a_terra_no(self):
        simulation = Simulation(seed=1)
        grounded, destroyed = simulation.drones[0], simulation.drones[1]
        grounded.position[:] = (5.0, 5.0)
        destroyed.position[:] = (5.5, 5.0)
        listener = simulation.drones[2]
        listener.position[:] = (5.2, 5.0)
        grounded.break_down(radio_destroyed=False)
        destroyed.break_down(radio_destroyed=True)

        neighbors = simulation.world.refresh_neighbors()
        heard_by_listener = [drone.idx for drone in neighbors[listener.idx]]
        self.assertIn(grounded.idx, heard_by_listener)
        self.assertNotIn(destroyed.idx, heard_by_listener)

    def test_un_drone_a_terra_continua_a_fare_da_ripetitore(self):
        simulation = Simulation(seed=1)
        relay = simulation.drones[0]
        relay.known_fires[(3.0, 4.0)] = 0
        relay.break_down(radio_destroyed=False)

        simulation._advance_one_step()
        self.assertIn((3.0, 4.0), relay.known_fires)           # se ne ricorda ancora
        self.assertEqual(relay.known_fires[(3.0, 4.0)], 1)     # ma la notizia è invecchiata di un passo

    def test_senza_droni_in_volo_la_missione_si_ferma(self):
        simulation = Simulation(seed=1)
        for drone in simulation.drones:
            drone.break_down(radio_destroyed=True)
        result = simulation.run(max_time_s=10.0)
        self.assertIs(result.outcome, MissionOutcome.SWARM_DOWN)
        self.assertEqual(result.drones_lost, len(simulation.drones))


class Relitti(unittest.TestCase):
    """Un drone precipitato non deve diventare un ostacolo per chi vola ancora."""

    def _wreck_and_flyer(self, radio_alive: bool = True):
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=2))
        wreck, flyer = simulation.drones
        wreck.position[:] = (10.0, 6.0)
        wreck.break_down(radio_destroyed=not radio_alive)
        flyer.position[:] = (9.0, 6.0)
        flyer.velocity[:] = (0.5, 0.0)
        flyer.target = np.array([14.0, 6.0])
        simulation.world.refresh_neighbors()
        wreck._broadcast()
        flyer.mailbox.begin_round()
        return simulation, wreck, flyer

    def test_non_ci_si_scansa_da_un_drone_a_terra(self):
        # Sta a terra: ci si vola sopra, quindi la spinta per scansarlo deve essere esattamente zero.
        _, _, flyer = self._wreck_and_flyer()
        correction, emergency, _ = flyer._avoidance_correction()
        self.assertEqual(float(np.linalg.norm(correction)), 0.0)
        self.assertFalse(emergency)

    def test_si_puo_chiedere_che_i_relitti_restino_ingombranti(self):
        # L'alternativa esiste come variante da misurare: il rottame resta dov'è e va scansato.
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=2, WRECK_BLOCKS_FLIGHT=True))
        wreck, flyer = simulation.drones
        wreck.position[:] = (10.0, 6.0)
        wreck.break_down(radio_destroyed=False)
        flyer.position[:] = (9.0, 6.0)
        flyer.velocity[:] = (0.5, 0.0)
        flyer.target = np.array([14.0, 6.0])
        simulation.world.refresh_neighbors()
        wreck._broadcast()
        flyer.mailbox.begin_round()
        correction, _, _ = flyer._avoidance_correction()
        self.assertGreater(float(np.linalg.norm(correction)), 1.0)

    def test_un_relitto_non_si_prende_una_zona_da_perlustrare(self):
        # La cella di Voronoi si divide solo tra chi può davvero andare a guardare.
        simulation, wreck, flyer = self._wreck_and_flyer()
        flyer.coverage = CoverageMemory(simulation.terrain.shape)
        destination = flyer._patrol_point()
        self.assertGreater(np.linalg.norm(destination - flyer.position), 1.0)

    def test_un_relitto_non_blocca_per_sempre_un_posto_di_rifornimento(self):
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=2))
        wreck, thirsty = simulation.drones
        station_idx = 0
        blocked_slot = thirsty._slot_position(station_idx, 0)
        wreck.position[:] = blocked_slot
        wreck.break_down(radio_destroyed=False)
        thirsty.position[:] = simulation.water_stations[station_idx] + np.array([1.0, 0.0])
        thirsty.water = 0.0
        thirsty.water_station_idx = station_idx
        thirsty.reloading = True
        simulation.world.refresh_neighbors()
        wreck._broadcast()
        thirsty.mailbox.begin_round()
        thirsty._update_my_slot()
        self.assertNotEqual(thirsty.station_slot, 0, "si è assegnato il posto occupato dal relitto")

    def test_il_guasto_silenzioso_continua_a_dire_di_volare(self):
        # È il caso peggiore: la macchina è a terra ma per lo sciame è ancora in servizio.
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=2, SILENT_FAILURE_PROBABILITY=1.0))
        liar, listener = simulation.drones
        liar.position[:] = (10.0, 6.0)
        liar.fire_target = (12.0, 7.0)
        liar.known_fires[(12.0, 7.0)] = 3
        liar.break_down(radio_destroyed=False)

        self.assertIs(liar.status, DroneStatus.SILENT)
        self.assertFalse(liar.is_flying)
        self.assertTrue(liar.radio_works)
        self.assertEqual(liar.fire_target, (12.0, 7.0), "non ha liberato l'incendio che aveva preso")

        listener.position[:] = (10.5, 6.0)
        simulation.world.refresh_neighbors()
        liar._broadcast()
        listener.mailbox.begin_round()
        message = listener.neighbors_heard[liar.idx]
        self.assertTrue(message.flying, "dovrebbe dichiararsi ancora in volo")
        self.assertEqual(message.fire_target, (12.0, 7.0))

    def test_il_guasto_silenzioso_congela_le_notizie(self):
        # La radio ripete la stessa fotografia: le età delle notizie non avanzano più, quindi per
        # i vicini quell'informazione resta eternamente "appena confermata".
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=2, SILENT_FAILURE_PROBABILITY=1.0))
        liar = simulation.drones[0]
        liar.known_fires[(12.0, 7.0)] = 3
        liar.break_down(radio_destroyed=False)
        first = liar._frozen_message.known_fires[(12.0, 7.0)]
        for _ in range(500):
            simulation._advance_one_step()
        self.assertEqual(liar._frozen_message.known_fires[(12.0, 7.0)], first)

    def test_senza_guasti_silenziosi_non_si_estrae_nessun_numero(self):
        # Con probabilità 0 il generatore del drone non deve essere toccato, altrimenti attivare
        # l'opzione cambierebbe le traiettorie anche quando è spenta.
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=2))
        drone = simulation.drones[0]
        state_before = drone.rng.getstate()
        drone.break_down(radio_destroyed=False)
        self.assertIs(drone.status, DroneStatus.GROUNDED)
        self.assertEqual(drone.rng.getstate(), state_before)

    def test_un_urto_multiplo_non_ripara_nessuno(self):
        # Tre droni a contatto nello stesso passo: le velocità d'impatto vanno misurate tutte
        # prima di applicare i danni, altrimenti un drone già distrutto tornerebbe "solo a terra".
        simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=3))
        first, second, third = simulation.drones
        first.position[:] = (5.0, 5.0); first.velocity[:] = (1.0, 0.0)
        second.position[:] = (5.05, 5.0); second.velocity[:] = (-1.0, 0.0)
        third.position[:] = (5.02, 5.03); third.velocity[:] = (0.9, 0.9)
        simulation._handle_collisions()
        for drone in simulation.drones:
            self.assertIs(drone.status, DroneStatus.DESTROYED)


class Perlustrazione(unittest.TestCase):

    def test_unire_due_mappe_di_copertura_non_dipende_dall_ordine(self):
        # È la proprietà che rende il gossip robusto: messaggi persi, ripetuti o fuori ordine
        # portano comunque tutti alla stessa mappa.
        terrain = ImportanceMap(SimConfig())
        first, second = CoverageMemory(terrain.shape), CoverageMemory(terrain.shape)
        first.mark_seen(terrain.cells_within(np.array([5.0, 5.0]), 2.0), 100)
        second.mark_seen(terrain.cells_within(np.array([6.0, 5.0]), 2.0), 200)

        one_way = CoverageMemory(terrain.shape)
        one_way.merge(first.last_seen)
        one_way.merge(second.last_seen)

        other_way = CoverageMemory(terrain.shape)
        other_way.merge(second.last_seen)
        other_way.merge(first.last_seen)
        other_way.merge(second.last_seen)          # ripetuto di proposito

        self.assertTrue(np.array_equal(one_way.last_seen, other_way.last_seen))

    def test_la_freschezza_si_dimezza_nel_tempo_previsto(self):
        config = SimConfig(COVERAGE_HALF_LIFE_S=30.0)
        coverage = CoverageMemory((1, 1))
        coverage.mark_seen((slice(0, 1), slice(0, 1), np.ones((1, 1), dtype=bool)), 0)
        half_life = config.COVERAGE_HALF_LIFE_STEPS
        self.assertAlmostEqual(float(coverage.freshness(0, half_life)[0, 0]), 1.0)
        self.assertAlmostEqual(float(coverage.freshness(int(half_life), half_life)[0, 0]), 0.5, places=6)
        self.assertAlmostEqual(float(CoverageMemory((1, 1)).freshness(0, half_life)[0, 0]), 0.0)

    def test_la_meta_di_perlustrazione_e_lontana_abbastanza_da_valere_il_viaggio(self):
        # Il guadagno cresce con i secondi di attesa: se fosse limitato a 1, come la freschezza,
        # dopo pochi secondi nessuna cella varrebbe più di un metro di volo e i droni si
        # fermerebbero dove sono.
        simulation = Simulation(seed=4, config=SimConfig(EXPLORATION_MODE="coverage"))
        simulation.world.fires.clear()
        for _ in range(3000):
            simulation._advance_one_step()
        distances = [float(np.linalg.norm(drone.target - drone.position)) for drone in simulation.drones]
        self.assertGreater(float(np.median(distances)), 2.0)

    def test_la_meta_resta_nella_propria_zona_di_competenza(self):
        config = SimConfig(EXPLORATION_MODE="coverage")
        terrain = ImportanceMap(config)
        me, neighbor = np.array([4.0, 6.0]), np.array([16.0, 6.0])
        destination = choose_patrol_point(terrain, CoverageMemory(terrain.shape), me, [neighbor], 0, config)
        self.assertLessEqual(np.linalg.norm(destination - me), np.linalg.norm(destination - neighbor))

    def test_le_accensioni_spontanee_seguono_l_importanza_del_terreno(self):
        self.assertEqual(simulate_steps(2000).world.ignited_count, 0)
        simulation = simulate_steps(20000, IGNITION_RATE_PER_S=0.5)
        self.assertGreater(simulation.world.ignited_count, 50)
        where_they_started = [simulation.terrain.value_at(*fire.pos) for fire in simulation.world.fires]
        self.assertGreater(np.mean(where_they_started), simulation.terrain.grid.mean())

    def test_con_le_accensioni_attive_la_missione_non_ha_un_esito(self):
        # "Riuscita" non è definita se gli incendi continuano ad accendersi: la misura deve
        # risultare indefinita, non 0% (che sembrerebbe un fallimento dello sciame).
        scenario = experiments.Scenario("senza fine", "", random_fires=False, random_stations=False,
                                        max_time_s=5.0)
        row = experiments.run_one_simulation("senza fine", SimConfig(IGNITION_RATE_PER_S=0.05),
                                             scenario, seed=1)
        self.assertIsNone(row["mission_complete"])
        self.assertIsNone(row["extinction_time_s"])

    def test_con_le_accensioni_attive_nessun_incendio_non_vuol_dire_missione_finita(self):
        simulation = simulate_steps(10, IGNITION_RATE_PER_S=0.1)
        simulation.world.fires.clear()
        self.assertFalse(simulation._all_fires_out())

        quiet = simulate_steps(10)
        quiet.world.fires.clear()
        self.assertTrue(quiet._all_fires_out())


class MisureEStatistica(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        scenario = experiments.Scenario("prova", "", random_fires=False, random_stations=False)
        cls.row = experiments.run_one_simulation("prova", SimConfig(), scenario, seed=42)

    def test_valori_noti(self):
        # Valori misurati su questa versione: devono restare stabili finché non si cambia di
        # proposito la logica dei droni.
        row = self.row
        self.assertTrue(row["mission_complete"])
        self.assertAlmostEqual(row["extinction_time_s"], 67.29)
        self.assertAlmostEqual(row["fire_damage"], 19078.074, places=3)
        self.assertAlmostEqual(row["burning_health_mean"], 283.5202, places=3)
        self.assertAlmostEqual(row["water_fairness"], 0.98916485, places=6)
        self.assertEqual(row["collisions"], 0)
        self.assertEqual(row["drones_lost"], 0)

    def test_ci_sono_tutte_le_misure_e_il_tempo_somma_a_uno(self):
        self.assertEqual([key for key in self.row if key in experiments.METRICS_BY_KEY],
                         [metric.key for metric in experiments.METRICS])
        time_shares = sum(value for key, value in self.row.items() if key.startswith("time_"))
        self.assertAlmostEqual(time_shares, 1.0, places=6)

    def test_intervallo_di_confidenza_di_una_media(self):
        summary = experiments.summarize([1.0, 2.0, 3.0, 4.0], experiments.METRICS_BY_KEY["fire_damage"])
        self.assertAlmostEqual(summary.average, 2.5)
        self.assertAlmostEqual(summary.high - summary.average, 3.182 * 1.2909944 / 2, places=4)

    def test_intervallo_di_confidenza_di_una_percentuale(self):
        summary = experiments.summarize([True] * 10, experiments.METRICS_BY_KEY["mission_complete"])
        self.assertEqual(summary.average, 1.0)
        self.assertLess(summary.low, 1.0)      # dieci successi su dieci non garantiscono il 100%

    def test_la_correzione_di_holm_toglie_i_falsi_allarmi(self):
        # Con decine di confronti nello stesso report, qualche differenza "significativa" esce
        # per puro caso: Holm alza la soglia in proporzione al numero di test.
        differences = [experiments.Difference(0.0, 1.0, p)
                       for p in (0.001, 0.02, 0.03, 0.2, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9)]
        experiments.holm_correction(differences)
        self.assertAlmostEqual(differences[0].corrected_probability, 0.01)
        self.assertEqual(sum(1 for d in differences if d.is_real), 1)
        self.assertEqual(sum(1 for d in differences if d.probability_of_luck < 0.05), 3)

    def test_il_numero_di_incendi_dello_scenario_fisso_viene_rispettato(self):
        self.assertEqual(len(Simulation(config=SimConfig(NUM_FIRES=5)).fires), 5)
        self.assertEqual(len(Simulation(config=SimConfig(NUM_FIRES=2)).fires), 2)

    def test_la_flotta_non_cambia_lo_scenario(self):
        # Le posizioni iniziali dei droni escono da generatori personali: cambiare il numero di
        # droni non deve spostare la sequenza casuale degli incendi.
        streams = []
        for fleet in (8, 12, 20):
            simulation = Simulation(seed=1, config=SimConfig(NUM_DRONES=fleet))
            streams.append(simulation.rng.random())
        self.assertEqual(len(set(streams)), 1)

    def test_quando_una_differenza_e_fortuna(self):
        # Otto coppie tutte a favore della variante: come fare otto teste di fila, 2 casi su 256.
        self.assertAlmostEqual(experiments._probability_of_luck(np.ones(8)), 2 / 256)
        # Nessuna differenza: qualunque combinazione di segni dà lo stesso risultato.
        self.assertEqual(experiments._probability_of_luck(np.zeros(5)), 1.0)
        # Sei coppie discordi tutte dalla stessa parte (misure sì/no).
        self.assertAlmostEqual(experiments._probability_of_luck_yes_no(6, 0), 2 / 64)
        self.assertEqual(experiments._probability_of_luck_yes_no(3, 3), 1.0)

    def test_il_verdetto_conosce_il_verso_giusto(self):
        collisions = experiments.METRICS_BY_KEY["collisions"]
        worse = experiments.Difference(reference_average=0.0, variant_average=3.0, probability_of_luck=0.001)
        self.assertEqual(experiments.verdict(collisions, worse), "peggiore")
        uncertain = experiments.Difference(reference_average=0.0, variant_average=3.0, probability_of_luck=0.2)
        self.assertEqual(experiments.verdict(collisions, uncertain), "")


if __name__ == "__main__":
    unittest.main()
