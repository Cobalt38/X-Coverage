"""
LE VERIFICHE AUTOMATICHE: si lanciano con   python main.py test

Servono a poter cambiare il codice senza paura. Sono divise in cinque gruppi, ognuno con uno scopo:

    1. Regressione     la simulazione produce ESATTAMENTE le stesse traiettorie di prima. Se si
                       cambia di proposito la logica dei droni questo test fallisce: è normale, va
                       aggiornata l'impronta sapendo che i risultati vecchi non sono più confrontabili.
    2. Varianti        ogni interruttore fa davvero qualcosa, e lasciandolo com'è non cambia nulla.
    3. Urti            un drone che si scontra si rompe, e nel modo giusto a seconda dell'impatto.
    4. Perlustrazione  la copertura persistente e le accensioni spontanee si comportano come descritto.
    5. Misure e statistica   i numeri del report sono calcolati correttamente su casi noti.
"""

import hashlib
import unittest

import numpy as np

import experiments
from drone import CoverageMemory, choose_patrol_point
from simulation import MissionOutcome, Simulation
from world import DroneStatus, ImportanceMap, SimConfig, validate_config

GOLDEN_STEPS = 3000

# Impronte calcolate sulla versione di riferimento del codice: riassumono in un'unica stringa
# posizioni, velocità, acqua e incendi dopo GOLDEN_STEPS passi.
GOLDEN = {
    (42, False, False): "c4844169664da35133ed4452b1c4625b71554de4c96e6ef96689a9535d6ae1fa",
    (7, True, True): "6ede2fe2faa7357f3915a2e81e00dd75afb7fc88dd58e2efb567b72f78e6487c",
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
        self.assertAlmostEqual(result.elapsed_s, 74.34, places=2)


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
        # Misurati sulla prima versione delle metriche: devono restare stabili nel tempo.
        row = self.row
        self.assertTrue(row["mission_complete"])
        self.assertAlmostEqual(row["extinction_time_s"], 74.34)
        self.assertAlmostEqual(row["fire_damage"], 19533.8235, places=3)
        self.assertAlmostEqual(row["distance_m"], 502.06494, places=4)
        self.assertAlmostEqual(row["water_fairness"], 0.97615064, places=6)
        self.assertEqual(row["collisions"], 0)
        self.assertEqual(row["drones_lost"], 0)

    def test_ci_sono_tutte_le_misure_e_il_tempo_somma_a_uno(self):
        self.assertEqual([key for key in self.row if key in experiments.METRICS_BY_KEY],
                         [metric.key for metric in experiments.METRICS])
        time_shares = sum(value for key, value in self.row.items() if key.startswith("time_"))
        self.assertAlmostEqual(time_shares, 1.0, places=6)

    def test_intervallo_di_confidenza_di_una_media(self):
        summary = experiments.summarize([1.0, 2.0, 3.0, 4.0], experiments.METRICS_BY_KEY["distance_m"])
        self.assertAlmostEqual(summary.average, 2.5)
        self.assertAlmostEqual(summary.high - summary.average, 3.182 * 1.2909944 / 2, places=4)

    def test_intervallo_di_confidenza_di_una_percentuale(self):
        summary = experiments.summarize([True] * 10, experiments.METRICS_BY_KEY["mission_complete"])
        self.assertEqual(summary.average, 1.0)
        self.assertLess(summary.low, 1.0)      # dieci successi su dieci non garantiscono il 100%

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
