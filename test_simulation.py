"""
Verifiche automatiche. Si lanciano con:    python -m unittest test_simulation

1. Regressione: la simulazione produce ESATTAMENTE le stesse traiettorie del vecchio xcoverage.py
   (impronte calcolate sul commit 77da2b5). Se si cambia la logica dei droni di proposito, questo test
   fallisce: è normale, le impronte vanno aggiornate sapendo che i risultati precedenti non sono più
   confrontabili con i nuovi.
2. Varianti: con i valori di default non cambiano nulla, e ognuna cambia davvero il comportamento.
3. Misure: sono coerenti tra loro e non alterano la simulazione.
4. Statistica: i calcoli degli esperimenti danno i valori attesi su casi noti.
"""

import hashlib
import unittest

import numpy as np

import experiments
from config import SimConfig, validate_config
from metrics import METRICS, METRICS_BY_KEY, MetricsCollector
from simulation import SwarmSimulation

GOLDEN_STEPS = 3000
GOLDEN = {
    (42, False, False): "c4844169664da35133ed4452b1c4625b71554de4c96e6ef96689a9535d6ae1fa",
    (7, True, True): "6ede2fe2faa7357f3915a2e81e00dd75afb7fc88dd58e2efb567b72f78e6487c",
}


def fingerprint(sim: SwarmSimulation) -> str:
    """Impronta dello stato completo: cambia se cambia anche un solo bit di posizioni, velocità, acqua o incendi."""
    h = hashlib.sha256()
    for d in sim.drones:
        h.update(d.position.tobytes())
        h.update(d.velocity.tobytes())
        h.update(np.float64(d.water).tobytes())
    for f in sim.world.fires:
        h.update(f.pos.tobytes())
        h.update(np.float64(f.health).tobytes())
    return h.hexdigest()


def run(steps: int, seed: int = 42, **params) -> SwarmSimulation:
    sim = SwarmSimulation(seed=seed, cfg=SimConfig().with_overrides(**params))
    for _ in range(steps):
        sim.step()
    return sim


class TestRegression(unittest.TestCase):
    def test_same_trajectories_as_original_script(self):
        for (seed, random_fires, random_stations), expected in GOLDEN.items():
            with self.subTest(seed=seed):
                sim = SwarmSimulation(seed=seed, random_fires=random_fires, random_stations=random_stations)
                metrics = MetricsCollector(sim)          # misurare non deve cambiare nulla
                for _ in range(GOLDEN_STEPS):
                    sim.step()
                    metrics.on_step()
                self.assertEqual(fingerprint(sim), expected)


class TestVariants(unittest.TestCase):
    def test_default_values_change_nothing(self):
        explicit = run(1500, AVOIDANCE_MODE="full", PACKET_LOSS=0.0, SATURATION_BOUNCE=True)
        self.assertEqual(fingerprint(explicit), fingerprint(run(1500)))

    def test_each_variant_changes_behaviour(self):
        base = fingerprint(run(1500))
        for params in ({"AVOIDANCE_MODE": "emergency-only"}, {"AVOIDANCE_MODE": "none"}, {"PACKET_LOSS": 0.3}):
            with self.subTest(**params):
                self.assertNotEqual(fingerprint(run(1500, **params)), base)

    def test_packet_loss_rate(self):
        sim = run(500, PACKET_LOSS=0.25)
        self.assertAlmostEqual(sim.channel.messages_dropped / sim.channel.messages_attempted, 0.25, delta=0.02)

    def test_packet_loss_keeps_same_starting_scenario(self):
        a = SwarmSimulation(seed=5, random_fires=True, cfg=SimConfig(PACKET_LOSS=0.5))
        b = SwarmSimulation(seed=5, random_fires=True)
        self.assertTrue(all(np.array_equal(x.pos, y.pos) for x, y in zip(a.fires, b.fires)))
        self.assertTrue(all(np.array_equal(x.position, y.position) for x, y in zip(a.drones, b.drones)))

    def test_config_checks(self):
        self.assertEqual(validate_config(SimConfig()), [])
        self.assertTrue(validate_config(SimConfig(AVOIDANCE_MODE="boh")))
        self.assertTrue(validate_config(SimConfig(PACKET_LOSS=1.5)))


class TestMetrics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scenario = experiments.Scenario("test", "", random_fires=False, random_stations=False)
        cls.row = experiments.simulate("test", SimConfig(), scenario, seed=42)

    def test_known_values(self):
        # Valori misurati con la prima versione delle metriche: devono restare stabili.
        r = self.row
        self.assertTrue(r["mission_complete"])
        self.assertAlmostEqual(r["extinction_time_s"], 74.34)
        self.assertAlmostEqual(r["fire_damage"], 19533.8235, places=3)
        self.assertAlmostEqual(r["distance_m"], 502.06494, places=4)
        self.assertAlmostEqual(r["water_fairness"], 0.97615064, places=6)
        self.assertAlmostEqual(r["fire_awareness"], 0.77874981, places=6)
        self.assertEqual(r["near_misses"], 1)

    def test_every_metric_present_and_time_budget_sums_to_one(self):
        self.assertEqual([k for k in self.row if k in METRICS_BY_KEY], [m.key for m in METRICS])
        self.assertAlmostEqual(sum(v for k, v in self.row.items() if k.startswith("time_")), 1.0, places=6)


class TestStatistics(unittest.TestCase):
    def test_mean_interval(self):
        s = experiments.summarize([1.0, 2.0, 3.0, 4.0], METRICS_BY_KEY["distance_m"])
        self.assertAlmostEqual(s.value, 2.5)
        self.assertAlmostEqual(s.high - s.value, 3.182 * 1.2909944 / 2, places=4)

    def test_proportion_interval(self):
        s = experiments.summarize([True] * 10, METRICS_BY_KEY["mission_complete"])
        self.assertEqual(s.value, 1.0)
        self.assertLess(s.low, 1.0)

    def test_paired_tests(self):
        self.assertAlmostEqual(experiments._sign_flip_test(np.ones(8)), 2 / 256)
        self.assertEqual(experiments._sign_flip_test(np.zeros(5)), 1.0)
        self.assertAlmostEqual(experiments._mcnemar(6, 0), 2 / 64)
        self.assertEqual(experiments._mcnemar(3, 3), 1.0)

    def test_verdict_direction(self):
        collisions = METRICS_BY_KEY["collisions"]
        worse = experiments.Difference(reference=0.0, variant=3.0, p_value=0.001)
        self.assertEqual(experiments.verdict(collisions, worse), "peggiore")
        not_significant = experiments.Difference(reference=0.0, variant=3.0, p_value=0.2)
        self.assertEqual(experiments.verdict(collisions, not_significant), "")


if __name__ == "__main__":
    unittest.main()
