"""
X-Coverage — simulazione 2D di uno sciame decentralizzato di droni antincendio.

    python main.py                                   # finestra Pygame
    python main.py --comm --show-vectors             # con rete radio e vettori di forza
    python main.py --random-fires --random-stations --seed 7
    python main.py --headless --steps 30000 --log    # senza finestra, statistiche su terminale

Per misurare il sistema e confrontarne delle varianti: python experiments.py

File del progetto:
    config.py       tutti i parametri
    world.py        il mondo fisico: incendi, canale radio, sensori
    drone.py        cosa decide e come si muove ogni drone
    simulation.py   fa avanzare tutti i droni di un passo
    renderer.py     la finestra
    metrics.py      le misure (usate solo da experiments.py)
    experiments.py  gli esperimenti
"""

import argparse
import sys

from config import DEFAULT_CONFIG, validate_config
from simulation import SwarmSimulation


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulazione 2D di swarm decentralizzato di droni antincendio")
    parser.add_argument("--show-vectors", action=argparse.BooleanOptionalAction, default=False,
                        help="Disegna i vettori di forza e di collision avoidance (tasto V)")
    parser.add_argument("--comm", action=argparse.BooleanOptionalAction, default=False,
                        help="Mostra le connessioni di comunicazione tra droni (tasto C)")
    parser.add_argument("--random-fires", action="store_true", help="Genera NUM_FIRES incendi casuali")
    parser.add_argument("--random-stations", action="store_true", help="Genera NUM_WATER_STATIONS stazioni casuali")
    parser.add_argument("--log", action="store_true", help="Stampa ogni collisione")
    parser.add_argument("--seed", type=int, default=DEFAULT_CONFIG.RANDOM_SEED, help="Seme casuale della simulazione")
    parser.add_argument("--headless", action="store_true", help="Esegue senza finestra e stampa le statistiche")
    parser.add_argument("--steps", type=int, default=30000, help="Numero di step in modalità --headless")
    return parser


def run_headless(sim: SwarmSimulation, steps: int) -> None:
    report_every = int(round(10.0 / sim.cfg.SIM_TIME_STEP))
    for _ in range(steps):
        sim.step()
        if sim.step_count % report_every == 0:
            print(sim.summary(), flush=True)
    print("FINE:", sim.summary())


def main() -> None:
    for problem in validate_config(DEFAULT_CONFIG):
        print(f"[config] ATTENZIONE: {problem}", file=sys.stderr)

    args = build_argument_parser().parse_args()
    sim = SwarmSimulation(seed=args.seed, random_fires=args.random_fires, random_stations=args.random_stations,
                          log_collisions=args.log)
    if args.headless:
        run_headless(sim, args.steps)
    else:
        from renderer import run_interactive  # import locale: --headless non richiede Pygame
        run_interactive(sim, args.show_vectors, args.comm)


if __name__ == "__main__":
    main()
