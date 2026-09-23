"""
X-Coverage — sciame decentralizzato di droni antincendio.

PUNTO D'INGRESSO UNICO. Tutto si lancia da qui:

    python main.py                                  apre la finestra e guarda lo sciame lavorare
    python main.py --random-fires --seed 7          un'altra situazione di partenza
    python main.py --headless --time 120            nessuna finestra, solo il risultato
    python main.py experiment                       elenca gli esperimenti disponibili
    python main.py experiment ablation --runs 30    esegue un esperimento e scrive il report
    python main.py test                             verifica che tutto funzioni ancora

I file del progetto, nell'ordine in cui conviene leggerli:

    world.py        i parametri e il mondo fisico: incendi, terreno, radio, sensori
    drone.py        l'agente: cosa sa, cosa decide, come vola
    simulation.py   il giro di ogni passo, gli urti, e la finestra
    experiments.py  le misure, gli esperimenti e i report
    tests.py        le verifiche automatiche

Nella finestra: P pausa, V vettori delle forze, C collegamenti radio, H mappa del terreno, Q esci.
"""

import argparse
import sys
from typing import List, Optional

from simulation import MissionResult, Simulation, run_with_window
from world import DEFAULT_CONFIG, validate_config

COMMANDS = ("run", "experiment", "test")


class ProgressPrinter:
    """Osservatore che stampa una riga di stato ogni tot secondi simulati (modalità senza finestra)."""

    def __init__(self, every_seconds: float = 10.0):
        self.every_seconds = every_seconds
        self._next_print = every_seconds

    def after_step(self, simulation: Simulation) -> None:
        if simulation.sim_time >= self._next_print:
            self._next_print += self.every_seconds
            print(simulation.summary(), flush=True)


EXAMPLES = """esempi:
  python main.py                                  apre la finestra
  python main.py --seed 7 --random-fires          un'altra situazione di partenza
  python main.py --headless --time 120            nessuna finestra, solo il risultato
  python main.py experiment                       elenca gli esperimenti
  python main.py experiment ablation --runs 30    esegue un esperimento
  python main.py test                             verifiche automatiche

Nella finestra: P pausa, V forze, C radio, H mappa del terreno, Q esci. Tutto si accende e si
spegne da lì, non servono opzioni."""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sciame decentralizzato di droni antincendio.",
                                     epilog=EXAMPLES,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command")

    run = commands.add_parser("run", help="Esegue una simulazione (è ciò che accade senza argomenti)")
    run.add_argument("--seed", type=int, default=DEFAULT_CONFIG.RANDOM_SEED,
                     help="Cambia la situazione di partenza (posizioni di droni, incendi, stazioni)")
    run.add_argument("--random-fires", action="store_true", help="Incendi in posizioni casuali")
    run.add_argument("--random-stations", action="store_true", help="Stazioni idriche in posizioni casuali")
    run.add_argument("--headless", action="store_true", help="Nessuna finestra: stampa solo lo stato")
    run.add_argument("--time", type=float, default=300.0,
                     help="Durata massima in secondi simulati (solo con --headless)")
    run.add_argument("--log", action="store_true", help="Stampa ogni urto tra droni")

    experiment = commands.add_parser("experiment", help="Esegue un esperimento e scrive il report")
    experiment.add_argument("name", nargs="?", help="Nome dell'esperimento (senza nome: li elenca)")
    experiment.add_argument("--runs", type=int, default=30, help="Simulazioni per variante (default 30)")
    experiment.add_argument("--workers", type=int, default=None,
                            help="Quante simulazioni in parallelo (default: tutti i core)")

    commands.add_parser("test", help="Esegue le verifiche automatiche del progetto")
    return parser


def run_simulation(args: argparse.Namespace) -> MissionResult:
    """Prepara lo scenario e lo fa girare: con la finestra, oppure in silenzio."""
    simulation = Simulation(seed=args.seed, random_fires=args.random_fires,
                            random_stations=args.random_stations, log_collisions=args.log)
    if not args.headless:
        return run_with_window(simulation)

    result = simulation.run(max_time_s=args.time, watchers=[ProgressPrinter()])
    print(f"\nFINE: {result.describe()}")
    print(f"Incendi spenti: {result.fires_extinguished}   urti: {result.collisions}   "
          f"droni ancora in volo: {result.drones_flying} su {len(simulation.drones)}")
    return result


def main(argv: Optional[List[str]] = None) -> None:
    # Il terminale di Windows può usare una codifica senza i caratteri → ▲ ▼: si forza UTF-8.
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

    for problem in validate_config(DEFAULT_CONFIG):
        print(f"[parametri] ATTENZIONE: {problem}", file=sys.stderr)

    arguments = list(sys.argv[1:] if argv is None else argv)
    asking_for_help = bool({"-h", "--help"} & set(arguments))
    if (not arguments or arguments[0] not in COMMANDS) and not asking_for_help:
        arguments.insert(0, "run")    # "python main.py --seed 3" vale come "python main.py run --seed 3"
    args = build_parser().parse_args(arguments)

    if args.command == "test":
        import unittest
        import tests
        unittest.main(module=tests, argv=["tests"], exit=False, verbosity=2)
        return

    if args.command == "experiment":
        import experiments
        if args.name is None:
            experiments.list_experiments()
        elif args.name not in experiments.EXPERIMENTS:
            print(f"Esperimento «{args.name}» inesistente.\n")
            experiments.list_experiments()
        else:
            experiments.run_and_report(args.name, args.runs, args.workers)
        return

    run_simulation(args)


if __name__ == "__main__":
    main()
