"""
Esperimenti: capire cosa conta davvero nel comportamento dello sciame.

Un esperimento risponde a UNA domanda confrontando delle VARIANTI del sistema (es. "con" e "senza"
un certo meccanismo). Il processo, che questo file segue sezione per sezione, è:

    1. DEFINIZIONE   Uno scenario (le condizioni di partenza) e le varianti da confrontare.
                     Gli esperimenti disponibili sono nel dizionario EXPERIMENTS, più sotto:
                     per crearne uno nuovo basta aggiungere una voce.
    2. ESECUZIONE    Ogni variante viene simulata N volte, sempre con gli stessi N seed.
                     Il seed decide tutto ciò che è casuale (dove nascono gli incendi, dove partono
                     i droni...), quindi con lo stesso seed tutte le varianti partono da situazioni
                     IDENTICHE: le differenze nei risultati sono dovute solo a ciò che cambia tra le varianti.
    3. RIASSUNTO     Per ogni variante e metrica: la media sulle N simulazioni e il suo intervallo di
                     confidenza al 95% (dove si trova, con buona probabilità, la media "vera").
    4. CONFRONTO     Ogni variante contro la prima (il riferimento), simulazione per simulazione.
                     Un test statistico dice se la differenza è reale o può essere dovuta al caso.
    5. REPORT        results/<esperimento>_<data>/report.md   da leggere
                     results/<esperimento>_<data>/runs.csv    una riga per simulazione, per grafici

Uso:
    python experiments.py                    # elenca gli esperimenti
    python experiments.py ablation           # esegue "ablation" (30 simulazioni per variante)
    python experiments.py ablation --runs 5  # prova veloce (risultati poco affidabili)
"""

import argparse
import csv
import datetime
import itertools
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from config import DEFAULT_CONFIG, SimConfig, validate_config
from metrics import METRICS, METRICS_BY_KEY, Metric, MetricsCollector
from simulation import SwarmSimulation

RESULTS_DIR = "results"
FIRST_SEED = 1          # le simulazioni usano i seed 1, 2, ..., N
SIGNIFICANCE = 0.05     # soglia del p-value sotto cui una differenza è considerata reale


# ============================================================
# 1. DEFINIZIONE
# ============================================================

@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    params: Dict[str, Any] = field(default_factory=dict)  # parametri di config.py diversi dal default
    random_fires: bool = True
    random_stations: bool = True
    max_time_s: float = 300.0          # durata massima di una simulazione
    max_active_fires: int = 40         # oltre questa soglia la simulazione si ferma e conta come fallita


@dataclass(frozen=True)
class Variant:
    name: str
    params: Dict[str, Any] = field(default_factory=dict)  # si aggiungono (e prevalgono) su quelli dello scenario


@dataclass(frozen=True)
class Experiment:
    question: str
    scenario: Scenario
    variants: List[Variant]            # la prima è il riferimento con cui si confrontano le altre


SCENARIOS = {
    "facile": Scenario(
        "facile", "3 incendi e 3 stazioni in posizioni casuali, parametri di default. "
                  "Lo sciame riesce quasi sempre: utile come controllo, poco per distinguere le varianti."),
    "critico": Scenario(
        "critico", "5 incendi in posizioni casuali che crescono più in fretta (0.6 vita/s). "
                   "Lo sciame riesce circa 2 volte su 3: è in questa zona che si vedono le differenze.",
        params={"NUM_FIRES": 5, "FIRE_GROWTH_RATE": 0.6}, max_time_s=600.0),
}

EXPERIMENTS = {
    "ablation": Experiment(
        "Quanto contribuisce ciascun meccanismo al successo e alla sicurezza dello sciame?",
        SCENARIOS["critico"],
        [Variant("completo"),
         Variant("senza predizione delle collisioni", {"AVOIDANCE_MODE": "emergency-only"}),
         Variant("senza evitamento delle collisioni", {"AVOIDANCE_MODE": "none"}),
         Variant("radio con 50% di messaggi persi", {"PACKET_LOSS": 0.5}),
         Variant("senza rimbalzo dagli incendi pieni", {"SATURATION_BOUNCE": False})]),
    "radio": Experiment(
        "Quanto peggiora lo sciame quando la radio perde messaggi?",
        SCENARIOS["critico"],
        [Variant(f"{p:.0%} di messaggi persi", {"PACKET_LOSS": p}) for p in (0.0, 0.1, 0.2, 0.3, 0.5)]),
    "difficolta": Experiment(
        "Fino a che velocità di crescita degli incendi lo sciame riesce a contenerli?",
        SCENARIOS["critico"],
        [Variant(f"crescita {g} vita/s", {"FIRE_GROWTH_RATE": g}) for g in (0.5, 0.6, 0.7, 0.8)]),
    "flotta": Experiment(
        "Quanti droni servono? Come cambiano risultati e sicurezza con la dimensione dello sciame?",
        SCENARIOS["critico"],
        [Variant(f"{n} droni", {"NUM_DRONES": n}) for n in (12, 8, 16, 20)]),
}


def variant_config(experiment: Experiment, variant: Variant) -> SimConfig:
    return DEFAULT_CONFIG.with_overrides(**{**experiment.scenario.params, **variant.params})


# ============================================================
# 2. ESECUZIONE
# ============================================================

def simulate(variant_name: str, cfg: SimConfig, scenario: Scenario, seed: int) -> Dict[str, Any]:
    """Una simulazione completa, misurata. Restituisce una riga: variante, seed, durata e tutte le metriche.

    Si ferma quando succede la prima di queste cose:
        - tutti gli incendi sono spenti (missione riuscita; da lì in poi non può cambiare nulla);
        - gli incendi accesi superano scenario.max_active_fires (fuori controllo: fallita);
        - è trascorso scenario.max_time_s (non conclusa).
    """
    sim = SwarmSimulation(seed=seed, random_fires=scenario.random_fires,
                          random_stations=scenario.random_stations, cfg=cfg)
    metrics = MetricsCollector(sim)
    for _ in range(int(round(scenario.max_time_s / cfg.SIM_TIME_STEP))):
        sim.step()
        metrics.on_step()
        if sim.all_fires_extinguished:
            break
        if len(sim.world.fires) > scenario.max_active_fires:
            metrics.fire_overrun = True
            break
    return {"variant": variant_name, "seed": seed, "duration_s": round(sim.sim_time, 2), **metrics.finalize()}


def run_all(experiment: Experiment, runs: int, workers: Optional[int]) -> List[Dict[str, Any]]:
    """Tutte le simulazioni dell'esperimento (varianti × seed), in parallelo sui core disponibili."""
    seeds = range(FIRST_SEED, FIRST_SEED + runs)
    jobs = [(v.name, variant_config(experiment, v), experiment.scenario, seed)
            for v in experiment.variants for seed in seeds]
    for variant in experiment.variants:
        for problem in validate_config(variant_config(experiment, variant)):
            print(f"  ATTENZIONE [{variant.name}]: {problem}")

    rows = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(simulate, *job) for job in jobs]
        for done, future in enumerate(as_completed(futures), start=1):
            row = future.result()
            rows.append(row)
            print(f"  [{done}/{len(jobs)}] {row['variant']}, seed {row['seed']}: {describe_outcome(row)}", flush=True)

    order = {v.name: i for i, v in enumerate(experiment.variants)}
    rows.sort(key=lambda r: (order[r["variant"]], r["seed"]))
    return rows


def describe_outcome(row: Dict[str, Any]) -> str:
    if row["mission_complete"]:
        return f"riuscita in {row['extinction_time_s']:.0f} s"
    if row["fire_overrun"]:
        return f"fallita, incendi fuori controllo dopo {row['duration_s']:.0f} s"
    return "non conclusa entro il tempo massimo"


# ============================================================
# 3. RIASSUNTO: media e intervallo di confidenza
# ============================================================

@dataclass
class Summary:
    value: Optional[float]   # media (per le metriche sì/no: quota di "sì")
    low: Optional[float]     # intervallo di confidenza al 95%
    high: Optional[float]
    n: int                   # simulazioni in cui la metrica è definita


def summarize(values: List[Any], metric: Metric) -> Summary:
    """Media e intervallo di confidenza al 95% di una metrica su più simulazioni.

    L'intervallo dice quanto ci si può fidare della media: con più simulazioni si stringe.
        - metriche numeriche: intervallo della t di Student (media ± t · deviazione standard / √n);
        - metriche sì/no:     intervallo di Wilson, corretto anche quando i "sì" sono quasi 0% o 100%.
    """
    values = [v for v in values if v is not None]
    n = len(values)
    if n == 0:
        return Summary(None, None, None, 0)
    if metric.unit == "sì/no":
        p = sum(1 for v in values if v) / n
        z = 1.959964
        center = (p + z * z / (2 * n)) / (1 + z * z / n)
        half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
        return Summary(p, max(0.0, center - half), min(1.0, center + half), n)
    x = np.asarray(values, dtype=float)
    mean = float(x.mean())
    if n == 1:
        return Summary(mean, None, None, 1)
    half = _t_975(n - 1) * float(x.std(ddof=1)) / math.sqrt(n)
    # Tutte le metriche sono >= 0 (e le percentuali <= 100%): l'intervallo viene limitato ai valori possibili.
    high = mean + half if metric.unit != "%" else min(1.0, mean + half)
    return Summary(mean, max(0.0, mean - half), high, n)


def _t_975(df: int) -> float:
    """Quantile 97.5% della t di Student (tabella fino a 30 gradi di libertà, poi approssimazione)."""
    table = [12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179, 2.160, 2.145,
             2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048,
             2.045, 2.042]
    if df <= len(table):
        return table[df - 1]
    z = 1.959964
    return z + (z ** 3 + z) / (4 * df) + (5 * z ** 5 + 16 * z ** 3 + 3 * z) / (96 * df ** 2)


# ============================================================
# 4. CONFRONTO con il riferimento, simulazione per simulazione
# ============================================================

@dataclass
class Difference:
    reference: float    # media del riferimento sulle coppie confrontate
    variant: float      # media della variante sulle stesse coppie
    p_value: float      # probabilità di una differenza così grande se le due varianti fossero equivalenti

    @property
    def significant(self) -> bool:
        return self.p_value < SIGNIFICANCE


def compare(ref_rows: List[Dict[str, Any]], var_rows: List[Dict[str, Any]], metric: Metric) -> Optional[Difference]:
    """Confronto APPAIATO: la simulazione con seed 7 del riferimento contro quella con seed 7 della variante.

    Confrontare coppie che partono identiche elimina la "fortuna dello scenario" e rende il
    confronto molto più sensibile di un semplice confronto tra medie.
        - metriche numeriche: test di permutazione dei segni. Se le varianti fossero equivalenti, il segno
          di ogni differenza (variante - riferimento) sarebbe casuale: si calcola quanto spesso, invertendo
          i segni a caso, si ottiene una differenza media grande almeno quanto quella osservata;
        - metriche sì/no: test di McNemar. Conta solo le coppie discordanti (una riesce, l'altra no):
          se le varianti fossero equivalenti, sarebbero divise circa a metà.
    """
    ref = {r["seed"]: r[metric.key] for r in ref_rows}
    pairs = [(ref[r["seed"]], r[metric.key]) for r in var_rows
             if r["seed"] in ref and ref[r["seed"]] is not None and r[metric.key] is not None]
    if not pairs:
        return None
    a = np.array([float(x) for x, _ in pairs])
    b = np.array([float(y) for _, y in pairs])
    if metric.unit == "sì/no":
        only_ref = int(np.sum((a == 1) & (b == 0)))
        only_var = int(np.sum((a == 0) & (b == 1)))
        p = _mcnemar(only_ref, only_var)
    else:
        p = _sign_flip_test(b - a)
    return Difference(float(a.mean()), float(b.mean()), p)


def _sign_flip_test(diffs: np.ndarray, permutations: int = 20000) -> float:
    if np.allclose(diffs, 0.0):
        return 1.0
    n = len(diffs)
    if n <= 16:   # poche coppie: si provano tutte le combinazioni di segni
        signs = np.array(list(itertools.product((-1.0, 1.0), repeat=n)))
    else:         # molte coppie: un campione casuale (fisso, per risultati riproducibili)
        signs = np.random.default_rng(0).choice((-1.0, 1.0), size=(permutations, n))
    null = np.abs((signs * diffs).mean(axis=1))
    return float(np.mean(null >= abs(diffs.mean()) - 1e-12))


def _mcnemar(only_ref: int, only_var: int) -> float:
    n = only_ref + only_var
    if n == 0:
        return 1.0
    tail = sum(math.comb(n, i) for i in range(min(only_ref, only_var) + 1)) / 2 ** n
    return min(1.0, 2.0 * tail)


def verdict(metric: Metric, diff: Optional[Difference]) -> str:
    """'migliore' / 'peggiore' / 'diverso' se la differenza è significativa, altrimenti ''."""
    if diff is None or not diff.significant or diff.variant == diff.reference:
        return ""
    if not metric.better:
        return "diverso"
    higher = diff.variant > diff.reference
    return "migliore" if higher == (metric.better == "alto") else "peggiore"


# ============================================================
# 5. REPORT
# ============================================================

MARKERS = {"migliore": " ▲", "peggiore": " ▼", "diverso": " ◆", "": ""}

# Sotto questa soglia un confronto appaiato difficilmente raggiunge la significatività
# (es. con 4 coppie il p-value più piccolo possibile è 0.125): il report lo segnala.
FEW_RUNS = 20

FEW_RUNS_WARNING = ("⚠ Poche simulazioni per variante: anche differenze grandi possono non risultare "
                    "significative. Per conclusioni affidabili usare almeno {n} simulazioni (--runs {n}).")

# Metriche citate nel riassunto "In breve" (tutte le altre sono nelle tabelle del report)
HEADLINE = ["mission_complete", "extinction_time_s", "fire_damage", "collisions", "emergency_fraction",
            "water_fairness", "fire_awareness"]


def fmt(metric: Metric, value: Optional[float]) -> str:
    """Un numero leggibile: percentuali per % e sì/no, cifre sensate per il resto."""
    if value is None:
        return "–"
    if metric.unit in ("%", "sì/no"):
        return f"{value * 100:.0f}%" if value >= 0.1 or value == 0 else f"{value * 100:.1f}%"
    if abs(value) >= 1000:
        return f"{value:,.0f}".replace(",", " ")      # 127 948: niente punto, che sembrerebbe un decimale
    return f"{value:.3g}"


def unit_label(metric: Metric) -> str:
    return "" if metric.unit in ("", "%", "sì/no") else f" [{metric.unit}]"


def headline_sentence(experiment: Experiment, variant: Variant, rows_by_variant: Dict[str, list]) -> str:
    reference = experiment.variants[0].name
    changes, others = [], 0
    for metric in METRICS:
        diff = compare(rows_by_variant[reference], rows_by_variant[variant.name], metric)
        v = verdict(metric, diff)
        if not v:
            continue
        if metric.key in HEADLINE:
            changes.append(f"{metric.label.lower()} {fmt(metric, diff.reference)} → {fmt(metric, diff.variant)}"
                           f" ({v})")
        else:
            others += 1
    if not changes and not others:
        return f"- **{variant.name}**: nessuna differenza significativa rispetto a «{reference}»."
    text = f"- **{variant.name}**: " + ("; ".join(changes) if changes else "nessun cambiamento tra le metriche principali")
    if others:
        text += (". Cambia in modo significativo anche un'altra metrica (vedi tabelle)" if others == 1 else
                 f". Cambiano in modo significativo anche altre {others} metriche (vedi tabelle)")
    return text + "."


def write_report(name: str, experiment: Experiment, rows: List[Dict[str, Any]], runs: int,
                 elapsed_s: float, folder: str) -> str:
    rows_by_variant = {v.name: [r for r in rows if r["variant"] == v.name] for v in experiment.variants}
    reference = experiment.variants[0].name
    scenario = experiment.scenario
    lines = [
        f"# Esperimento «{name}»",
        "",
        f"**Domanda.** {experiment.question}",
        "",
        f"**Scenario «{scenario.name}».** {scenario.description}",
        f"Parametri diversi dal default: {_params_text(scenario.params)}. "
        f"Ogni simulazione dura al massimo {scenario.max_time_s:.0f} s ed è fermata (e conta come fallita) "
        f"se gli incendi accesi superano {scenario.max_active_fires}.",
        "",
        "**Varianti.** " + "; ".join(f"«{v.name}» ({_params_text(v.params)})" for v in experiment.variants)
        + f". Il riferimento è «{reference}».",
        "",
        f"**Metodo.** {runs} simulazioni per variante, con i seed {FIRST_SEED}–{FIRST_SEED + runs - 1}, "
        f"uguali per tutte le varianti. Eseguito il {datetime.datetime.now():%d/%m/%Y alle %H:%M} "
        f"in {elapsed_s / 60:.1f} minuti (codice: commit {_git_commit()}).",
        "",
        "## In breve",
        "",
        *([FEW_RUNS_WARNING.format(n=FEW_RUNS), ""] if runs < FEW_RUNS else []),
        *[headline_sentence(experiment, v, rows_by_variant) for v in experiment.variants[1:]],
        "",
        "## Risultati",
        "",
        "Ogni cella: media sulle simulazioni e, tra parentesi, l'intervallo di confidenza al 95%. "
        f"▲ / ▼ = significativamente migliore / peggiore di «{reference}»; "
        "◆ = significativamente diverso (per metriche senza un valore \"migliore\").",
    ]

    for group in dict.fromkeys(m.group for m in METRICS):
        lines += ["", f"### {group}", "",
                  "| Metrica | " + " | ".join(v.name for v in experiment.variants) + " |",
                  "|---|" + "---|" * len(experiment.variants)]
        for metric in (m for m in METRICS if m.group == group):
            cells = []
            for v in experiment.variants:
                s = summarize([r[metric.key] for r in rows_by_variant[v.name]], metric)
                cell = fmt(metric, s.value)
                if s.low is not None:
                    cell += f" ({fmt(metric, s.low)}–{fmt(metric, s.high)})"
                if v.name != reference:
                    cell += MARKERS[verdict(metric, compare(rows_by_variant[reference], rows_by_variant[v.name], metric))]
                cells.append(cell)
            lines.append(f"| {metric.label}{unit_label(metric)} | " + " | ".join(cells) + " |")

    lines += [
        "",
        "## Come leggere questi numeri",
        "",
        "- **Intervallo di confidenza al 95%**: se si ripetesse l'esperimento con infinite simulazioni, la media "
        "cadrebbe quasi certamente in quell'intervallo. Intervalli larghi = servono più simulazioni.",
        f"- **▲ ▼ ◆**: la differenza dal riferimento supera il test statistico (p < {SIGNIFICANCE}), cioè "
        "difficilmente è dovuta al caso. Le simulazioni sono confrontate a coppie con lo stesso seed "
        "(test di permutazione dei segni; test di McNemar per le metriche sì/no). "
        "Significativo non vuol dire grande: guardare anche di quanto cambia la media.",
        "- **Tempo per spegnere tutto** è calcolato solo sulle missioni riuscite: va letto insieme a "
        "«Missione riuscita».",
        "- Le simulazioni si fermano quando la missione riesce o gli incendi vanno fuori controllo, quindi hanno "
        "durate diverse: danno, distanza e sforzo accumulati vanno confrontati con cautela tra varianti con "
        "esiti molto diversi.",
        "",
        "## Significato delle metriche",
        "",
        *[f"- **{m.label}** (`{m.key}`): {m.description}" for m in METRICS],
        "",
        "I dati di ogni singola simulazione sono in `runs.csv` (una riga per simulazione, colonne = nomi tra "
        "parentesi qui sopra).",
    ]
    path = os.path.join(folder, "report.md")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    return path


def write_runs_csv(rows: List[Dict[str, Any]], folder: str) -> str:
    path = os.path.join(folder, "runs.csv")
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows({k: ("" if v is None else v) for k, v in row.items()} for row in rows)
    return path


def _params_text(params: Dict[str, Any]) -> str:
    return ", ".join(f"{k} = {v}" for k, v in params.items()) or "nessuno"


def _git_commit() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=5)
        dirty = subprocess.run(["git", "status", "--porcelain"], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return "sconosciuto"
    if out.returncode != 0:
        return "sconosciuto"
    return out.stdout.strip() + (" con modifiche non salvate" if dirty.stdout.strip() else "")


# ============================================================
# Avvio
# ============================================================

def main() -> None:
    # Il terminale di Windows può usare una codifica senza caratteri come → ▲ ▼: si forza UTF-8.
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description="Esperimenti sullo sciame di droni. Senza argomenti elenca gli esperimenti.")
    parser.add_argument("experiment", nargs="?", choices=list(EXPERIMENTS), help="Esperimento da eseguire")
    parser.add_argument("--runs", type=int, default=30, help="Simulazioni per variante (default 30)")
    parser.add_argument("--workers", type=int, default=None, help="Simulazioni in parallelo (default: tutti i core)")
    args = parser.parse_args()

    if args.experiment is None:
        print("Esperimenti disponibili (python experiments.py NOME):\n")
        for name, exp in EXPERIMENTS.items():
            print(f"  {name:<12} {exp.question}")
            print(f"  {'':<12} varianti: {', '.join(v.name for v in exp.variants)}\n")
        return

    experiment = EXPERIMENTS[args.experiment]
    print(f"Esperimento «{args.experiment}»: {experiment.question}")
    print(f"{len(experiment.variants)} varianti × {args.runs} simulazioni, scenario «{experiment.scenario.name}»\n")
    start = time.perf_counter()
    rows = run_all(experiment, args.runs, args.workers)
    elapsed = time.perf_counter() - start

    folder = os.path.join(RESULTS_DIR, f"{args.experiment}_{datetime.datetime.now():%Y%m%d-%H%M}")
    os.makedirs(folder, exist_ok=True)
    report = write_report(args.experiment, experiment, rows, args.runs, elapsed, folder)
    data = write_runs_csv(rows, folder)

    rows_by_variant = {v.name: [r for r in rows if r["variant"] == v.name] for v in experiment.variants}
    print(f"\nIn breve (rispetto a «{experiment.variants[0].name}»):")
    for variant in experiment.variants[1:]:
        print(headline_sentence(experiment, variant, rows_by_variant).replace("**", ""))
    if args.runs < FEW_RUNS:
        print(FEW_RUNS_WARNING.format(n=FEW_RUNS))
    print(f"\nReport completo: {report}\nDati grezzi:     {data}")


if __name__ == "__main__":
    main()
