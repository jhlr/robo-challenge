"""Monte Carlo: compara politicas na arena simulada. Uso: python3 sim/run.py [--episodes 200]"""
from __future__ import annotations

import argparse
import csv
import random
import statistics
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from arena import Arena  # noqa: E402
from engine import Engine, Params, Policy, Result  # noqa: E402
from policies import INF, NearestFirst, Planner, RandomWalk  # noqa: E402


def make_policies() -> list[Policy]:
    return [
        RandomWalk("RandomWalk (sem memoria)"),
        NearestFirst("Nearest, memoria decide", tau=INF, gated=True, sensor_first=False),
        NearestFirst("Nearest, sensor decide", tau=INF, gated=True, sensor_first=True),
        Planner("Planner tau=INF, memoria decide", tau=INF, gated=True, sensor_first=False),
        Planner("Planner tau=INF, sensor decide", tau=INF, gated=True, sensor_first=True),
        Planner("Planner tau=60, memoria decide", tau=60.0, gated=True, sensor_first=False),
        Planner("Planner tau=60, sensor decide", tau=60.0, gated=True, sensor_first=True),
        Planner("Planner tau=30, sensor decide", tau=30.0, gated=True, sensor_first=True),
        Planner("Planner tau=60, memoria decide, 30% crenca errada", tau=60.0, gated=True, sensor_first=False, corrupt=0.3),
        Planner("Planner tau=60, sensor decide, 30% crenca errada", tau=60.0, gated=True, sensor_first=True, corrupt=0.3),
    ]


def run_episode(policy: Policy, seed: int, params: Params) -> Result:
    arena = Arena.random(random.Random(seed))
    return Engine(arena, policy, params, rng=random.Random(seed * 7 + 1), opp_rng=random.Random(seed * 13 + 5)).run()


def evaluate(params: Params, episodes: int) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    for pol in make_policies():
        results = [run_episode(pol, seed, params) for seed in range(episodes)]
        scores = [r.score for r in results]
        rows.append({
            "policy": pol.name,
            "score_mean": round(statistics.fmean(scores), 1),
            "score_sem": round(statistics.stdev(scores) / len(scores) ** 0.5, 1),
            "delivered": round(statistics.fmean(r.stats.delivered for r in results), 2),
            "identified": round(statistics.fmean(r.stats.identified for r in results), 2),
            "lost": round(statistics.fmean(r.stats.lost for r in results), 2),
            "wasted": round(statistics.fmean(r.stats.wasted for r in results), 1),
            "miss_events": round(statistics.fmean(r.stats.miss_events for r in results), 1),
        })
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--opp-rate", type=float, default=None, help="eventos do oponente por segundo")
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()
    params = Params() if args.opp_rate is None else replace(Params(), opp_event_rate=args.opp_rate)
    rows = evaluate(params, args.episodes)
    cols = list(rows[0])
    print("| " + " | ".join(cols) + " |")
    print("|" + "|".join("---" for _ in cols) + "|")
    for r in rows:
        print("| " + " | ".join(str(r[c]) for c in cols) + " |")
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
