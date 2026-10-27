"""Invariantes basicas do simulador. Uso: python3 -m unittest sim/test_sim.py (a partir de Arduino/)."""
from __future__ import annotations

import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from arena import CUBES_PER_COLOR, N_WHITE, PLATFORM_COLORS, Arena, all_edge_locs  # noqa: E402
from engine import Engine, Params  # noqa: E402
from policies import INF, NearestFirst, Planner, RandomWalk  # noqa: E402


def episode(policy, seed: int):  # type: ignore[no-untyped-def]
    arena = Arena.random(random.Random(seed))
    res = Engine(arena, policy, Params(), rng=random.Random(seed + 1), opp_rng=random.Random(seed + 2)).run()
    return arena, res


class SimTests(unittest.TestCase):
    def test_arena_has_36_cubes_on_distinct_streets(self) -> None:
        a = Arena.random(random.Random(0))
        self.assertEqual(len(a.cubes), len(PLATFORM_COLORS) * CUBES_PER_COLOR + N_WHITE)
        self.assertEqual(len(all_edge_locs()), 49)
        self.assertTrue(set(a.cubes) <= set(all_edge_locs()))

    def test_deterministic_per_seed(self) -> None:
        for make in (lambda: RandomWalk(), lambda: NearestFirst("n", INF, True), lambda: Planner("p", 60.0, True)):
            self.assertEqual(episode(make(), 5)[1].score, episode(make(), 5)[1].score)

    def test_score_and_deliveries_are_consistent(self) -> None:
        for seed in range(5):
            arena, res = episode(Planner("p", 60.0, True), seed)
            self.assertGreaterEqual(res.score, 0.0)
            self.assertEqual(sum(arena.delivered.values()), res.stats.delivered)
            self.assertTrue(all(n <= 3 for n in arena.delivered.values()))
            self.assertTrue(all(c in arena.platforms for c in arena.delivered))


if __name__ == "__main__":
    unittest.main()
