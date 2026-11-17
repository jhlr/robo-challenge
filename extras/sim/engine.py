"""Motor de simulacao: executa passos do robo na arena, aplica oponente, ruido e pontuacao."""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Protocol

from arena import (CELL_MM, GATE_STUB_MM, GREEN_TO_PLATFORM_MM, HALF_MM, ROW_Y_MM, SLOT_Y_MM, WHITE, Arena, Cube, Loc,
                   Node, adjacent_locs, edge_loc, node_loc)


@dataclass(frozen=True)
class Params:
    v_mm_s: float = 180.0          # STS3215 + roda ~65 mm, ~52 rpm
    t_turn90: float = 1.0
    t_turn180: float = 1.6
    t_scan: float = 0.8            # parar diante do cubo e ler a cor
    t_peek: float = 0.5            # ToF olha a rua antes de entrar (so presenca, sem cor)
    t_pick: float = 2.5
    t_place: float = 2.5
    t_signal: float = 1.5          # sinal sonoro/luminoso do cubo branco
    p_pick_fail: float = 0.10
    match_s: float = 720.0
    cap: int = 2
    opp_event_rate: float = 1 / 25   # empurroes/retiradas de cubo pelo oponente por segundo
    p_opp_take: float = 0.35         # fracao dos eventos em que o oponente leva o cubo (resto: empurra)
    p_opp_block: float = 0.05        # chance por rua de encontrar o oponente de frente
    t_opp_block: float = 6.0
    p_miss: float = 0.02             # cruzamento nao detectado por intersecao
    gated_factor: float = 0.1        # reducao do erro com odometria + IMU conferindo os cruzamentos
    t_lost: float = 20.0             # tempo medio para se relocalizar em uma linha de referencia


Step = Node


class Deliver:
    pass


Action = Step | Deliver


class Policy(Protocol):
    name: str
    uses_map: bool
    gated: bool

    def reset(self, arena: Arena, params: Params, rng: random.Random) -> None: ...
    def observe(self, loc: Loc, cube: Cube | None, t: float) -> None: ...
    def observe_presence(self, loc: Loc, present: bool, t: float) -> None: ...
    def verify(self, loc: Loc, t: float) -> bool: ...
    def blocked(self, loc: Loc, carrying: list[Cube], t: float) -> bool: ...
    def wants(self, color: str, carrying: list[Cube], t: float) -> bool: ...
    def act(self, node: Node, carrying: list[Cube], t: float) -> Action: ...


@dataclass
class Stats:
    delivered: int = 0
    identified: int = 0
    lost: int = 0
    wasted: int = 0                # aproximacoes que terminaram sem pegar o cubo
    miss_events: int = 0
    opp_blocks: int = 0
    opp_removed: int = 0
    opp_pushed: int = 0


@dataclass
class Result:
    score: float
    stats: Stats


@dataclass
class Engine:
    arena: Arena
    policy: Policy
    params: Params
    rng: random.Random
    opp_rng: random.Random
    t: float = 8.0
    score: float = 0.0
    node: Node = (0, 0)
    heading: Node = (1, 0)
    carrying: list[Cube] = field(default_factory=list)
    stats: Stats = field(default_factory=Stats)

    # ---- tempo e oponente ----
    def advance(self, seconds: float) -> None:
        end = self.t + seconds
        events = _poisson(self.opp_rng, self.params.opp_event_rate * seconds)
        self.t = end
        for _ in range(events):
            self._opponent_event()

    def _opponent_event(self) -> None:
        if not self.arena.cubes:
            return
        loc = self.opp_rng.choice(sorted(self.arena.cubes))
        if self.opp_rng.random() < self.params.p_opp_take:
            del self.arena.cubes[loc]
            self.stats.opp_removed += 1
            return
        free = [a for a in adjacent_locs(loc) if a not in self.arena.cubes]
        if free:
            self.arena.cubes[self.opp_rng.choice(sorted(free))] = self.arena.cubes.pop(loc)
            self.stats.opp_pushed += 1

    # ---- movimento ----
    def _turn_to(self, direction: Node) -> None:
        if direction == self.heading:
            return
        opposite = (-self.heading[0], -self.heading[1])
        self.advance(self.params.t_turn180 if direction == opposite else self.params.t_turn90)
        self.heading = direction

    def _drive(self, mm: float) -> None:
        self.advance(mm / self.params.v_mm_s)

    def _encounter(self, loc: Loc) -> str:
        """Retorna 'picked', 'blocked' ou 'none'."""
        cube = self.arena.cubes.get(loc)
        self.policy.observe(loc, cube, self.t)
        if cube is None:
            return "none"
        self.advance(self.params.t_scan)
        if self.arena.cubes.get(loc) is not cube:      # o oponente levou ou empurrou o cubo enquanto lia
            return "none"
        self.policy.observe(loc, cube, self.t)
        if len(self.carrying) < self.params.cap and self.policy.wants(cube.color, self.carrying, self.t):
            self.advance(self.params.t_pick)
            if self.arena.cubes.get(loc) is not cube:
                return "none"
            if self.rng.random() < self.params.p_pick_fail:
                self.score -= 10
                self.stats.lost += 1
                self.stats.wasted += 1
                return "blocked"
            del self.arena.cubes[loc]
            self.carrying.append(cube)
            return "picked"
        if (cube.color == WHITE or not self.arena.deliverable(cube.color)) and cube.cid not in self.arena.identified:
            self.arena.identified.add(cube.cid)
            self.score += 50
            self.stats.identified += 1
            self.advance(self.params.t_signal)
        self.stats.wasted += 1
        return "blocked"

    def _back_off(self, mm: float) -> None:
        self._drive(mm)
        self.heading = (-self.heading[0], -self.heading[1])
        self.advance(self.params.t_turn180)

    def step(self, nxt: Node) -> None:
        a = self.node
        direction = (nxt[0] - a[0], nxt[1] - a[1])
        self._turn_to(direction)
        ahead = (edge_loc(a, nxt), node_loc(nxt))
        if any(self.policy.verify(loc, self.t) for loc in ahead):
            self.advance(self.params.t_peek)
            for loc in ahead:
                self.policy.observe_presence(loc, loc in self.arena.cubes, self.t)
            if any(loc in self.arena.cubes and self.policy.blocked(loc, self.carrying, self.t) for loc in ahead):
                return                  # o sensor confirmou o bloqueio: nao entra
        if self.rng.random() < self.params.p_opp_block:
            self.stats.opp_blocks += 1
            self.advance(self.params.t_opp_block)
        self._drive(HALF_MM)
        mid = self._encounter(edge_loc(a, nxt))
        if mid == "blocked":
            self._back_off(HALF_MM)
            return
        self._drive(HALF_MM)
        if self._encounter(node_loc(nxt)) == "blocked":
            self._back_off(CELL_MM)
            return
        self.node = nxt
        p = self.params.p_miss * (self.params.gated_factor if self.policy.gated else 1.0)
        if self.policy.uses_map and self.rng.random() < p:
            self.stats.miss_events += 1
            self.advance(self.params.t_lost)

    def deliver(self) -> None:
        r = self.node[1]
        slots = sorted(self.arena.platforms[c.color] for c in self.carrying if self.arena.deliverable(c.color))
        if not slots:
            return
        y_gate = ROW_Y_MM[r]
        legs = abs(SLOT_Y_MM[slots[0]] - y_gate) + (SLOT_Y_MM[slots[-1]] - SLOT_Y_MM[slots[0]]) + abs(SLOT_Y_MM[slots[-1]] - y_gate)
        self._turn_to((-1, 0))
        self._drive(GATE_STUB_MM + 2 * GREEN_TO_PLATFORM_MM + legs)
        self.advance(self.params.t_place * len(slots) + 2 * self.params.t_turn90)
        if self.t > self.params.match_s:
            return
        for cube in self.carrying:
            if self.arena.has_room(cube.color):
                self.arena.delivered[cube.color] = self.arena.delivered.get(cube.color, 0) + 1
                self.score += 100
                self.stats.delivered += 1
        self.carrying = []
        self._back_off(0.0)

    def run(self) -> Result:
        self.policy.reset(self.arena, self.params, self.rng)
        self.node = (0, self.rng.randrange(len(ROW_Y_MM)))
        guard = 0
        while self.t < self.params.match_s and guard < 20000:
            guard += 1
            action = self.policy.act(self.node, self.carrying, self.t)
            if isinstance(action, Deliver):
                if self.node[0] != 0:
                    raise RuntimeError("Deliver fora da coluna 0")
                self.deliver()
            else:
                self.step(action)
        return Result(max(self.score, 0.0), self.stats)


def _poisson(rng: random.Random, lam: float) -> int:
    limit, k, p = math.exp(-lam), 0, 1.0
    while True:
        p *= rng.random()
        if p <= limit:
            return k
        k += 1
