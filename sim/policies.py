"""Politicas: passeio aleatorio (reativo), vizinho mais proximo e planejador, com mapa de crencas que decai."""
from __future__ import annotations

import heapq
import math
import random

from arena import CELL_MM, COLS, ROWS, WHITE, Arena, Cube, Loc, Node, all_edge_locs, edge_loc, neighbors, node_loc
from engine import Action, Deliver, Params

INF = float("inf")
UNKNOWN_COLOR = "?"
ALL_NODES: list[Node] = [(c, r) for c in range(COLS) for r in range(ROWS)]
ALL_LOCS: list[Loc] = all_edge_locs() + [node_loc(n) for n in ALL_NODES]


class RandomWalk:
    """Reativo, sem memoria: nao volta pela rua de onde veio, entrega ao carregar o limite ou apos 60 s."""

    uses_map = False
    gated = False

    def __init__(self, name: str = "RandomWalk") -> None:
        self.name = name

    def reset(self, arena: Arena, params: Params, rng: random.Random) -> None:
        self.arena, self.params, self.rng = arena, params, rng
        self.prev: Node | None = None
        self.first_pick: float | None = None

    def observe(self, loc: Loc, cube: Cube | None, t: float) -> None:
        pass

    def observe_presence(self, loc: Loc, present: bool, t: float) -> None:
        pass

    def verify(self, loc: Loc, t: float) -> bool:
        return False

    def blocked(self, loc: Loc, carrying: list[Cube], t: float) -> bool:
        return False

    def wants(self, color: str, carrying: list[Cube], t: float) -> bool:
        return self.arena.has_room(color, sum(1 for c in carrying if c.color == color))

    def act(self, node: Node, carrying: list[Cube], t: float) -> Action:
        if not carrying:
            self.first_pick = None
        elif self.first_pick is None:
            self.first_pick = t
        if node[0] == 0 and carrying and (len(carrying) >= self.params.cap or t - (self.first_pick or t) > 60):
            self.prev = None
            return Deliver()
        options = [n for n in neighbors(node) if n != self.prev] or neighbors(node)
        nxt = self.rng.choice(options)
        self.prev = node
        return nxt


class MemoryPolicy:
    """Base com mapa de crencas: o que foi visto em cada ponto e quando. `tau` e a meia-vida da confianca."""

    uses_map = True

    BLOCK_PEN_MM = 40 * 180.0     # com sensor_first, rua que a memoria diz bloqueada custa +40 s, nunca infinito

    def __init__(self, name: str, tau: float, gated: bool, sensor_first: bool = True, corrupt: float = 0.0) -> None:
        self.name, self.tau, self.gated = name, tau, gated
        self.sensor_first = sensor_first      # True: a memoria so guia e o ToF confere antes de entrar
        self.corrupt = corrupt                # chance de uma crenca ser gravada errada (teste de robustez)

    def reset(self, arena: Arena, params: Params, rng: random.Random) -> None:
        self.arena, self.params, self.rng = arena, params, rng
        self.belief: dict[Loc, tuple[str | None, float]] = {}
        self.done: set[Loc] = set()      # pontos onde um cubo sem destino ja foi identificado (ponto so uma vez)
        self.target: Loc | None = None
        self.dist: dict[Node, float] = {}
        self.prev: dict[Node, Node] = {}
        self.ctx: tuple[list[Cube], float] = ([], 0.0)

    # ---- crencas ----
    def observe(self, loc: Loc, cube: Cube | None, t: float) -> None:
        color = cube.color if cube else None
        if self.corrupt and self.rng.random() < self.corrupt:
            color = None if color is not None else self.rng.choice(["blue", "red", "green", "yellow", "black", "brown", WHITE])
        self.belief[loc] = (color, t)
        if cube is not None and (cube.color == WHITE or not self.arena.deliverable(cube.color)):
            self.done.add(loc)

    def observe_presence(self, loc: Loc, present: bool, t: float) -> None:
        """ToF viu (ou nao viu) algo no ponto: confirma ou apaga a crenca, sem ler a cor."""
        b = self.belief.get(loc)
        if not present:
            self.belief[loc] = (None, t)
        elif b is not None and b[0] is not None:
            self.belief[loc] = (b[0], t)
        else:
            self.belief[loc] = (UNKNOWN_COLOR, t)   # algo na rua, cor desconhecida: vale ir ler de perto

    def verify(self, loc: Loc, t: float) -> bool:
        b = self.belief.get(loc)
        return self.sensor_first and b is not None and b[0] is not None

    def conf(self, loc: Loc, t: float) -> float:
        if loc not in self.belief:
            return 0.0
        return 1.0 if self.tau == INF else math.exp(-(t - self.belief[loc][1]) * math.log(2) / self.tau)

    def wants(self, color: str, carrying: list[Cube], t: float) -> bool:
        return self.arena.has_room(color, sum(1 for c in carrying if c.color == color))

    def wanted_known(self, loc: Loc, carrying: list[Cube], t: float) -> bool:
        b = self.belief.get(loc)
        if b is None or b[0] is None or self.conf(loc, t) <= 0.2:
            return False
        return b[0] == UNKNOWN_COLOR or self.wants(b[0], carrying, t)

    def blocked(self, loc: Loc, carrying: list[Cube], t: float) -> bool:
        b = self.belief.get(loc)
        if b is None or b[0] is None or b[0] == UNKNOWN_COLOR:
            return False
        return self.conf(loc, t) > 0.3 and not self.wants(b[0], carrying, t)

    # ---- roteamento (uma busca por decisao) ----
    def plan_from(self, src: Node, carrying: list[Cube], t: float) -> None:
        self.ctx = (carrying, t)
        self.dist, self.prev = {src: 0.0}, {}
        heap = [(0.0, src)]
        while heap:
            d, n = heapq.heappop(heap)
            if d > self.dist[n]:
                continue
            for m in neighbors(n):
                w = self.edge_w(n, m)
                if w is None:
                    continue
                if d + w < self.dist.get(m, INF):
                    self.dist[m], self.prev[m] = d + w, n
                    heapq.heappush(heap, (d + w, m))

    def edge_w(self, a: Node, b: Node) -> float | None:
        """Custo (mm) de ir de a para b. Memory-decisive: bloqueio e proibicao. Sensor-first: so penaliza."""
        carrying, t = self.ctx
        w = CELL_MM
        for loc in (edge_loc(a, b), node_loc(b)):
            if self.blocked(loc, carrying, t):
                if not self.sensor_first:
                    return None
                w += self.BLOCK_PEN_MM * self.conf(loc, t)
        return w

    def path_to(self, dst: Node) -> list[Node]:
        path = [dst]
        while path[-1] in self.prev:
            path.append(self.prev[path[-1]])
        return path[::-1]

    def approach(self, loc: Loc) -> tuple[list[Node], float] | None:
        """Caminho ate o ponto: chega a uma ponta da rua e cruza; para intersecao, entra nela."""
        a, b = loc
        cands: list[tuple[Node, Node]] = [(a, b), (b, a)] if a != b else [(m, a) for m in neighbors(a)]
        best: tuple[Node, Node] | None = None
        best_d = INF
        for start, end in cands:
            last = CELL_MM if a != b else self.edge_w(start, end)
            if last is None or start not in self.dist:
                continue
            d = self.dist[start] + last
            if d < best_d:
                best, best_d = (start, end), d
        if best is None:
            return None
        return self.path_to(best[0]) + [best[1]], best_d / self.params.v_mm_s

    def gate_path(self) -> list[Node] | None:
        best = min(((self.dist[(0, r)], r) for r in range(ROWS) if (0, r) in self.dist), default=None)
        return None if best is None else self.path_to((0, best[1]))

    def deliver_time(self) -> float:
        p = self.gate_path()
        return INF if p is None else self.dist[p[-1]] / self.params.v_mm_s + 20.0

    def go_deliver(self, node: Node) -> Action:
        if node[0] == 0:
            return Deliver()
        p = self.gate_path()
        return p[1] if p and len(p) > 1 else self.rng.choice(neighbors(node))

    def patrol_target(self, t: float) -> Loc | None:
        """Sem alvo de valor: reconfere o ponto mais antigo (cubos sao empurrados pelo oponente)."""
        best, best_score = None, -1.0
        for loc in ALL_LOCS:
            res = self.approach(loc)
            if res is None or res[1] <= 0 or self.blocked(loc, *self.ctx):
                continue
            age = min(t - self.belief[loc][1], 300.0) if loc in self.belief else 300.0
            score = age / (res[1] + 5.0)
            if score > best_score:
                best, best_score = loc, score
        return best

    # ---- decisao ----
    def pick_target(self, node: Node, carrying: list[Cube], t: float) -> Loc | None:
        raise NotImplementedError

    def should_deliver(self, node: Node, carrying: list[Cube], t: float) -> bool:
        raise NotImplementedError

    def act(self, node: Node, carrying: list[Cube], t: float) -> Action:
        self.plan_from(node, carrying, t)
        if carrying and self.should_deliver(node, carrying, t):
            return self.go_deliver(node)
        tgt = self.target
        if tgt is not None:
            b = self.belief.get(tgt)
            if b is not None and self.conf(tgt, t) > 0.9 and (b[0] is None or self.blocked(tgt, carrying, t)):
                self.target = tgt = None
        if tgt is None:
            self.target = tgt = self.pick_target(node, carrying, t) or self.patrol_target(t)
        if tgt is None:
            return self.go_deliver(node) if carrying else self.rng.choice(neighbors(node))
        res = self.approach(tgt)
        if res is None or len(res[0]) < 2:
            self.target = None
            return self.rng.choice(neighbors(node))
        nxt = res[0][1]
        if edge_loc(node, nxt) == tgt or node_loc(nxt) == tgt:
            self.target = None            # o encontro deste passo resolve o alvo
        return nxt


class NearestFirst(MemoryPolicy):
    """Memoria sem decaimento: vai ao cubo desejado conhecido mais proximo e entrega apos cada cubo."""

    def should_deliver(self, node: Node, carrying: list[Cube], t: float) -> bool:
        return True

    def pick_target(self, node: Node, carrying: list[Cube], t: float) -> Loc | None:
        best, best_cost = None, INF
        stale, stale_cost = None, INF
        for loc in ALL_LOCS:
            res = self.approach(loc)
            if res is None or res[1] <= 0:
                continue
            if self.wanted_known(loc, carrying, t):
                if res[1] < best_cost:
                    best, best_cost = loc, res[1]
            elif self.conf(loc, t) < 0.3 and res[1] < stale_cost:
                stale, stale_cost = loc, res[1]
        return best or stale


class Planner(MemoryPolicy):
    """Memoria com decaimento, ate `cap` cubos por viagem, alvos por valor esperado / tempo."""

    P_PRIOR = 0.5                # densidade esperada de cubos em um ponto nunca visto

    def should_deliver(self, node: Node, carrying: list[Cube], t: float) -> bool:
        if len(carrying) >= self.params.cap:
            return True
        if self.params.match_s - t < self.deliver_time() + 40:
            return True
        tgt = self.pick_target(node, carrying, t)
        res = None if tgt is None else self.approach(tgt)
        return res is None or res[1] > 25.0

    def value(self, loc: Loc, carrying: list[Cube], t: float) -> float:
        b = self.belief.get(loc)
        c = self.conf(loc, t)
        n_room = sum(1 for col in self.arena.platforms if self.arena.has_room(col))
        p_wanted = min(1.0, n_room * 5 / 36)
        if b is not None and b[0] == UNKNOWN_COLOR:
            return self.conf(loc, t) * (p_wanted * 100.0 + (1 - p_wanted) * 35.0)
        if b is not None and b[0] is not None:
            color = b[0]
            if self.wants(color, carrying, t):
                return 100.0 * c
            return 50.0 * c if ((color == WHITE or not self.arena.deliverable(color)) and loc not in self.done) else 0.0
        p_cube = (1.0 - c) * self.P_PRIOR
        return p_cube * (p_wanted * 100.0 + (1 - p_wanted) * 25.0)

    def pick_target(self, node: Node, carrying: list[Cube], t: float) -> Loc | None:
        best, best_ratio = None, 0.0
        for loc in ALL_LOCS:
            res = self.approach(loc)
            if res is None or res[1] <= 0:
                continue
            ratio = self.value(loc, carrying, t) / (res[1] + self.params.t_scan + self.params.t_pick)
            if ratio > best_ratio:
                best, best_ratio = loc, ratio
        return best
