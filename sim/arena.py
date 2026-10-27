"""Arena do RCB Challenge 2026 como grafo topologico (v0).

Geometria estimada da Figura 1 das regras (nao esta no texto): celulas de ~300 mm, 6x5 intersecoes,
rua de 20 mm, cubos de 50 mm. Cubos ocupam pontos medios das 49 ruas internas ou intersecoes.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field

Node = tuple[int, int]            # (coluna 0..5, linha 0..4)
Loc = tuple[Node, Node]           # ponto medio de uma rua (a, b ordenados) ou intersecao (n, n)

COLS, ROWS = 6, 5
CELL_MM = 300.0
HALF_MM = CELL_MM / 2
GATE_STUB_MM = 150.0              # da primeira coluna ate a linha azul
GREEN_TO_PLATFORM_MM = 350.0      # da linha azul ate a plataforma
SLOT_Y_MM = tuple(146.0 + 292.0 * k for k in range(5))     # 5 retangulos brancos no Ecoponto
ROW_Y_MM = tuple(150.0 + CELL_MM * r for r in range(ROWS))

PLATFORM_COLORS = ("blue", "red", "green", "yellow", "black", "brown")
WHITE = "white"
CUBES_PER_COLOR = 5
N_WHITE = 6
PLATFORM_CAP = 3                  # sem empilhar; 5 se empilhar


def edge_loc(a: Node, b: Node) -> Loc:
    return (a, b) if a <= b else (b, a)


def node_loc(n: Node) -> Loc:
    return (n, n)


def neighbors(n: Node) -> list[Node]:
    c, r = n
    out = []
    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        if 0 <= c + dc < COLS and 0 <= r + dr < ROWS:
            out.append((c + dc, r + dr))
    return out


def all_edge_locs() -> list[Loc]:
    return sorted({edge_loc(n, m) for c in range(COLS) for r in range(ROWS) for n in [(c, r)] for m in neighbors(n)})


def adjacent_locs(loc: Loc) -> list[Loc]:
    """Onde um cubo pode parar se for empurrado a partir de `loc`."""
    a, b = loc
    if a == b:
        return [edge_loc(a, m) for m in neighbors(a)]
    return [node_loc(a), node_loc(b)]


@dataclass
class Cube:
    cid: int
    color: str


@dataclass
class Arena:
    cubes: dict[Loc, Cube]
    platforms: dict[str, int]                                  # cor -> indice do slot no Ecoponto
    delivered: dict[str, int] = field(default_factory=dict)
    identified: set[int] = field(default_factory=set)

    @staticmethod
    def random(rng: random.Random) -> "Arena":
        colors = [c for c in PLATFORM_COLORS for _ in range(CUBES_PER_COLOR)] + [WHITE] * N_WHITE
        rng.shuffle(colors)
        locs = rng.sample(all_edge_locs(), len(colors))
        cubes = {loc: Cube(i, col) for i, (loc, col) in enumerate(zip(locs, colors))}
        k = rng.randint(1, 5)
        platforms = dict(zip(rng.sample(PLATFORM_COLORS, k), rng.sample(range(5), k)))
        return Arena(cubes, platforms)

    def has_room(self, color: str, carrying: int = 0) -> bool:
        return color in self.platforms and self.delivered.get(color, 0) + carrying < PLATFORM_CAP

    def deliverable(self, color: str) -> bool:
        return color in self.platforms
