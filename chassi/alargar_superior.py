#!/usr/bin/env python3
"""Alarga o chassi superior para a largura do Esboço30 e alinha os furos dos postes ao Esboço30.

Entrada : Esboço30.dxf (referência, chassi de baixo) e chassi_superior.dxf
Saída   : chassi_superior_alargado.dxf no quadro de coordenadas do Esboço30 (cantos do contorno coincidem)
          + chassi/compare_after.png (sobreposição para conferir)

Regras (todas medidas nos dois DXF, nada chutado):
1. Largura: o contorno passa de 94,01 para 99,01 mm (a mesma do Esboço30). A altura (156,02) já é igual.
2. Como alargar: tudo que está à direita do centro do contorno anda +delta em x (entalhes e furos da direita mantêm
   a distância até a borda direita); o que está à esquerda fica. O rasgo central 25 x 12 mantém o tamanho e anda delta/2
   para continuar no meio.
3. Alinhar: o conjunto é transladado para o contorno coincidir com o do Esboço30 e os 8 furos de poste (3,21 mm) são
   levados ao centro exato dos 8 furos de poste do Esboço30 (mesmo padrão 38 / 60,74 / 38 mm, só estava deslocado).
Uso: .scratch/venv/bin/python chassi/alargar_superior.py   (precisa de ezdxf, shapely, matplotlib)
"""
from __future__ import annotations

from pathlib import Path

import ezdxf
import matplotlib
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

DOWNLOADS = Path.home() / "Downloads"
REF = DOWNLOADS / "Esboço30.dxf"
SRC = DOWNLOADS / "chassi_superior.dxf"
OUT = DOWNLOADS / "chassi_superior_alargado.dxf"
OUT_WINGS = DOWNLOADS / "chassi_superior_asas.dxf"
WING = 40.0  # asas de cada lado (mm): largura 99 -> 179 mm; as rodas externas ja chegam a |x| = 91 do centro, entao a largura total do robo nao muda
HERE = Path(__file__).resolve().parent

Pt = tuple[float, float]
Poly = list[Pt]


def load(path: Path) -> list[Poly]:
    doc = ezdxf.readfile(str(path))
    return [[(p[0], p[1]) for p in e.get_points()] for e in doc.modelspace()]


def bounds(pl: Poly) -> tuple[float, float, float, float]:
    xs, ys = [p[0] for p in pl], [p[1] for p in pl]
    return min(xs), min(ys), max(xs), max(ys)


def center(pl: Poly) -> Pt:
    x0, y0, x1, y1 = bounds(pl)
    return (x0 + x1) / 2, (y0 + y1) / 2


def move(pl: Poly, dx: float, dy: float) -> Poly:
    return [(x + dx, y + dy) for x, y in pl]


def is_post_hole(pl: Poly) -> bool:
    x0, y0, x1, y1 = bounds(pl)
    return 3.1 < (y1 - y0) < 3.3 and (x1 - x0) < 3.6  # furos de 3,21 mm (os de 3,01 mm sao de componentes)


def main() -> None:
    ref, src = load(REF), load(SRC)
    ref_outline = max(ref, key=lambda p: Polygon(p).area)
    src_outline = max(src, key=lambda p: Polygon(p).area)
    rx0, ry0, rx1, ry1 = bounds(ref_outline)
    sx0, sy0, sx1, sy1 = bounds(src_outline)
    delta = (rx1 - rx0) - (sx1 - sx0)
    assert abs((ry1 - ry0) - (sy1 - sy0)) < 0.01, "alturas diferentes: o script so alarga"
    x_mid = (sx0 + sx1) / 2
    print(f"largura {sx1 - sx0:.3f} -> {rx1 - rx0:.3f} mm (delta {delta:.3f}); altura {sy1 - sy0:.3f} igual")

    # --- 1/2: alargar (direita anda +delta; rasgo central anda delta/2)
    stretched: list[Poly] = []
    for pl in src:
        cx, _ = center(pl)
        x0, _, x1, _ = bounds(pl)
        if pl is src_outline:
            stretched.append([(x + delta if x > x_mid else x, y) for x, y in pl])
        elif x0 < x_mid < x1:  # cruza o centro: rigido, centralizado
            stretched.append(move(pl, delta / 2, 0.0))
        else:
            stretched.append(move(pl, delta if cx > x_mid else 0.0, 0.0))

    # --- 3: alinhar ao quadro do Esboço30 pelos cantos do contorno
    nx0, ny0, _, _ = bounds(stretched[src.index(src_outline)])
    dx, dy = rx0 - nx0, ry0 - ny0
    placed = [move(pl, dx, dy) for pl in stretched]

    ref_posts = sorted((center(p) for p in ref if is_post_hole(p)), key=lambda c: (c[0] > (rx0 + rx1) / 2, -c[1]))
    idx_posts = [i for i, p in enumerate(placed) if is_post_hole(p)]
    sup_posts = sorted(idx_posts, key=lambda i: (center(placed[i])[0] > (rx0 + rx1) / 2, -center(placed[i])[1]))
    assert len(ref_posts) == len(sup_posts) == 8, (len(ref_posts), len(sup_posts))
    for i, (tx, ty) in zip(sup_posts, ref_posts):
        cx, cy = center(placed[i])
        print(f"  furo de poste ({cx:9.3f},{cy:8.3f}) -> ({tx:9.3f},{ty:8.3f})  mover ({tx - cx:+.3f}, {ty - cy:+.3f})")
        placed[i] = move(placed[i], tx - cx, ty - cy)

    # --- conferencia geometrica
    outline = Polygon(placed[src.index(src_outline)])
    assert outline.is_valid, "contorno invalido"
    for i, pl in enumerate(placed):
        if pl is placed[src.index(src_outline)]:
            continue
        hole = Polygon(pl)
        assert outline.contains(hole), f"entidade {i} saiu do contorno"
        gap = outline.exterior.distance(hole)
        assert gap > 1.5, f"entidade {i} a {gap:.2f} mm da borda"
    ox0, oy0, ox1, oy1 = outline.bounds
    assert max(abs(ox0 - rx0), abs(oy0 - ry0), abs(ox1 - rx1), abs(oy1 - ry1)) < 0.01, "contorno nao coincide com o Esboço30"
    print("conferido: contorno coincide com o Esboço30, nenhum furo/rasgo toca a borda (>1,5 mm)")

    doc = ezdxf.new("R2010", units=4)
    for pl in placed:
        doc.modelspace().add_lwpolyline(pl, close=True)
    doc.saveas(str(OUT))
    (HERE / OUT.name).write_bytes(OUT.read_bytes())
    print(f"gravado {OUT} (copia em {HERE / OUT.name})")

    # --- 4: asas: onde o contorno chega a borda (faixas sem entalhe) cresce WING mm para fora; os entalhes dos motores
    # viram JANELAS FECHADAS: a asa cobre a faixa entre a borda antiga e a nova e o entalhe fica como furo interno
    oi = src.index(src_outline)
    poly = Polygon(placed[oi])
    ys = [oy0 + k * 0.1 for k in range(int((oy1 - oy0) / 0.1) + 1)]
    wings, fills = [], []
    for edge, sign in ((ox1, 1), (ox0, -1)):
        for full_run, target in ((True, wings), (False, fills)):
            run: list[float] = []
            for y in ys + [oy1 + 1]:
                inside = y <= oy1 and poly.covers(Point(edge - sign * 0.05, y)) == full_run  # sentinela y > oy1 nunca conta
                if inside:
                    run.append(y)
                elif run:
                    if run[-1] - run[0] > 1.0:  # ignora amostra solta de 0,1 mm nos cantos (erro de ponto flutuante)
                        target.append(box(min(edge, edge + sign * WING), run[0] - 0.05, max(edge, edge + sign * WING), run[-1] + 0.05))  # +-0,05: sem fresta entre faixas
                    run = []
    wide = unary_union([poly, *wings, *fills]).simplify(0.001)
    assert wide.geom_type == "Polygon" and wide.is_valid
    placed[oi] = list(wide.exterior.coords)[:-1]
    windows = [list(ring.coords)[:-1] for ring in wide.interiors]
    assert len(windows) == len(fills) == 4, (len(windows), len(fills))
    placed.extend(windows)
    wx0, _, wx1, _ = wide.bounds
    print(f"asas de {WING:.0f} mm: largura {ox1 - ox0:.2f} -> {wx1 - wx0:.2f} mm; janelas fechadas dos motores: {len(windows)}")
    doc = ezdxf.new("R2010", units=4)
    for pl in placed:
        doc.modelspace().add_lwpolyline(pl, close=True)
    doc.saveas(str(OUT_WINGS))
    (HERE / OUT_WINGS.name).write_bytes(OUT_WINGS.read_bytes())
    print(f"gravado {OUT_WINGS} (copia em {HERE / OUT_WINGS.name})")

    fig, ax = plt.subplots(figsize=(7, 11))
    for pl in ref:
        ax.plot([p[0] for p in pl] + [pl[0][0]], [p[1] for p in pl] + [pl[0][1]], "b", lw=0.8)
    for pl in placed:
        ax.plot([p[0] for p in pl] + [pl[0][0]], [p[1] for p in pl] + [pl[0][1]], "r", lw=0.8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title("azul = Esboço30, vermelho = superior com asas")
    fig.tight_layout()
    fig.savefig(HERE / "compare_after.png", dpi=80)


if __name__ == "__main__":
    main()
