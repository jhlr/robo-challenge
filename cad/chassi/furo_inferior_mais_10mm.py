#!/usr/bin/env python3
"""Aumenta em 10 mm o furo em L (passagem de fios) do chassi inferior (Esboço30.dxf), como riscado a lapis na foto:
o lado do corpo largo do furo (borda de x maior) avanca 10 mm, na largura inteira do corpo. O resto do arquivo fica igual.
Eixos conferidos na foto: x do DXF cresce para baixo na imagem, y cresce para a esquerda (furos de 2,51 mm a 15,65 mm um do outro batem).
Uso: .scratch/venv/bin/python cad/chassi/furo_inferior_mais_10mm.py
"""
from __future__ import annotations

from pathlib import Path

import ezdxf
from shapely.geometry import Polygon

SRC = Path.home() / "Downloads" / "Esboço30.dxf"
OUT = Path.home() / "Downloads" / "Esboço30_furo_mais_10mm.dxf"
HERE = Path(__file__).resolve().parent
GROW_MM = 10.0

doc = ezdxf.readfile(str(SRC))
msp = doc.modelspace()
ents = list(msp)
# o furo em L e a unica polilinha de 6 vertices
(hole,) = [e for e in ents if len(e.get_points()) == 6]
pts = [(p[0], p[1]) for p in hole.get_points()]
x_max = max(x for x, _ in pts)
new_pts = [(x + GROW_MM if abs(x - x_max) < 1e-6 else x, y) for x, y in pts]
before, after = Polygon(pts), Polygon(new_pts)
print(f"furo L: area {before.area:.1f} -> {after.area:.1f} mm2; x_max {x_max:.2f} -> {x_max + GROW_MM:.2f}")
assert abs(after.area - before.area - GROW_MM * (before.bounds[3] - before.bounds[1])) < 1e-6, "so o corpo largo deveria crescer"

others = [Polygon([(p[0], p[1]) for p in e.get_points()]) for e in ents if e is not hole]
outline = max(others, key=lambda p: p.area)
assert outline.contains(after) and outline.exterior.distance(after) > 20, "furo perto da borda"
nearest = min(o.distance(after) for o in others if o is not outline)
print(f"folga ate o furo/oval mais proximo: {nearest:.1f} mm (antes {min(o.distance(before) for o in others if o is not outline):.1f} mm)")
assert nearest > 3.0, "ficou colado em outro furo"

hole.set_points(new_pts, format="xy")
doc.saveas(str(OUT))
(HERE / OUT.name).write_bytes(OUT.read_bytes())
print(f"gravado {OUT}")
