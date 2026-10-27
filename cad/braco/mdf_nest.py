"""Gera as pecas 2D (DXF) do braco em MDF e as organiza em chapas de 300 x 200 mm (cortadora 20 x 30 cm).

Uso: python3 cad/braco/mdf_nest.py [grupo]    (saida em cad/braco/mdf/: parts/*.dxf, sheet_N.dxf, sheet_N.svg, plano.txt)
     grupo = "moldura" gera so a folha da moldura da pinca (sheet_moldura_N.*); sem grupo, todas as pecas. A espessura vem de mt em braco_mdf.scad.
Fonte unica: braco_mdf.scad (PARTS_2D, mt, fit). Aqui so exporta, mede e encaixa as pecas na chapa.
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "mdf"
SHEET_W, SHEET_H, MARGIN, GAP = 300.0, 200.0, 5.0, 3.0
GROUPS = {"moldura": ["cshelf", "rear", "stop", "bridge"], "ombro": ["base", "servoplate", "bar"], "torres": ["tower", "back", "side", "front"]}   # ombro: MG996R + reducao 4:1
PARTS = {"base": 1, "tower": 2, "back": 1, "servoplate": 1, "side": 2, "front": 1, "cshelf": 1, "rear": 1,
         "stop": 1, "bridge": 1, "bar": 4, "coupon": 1}      # as mandibulas sao impressas (stl/claw_jaw_*.stl)


def openscad(args: list[str]) -> None:
    subprocess.run(["caffeinate", "-i", "openscad", *args], check=True, capture_output=True, cwd=HERE)


def bbox(dxf: Path) -> tuple[float, float, float, float]:
    lines = dxf.read_text().split("\n")
    xs, ys = [], []
    for i in range(0, len(lines) - 3, 2):
        code, val = lines[i].strip(), lines[i + 1].strip()
        if code == "10" and lines[i + 2].strip() == "20":
            xs.append(float(val)); ys.append(float(lines[i + 3].strip()))
    return min(xs), min(ys), max(xs), max(ys)


def main() -> None:
    extra: list[str] = []
    group = sys.argv[1] if len(sys.argv) > 1 else None
    parts = {k: v for k, v in PARTS.items() if group is None or k in GROUPS[group]}
    tag = f"_{group}" if group else ""
    (OUT / "parts").mkdir(parents=True, exist_ok=True)
    items = []
    for name, qty in parts.items():
        f = OUT / "parts" / f"{name}.dxf"
        openscad(["-o", str(f), "-D", f'scene="2d_{name}"', *extra, "braco_mdf.scad"])
        x0, y0, x1, y1 = bbox(f)
        items += [(name, x0, y0, x1 - x0, y1 - y0)] * qty
    items.sort(key=lambda t: -t[4])                                   # altas primeiro (shelf packing)
    sheets: list[list[tuple[str, float, float, float, float]]] = [[]]
    x = y = MARGIN; row_h = 0.0
    for name, bx, by, w, h in items:
        if x + w > SHEET_W - MARGIN:
            x = MARGIN; y += row_h + GAP; row_h = 0.0
        if y + h > SHEET_H - MARGIN:
            sheets.append([]); x = y = MARGIN; row_h = 0.0
        sheets[-1].append((name, x - bx, y - by, w, h))
        x += w + GAP; row_h = max(row_h, h)
    report = []
    for n, sh in enumerate(sheets, 1):
        scad = OUT / f"sheet{tag}_{n}.scad"
        scad.write_text("\n".join(f'translate([{dx:.3f}, {dy:.3f}]) import("parts/{name}.dxf");' for name, dx, dy, w, h in sh) + "\n")
        for ext in ("dxf", "svg"):
            openscad(["-o", str(OUT / f"sheet{tag}_{n}.{ext}"), str(scad)])
        used = sum(w * h for *_, w, h in sh) / (SHEET_W * SHEET_H) * 100
        counts = {k: sum(1 for t in sh if t[0] == k) for k in dict.fromkeys(t[0] for t in sh)}
        report.append(f"chapa {n} ({SHEET_W:.0f} x {SHEET_H:.0f} mm, ~{used:.0f}% ocupada pelas caixas): {counts}")
    (OUT / f"plano{tag}.txt").write_text("\n".join(report) + "\n")
    print("\n".join(report))


if __name__ == "__main__":
    main()
