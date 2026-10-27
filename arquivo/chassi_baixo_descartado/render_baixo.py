"""Vista de cima do convés baixo com as rodas por baixo (confere o vão da ponte). Uso: .scratch/venv/bin/python cad/chassi/render_baixo.py"""
from pathlib import Path

import ezdxf
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from shapely.geometry import Polygon  # noqa: E402

HERE = Path(__file__).resolve().parent
P = [[(p[0], p[1]) for p in e.get_points()] for e in ezdxf.readfile(str(HERE / "chassi_superior_baixo.dxf")).modelspace()]
out = max(P, key=lambda p: Polygon(p).area)
x0, y0, x1, y1 = Polygon(out).bounds
cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
fig, ax = plt.subplots(figsize=(10, 7.5))
ax.fill(*zip(*out), color="#d9b382", ec="k", lw=1)
for pl in P:
    if pl is not out:
        ax.fill(*zip(*pl), color="white", ec="k", lw=0.8)
for s in (1, -1):
    for wy in (45, -45):
        ax.add_patch(Rectangle((cx + (67 if s == 1 else -91), cy + wy - 33), 24, 66, color="#202020", alpha=0.45))
ax.text(cx, cy, "convés baixo\n(sem espaçador)", ha="center", va="center", fontsize=10)
ax.text(cx + 80, cy + 63, "roda (sob o convés)", fontsize=8, color="#202020")
ax.set_aspect("equal"); ax.set_title(f"largura total {x1 - x0:.0f} mm | cinza = rodas por baixo | ponte 18 mm no vão entre as rodas")
ax.set_xlabel("mm"); fig.tight_layout(); fig.savefig(HERE / "chassi_baixo_topo.png", dpi=110)
