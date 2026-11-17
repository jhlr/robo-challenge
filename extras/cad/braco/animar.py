#!/usr/bin/env python3
"""Renderiza a sequencia pegar -> levantar -> soltar -> voltar do braco (braco.scad, scene="anim") e monta MP4/GIF.
Uso: python3 cad/braco/animar.py [n_frames]   -> docs/2026-10-21_braco-movimento.mp4 e .gif (frames em docs/anim_frames/)"""
import subprocess, sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]   # raiz do repo (docs/ fica la)
SCAD = ROOT / "cad" / "braco" / "braco_mdf.scad"      # versao em MDF (a que esta sendo construida)
FRAMES = ROOT / "docs" / "anim_frames"
A_PICK, A_PLACE, PSI_OPEN = -9.788, 25.468, 20.0     # de params_gen.scad / params.json

def ease(x): x = min(max(x, 0.0), 1.0); return x * x * (3 - 2 * x)
def seg(t, a, b): return ease((t - a) / (b - a))

def state(t):
    """(alpha, psi, cube_mode, show_platform) em t em [0, 1)."""
    if t < 0.12: return A_PICK, PSI_OPEN, "floor", False
    if t < 0.24: return A_PICK, PSI_OPEN * (1 - seg(t, 0.12, 0.24)), "floor", False
    if t < 0.56: return A_PICK + (A_PLACE - A_PICK) * seg(t, 0.24, 0.56), 0.0, "carry", True
    if t < 0.68: return A_PLACE, PSI_OPEN * seg(t, 0.56, 0.68), "carry", True
    if t < 0.80: return A_PLACE, PSI_OPEN, "placed", True
    return A_PLACE + (A_PICK - A_PLACE) * seg(t, 0.80, 1.0), PSI_OPEN, "placed", True

def main(n):
    FRAMES.mkdir(parents=True, exist_ok=True)
    for i in range(n):
        alpha, psi, mode, plat = state(i / n)
        out = FRAMES / f"f{i:03d}.png"
        subprocess.run(["openscad", "-o", str(out), "--camera=-560,-200,360,0,90,70",
                        "--imgsize=1000,760", "-D", 'scene="anim"', "-D", f"alpha={alpha:.3f}", "-D", f"claw_psi={psi:.3f}",
                        "-D", f'cube_mode="{mode}"', "-D", f"show_platform={'true' if plat else 'false'}", str(SCAD)],
                       check=True, capture_output=True)
    mp4 = ROOT / "docs" / "2026-10-21_braco-movimento.mp4"
    gif = mp4.with_suffix(".gif")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "24", "-i", str(FRAMES / "f%03d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", str(mp4)], check=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "24", "-i", str(FRAMES / "f%03d.png"),
                    "-vf", "scale=700:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse", "-loop", "0", str(gif)], check=True)
    print(mp4, gif)

if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 96)
