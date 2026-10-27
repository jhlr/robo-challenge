"""Renderiza robot_completo.scad em 3 vistas e compoe docs/2026-10-19_robo-completo.png com legenda.

Uso: python3 cad/modelo_robo/robo_ilustracao.py
Ilustracao conceitual: medidas do chassi novo e da eletronica sao estimadas; a garra e a do modelo real.
"""
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / "docs" / "2026-10-19_robo-completo.png"
BG = (255, 255, 229)

VIEWS = {
    "iso": ["--camera=-330,330,300,0,40,45", "--imgsize=1600,1200"],
    "top": ["--camera=0,40,700,0,40,0", "--projection=o", "--imgsize=1000,1000"],
    "side": ["--camera=700,40,50,0,40,50", "--projection=o", "--imgsize=1100,700"],
}

LEGEND = [
    ("#d9b382", "Chassi: placas de MDF (estimado)"),
    ("#1a1a1a", "4 rodas e engrenagens, 2 servos STS3215"),
    ("#1f5fbf", "Arduino Mega (convés de cima)"),
    ("#16a085", "Adaptador de barramento do STS3215"),
    ("#2b8a3e", "Conversor buck 5 V"),
    ("#2a4d9b", "Bateria LiPo 2S"),
    ("#8e44ad", "IMU (direção)"),
    ("#a31f1f", "Linha de IR de 8 canais (sob o convés)"),
    ("#d63384", "2 sensores de cor do chão (sob o convés)"),
    ("#84cc16", "ToF: 2 laterais e 1 frontal na garra"),
    ("#f59e0b", "Sensor de cor do cubo (dedo da garra)"),
    ("#f1c40f", "Buzzer e LED (sinal do cubo branco)"),
    ("#d3d3d3", "Garra: mastro, rack e hastes"),
    ("#ff6347", "Garra: carriage com STS3215 e pinça com SG90"),
    ("#dc2626", "Cubo de 50 mm, para escala"),
]


def render(view: str, tmp: Path) -> Image.Image:
    png = tmp / f"{view}.png"
    subprocess.run(["openscad", "-o", str(png), *VIEWS[view], "--viewall", "--autocenter", str(HERE / "robot_completo.scad")],
                   check=True, capture_output=True)
    im = Image.open(png).convert("RGB")
    bbox = ImageChops.difference(im, Image.new("RGB", im.size, BG)).getbbox()
    return im.crop(bbox) if bbox else im


def fit(im: Image.Image, w: int, h: int) -> Image.Image:
    s = min(w / im.width, h / im.height)
    return im.resize((int(im.width * s), int(im.height * s)), Image.LANCZOS)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for p in ("/System/Library/Fonts/Helvetica.ttc", "/System/Library/Fonts/Supplemental/Arial.ttf"):
        try:
            return ImageFont.truetype(p, size, index=1 if bold and p.endswith(".ttc") else 0)
        except OSError:
            continue
    return ImageFont.load_default()


def main() -> None:
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        iso, top, side = (render(v, tmp) for v in ("iso", "top", "side"))
    W, H = 2400, 1720
    canvas = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(canvas)
    dr.text((60, 40), "Robô completo, RCB Challenge 2026", font=font(54, True), fill="#111")
    dr.text((60, 108), "Ilustração conceitual: a garra é o modelo real; chassi, eletrônica e posições dos sensores são estimados.",
            font=font(26), fill="#555")
    dr.text((1150, 52), "~230 x 168 x 145 mm com a garra aberta e abaixada (limite da regra: 300 x 300 mm)", font=font(26), fill="#333")
    iso = fit(iso, 1450, 1000)
    canvas.paste(iso, (60, 180))
    dr.text((70, 190), "Vista 3D", font=font(30, True), fill="#333")
    top = fit(top, 560, 640)
    canvas.paste(top, (1560, 180))
    dr.text((1560, 150), "Vista de cima (frente para cima)", font=font(28, True), fill="#333")
    side = fit(side, 760, 480)
    canvas.paste(side, (1560, 860))
    dr.text((1560, 820), "Vista lateral (frente à direita)", font=font(28, True), fill="#333")
    y0 = max(180 + iso.height, 860 + side.height) + 90      # legenda sempre abaixo das imagens
    dr.text((60, y0 - 50), "Legenda", font=font(32, True), fill="#111")
    for i, (col, label) in enumerate(LEGEND):
        cx, cy = 60 + (i % 3) * 760, y0 + (i // 3) * 46
        dr.rectangle([cx, cy, cx + 30, cy + 30], fill=col, outline="#333")
        dr.text((cx + 44, cy), label, font=font(27), fill="#222")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(OUT)
    print("ok", OUT)


if __name__ == "__main__":
    main()
