"""Renderiza braco.scad (pegar e soltar) e compoe docs/2026-10-20_braco-opcao2.png.

Uso: python3 braco/braco_ilustracao.py
Chassi, eletronica e posicoes dos sensores sao estimados; numeros do braco vem de cinematica.py / params.json.
"""
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cinematica as K  # noqa: E402

OUT = HERE.parent / "docs" / "2026-10-20_braco-opcao2.png"
BG = (255, 255, 229)
ISO = ["--camera=-330,330,260,0,110,70", "--imgsize=1500,1100"]
SIDE = ["--camera=800,100,100,0,100,100", "--projection=o", "--imgsize=1500,900"]

LEGEND = [
    ("#d4af37", "STS3215 do ombro, eixo horizontal"),
    ("#374151", "4 barras do paralelogramo (a pinça fica nivelada)"), ("#6b7280", "Carrier: placas laterais e frontal"),
    ("#ff6347", "Ponte, anteparo e prateleira da pinça"), ("#e8dc8a", "2 mandíbulas com setores de engrenagem"),
    ("#4169e1", "SG90 ou MG90S que fecha a pinça"), ("#84cc16", "ToF frontal e laterais, no chassi"),
    ("#f59e0b", "Sensor de cor do cubo, atrás do anteparo"), ("#1f5fbf", "Arduino Mega deitado no fundo"),
    ("#d9b382", "Chassi (estimado)"), ("#dc2626", "Cubo de 50 mm e plataforma verde (160 x 50 x 50)"),
]


def render(extra: list[str], pose: str, tmp: Path, name: str) -> Image.Image:
    png = tmp / f"{name}.png"
    subprocess.run(["openscad", "-o", str(png), *extra, "--viewall", "--autocenter", "-D", f'pose="{pose}"', str(HERE / "braco.scad")],
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
        t = Path(d)
        imgs = {(v, p): render(ISO if v == "iso" else SIDE, p, t, f"{v}_{p}") for v in ("iso", "side") for p in ("pick", "place")}
    W, H = 2400, 2080
    c = Image.new("RGB", (W, H), "white")
    dr = ImageDraw.Draw(c)
    dr.text((60, 36), "Opção 2: braço paralelogramo no topo do carrinho", font=font(54, True), fill="#111")
    dr.text((60, 108), "3 servos: giro e ombro na base, pinça na ponta. Chassi, eletrônica e sensores são estimados; a pinça tem duas mandíbulas engrenadas que giram e fecham em ângulo no cubo.",
            font=font(26), fill="#555")
    col_w, row1, row2 = 1130, 190, 1010
    for i, (pose, title) in enumerate((("pick", f"Pegar no chão (barras a {K.a_pick:.0f} graus)"), ("place", f"Soltar na plataforma (barras a {K.a_place:.0f} graus)"))):
        x = 60 + i * (col_w + 20)
        dr.text((x, row1 - 40), title, font=font(30, True), fill="#333")
        iso = fit(imgs[("iso", pose)], col_w, 760)
        c.paste(iso, (x, row1))
        side = fit(imgs[("side", pose)], col_w, 640)
        dr.text((x, row2 - 36), "Vista lateral (frente à direita)", font=font(26, True), fill="#333")
        c.paste(side, (x, row2))
    y0 = 1750
    dr.text((60, y0 - 46), "Legenda", font=font(32, True), fill="#111")
    for i, (col, label) in enumerate(LEGEND):
        cx, cy = 60 + (i % 2) * 600, y0 + (i // 2) * 42
        dr.rectangle([cx, cy, cx + 28, cy + 28], fill=col, outline="#333")
        dr.text((cx + 40, cy), label, font=font(24), fill="#222")
    k = K.summary()
    nums = [f"Cubo preso fica {k['beyond_min']:.0f} a {k['beyond_max']:.0f} mm além da frente do chassi (pegar e soltar)",
            f"Comprimento total do robô: até {k['length_max']:.0f} mm (limite 300 mm)",
            "Sem giro: a pinça fica sempre à frente; mira girando o robô",
            f"Torque no ombro: {k['torque_x2']:.1f} kg.cm com folga 2x; o STS3215 tem 19,5",
            f"Aperto por mandíbula: {K.clamp_force_n(1.8):.1f} N com SG90 e {K.clamp_force_n(2.4):.1f} N com MG90S; mínimo {K.required_clamp_n():.1f} N"]
    dr.text((1330, y0 - 46), "Números do projeto", font=font(32, True), fill="#111")
    for i, line in enumerate(nums):
        dr.text((1330, y0 + i * 52), line, font=font(24), fill="#222")
    c.save(OUT)
    print("ok", OUT)


if __name__ == "__main__":
    main()
