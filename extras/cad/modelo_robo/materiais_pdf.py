"""Gera docs/2026-10-19_lista-de-materiais.pdf (lista de materiais da garra + chassi).

Uso: python3 cad/modelo_robo/materiais_pdf.py
Fotos: cad/modelo_robo/materiais_img/ (4 da Wikimedia Commons, creditadas no rodape; 2 do proprio usuario).
Itens sem foto livre usam desenho esquemático.
"""
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Flowable, Image, KeepTogether, Paragraph, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

ROOT = Path(__file__).resolve().parent
IMG = ROOT / "materiais_img"
OUT = ROOT.parents[1] / "docs" / "2026-10-19_lista-de-materiais.pdf"

ss = getSampleStyleSheet()
H1 = ParagraphStyle("h1", parent=ss["Title"], fontSize=20, leading=24, alignment=0, spaceAfter=2)
SUB = ParagraphStyle("sub", parent=ss["Normal"], fontSize=9.5, leading=13, textColor=colors.HexColor("#555555"))
H2 = ParagraphStyle("h2", parent=ss["Heading2"], fontSize=13, leading=16, spaceBefore=10, spaceAfter=4,
                    textColor=colors.HexColor("#1f3a5f"))
NAME = ParagraphStyle("name", parent=ss["Normal"], fontName="Helvetica-Bold", fontSize=10.5, leading=13)
BODY = ParagraphStyle("body", parent=ss["Normal"], fontSize=9, leading=12)
SMALL = ParagraphStyle("small", parent=ss["Normal"], fontSize=7.5, leading=10, textColor=colors.HexColor("#666666"))

GRAY = colors.HexColor("#8a8f98")
BLUE = colors.HexColor("#2f5aa8")


class Schem(Flowable):
    """Desenho esquemático simples (não e foto)."""

    def __init__(self, kind, w=42 * mm, h=30 * mm):
        super().__init__()
        self.kind, self.width, self.height = kind, w, h

    def draw(self):
        c, w, h, k = self.canv, self.width, self.height, self.kind
        c.setStrokeColor(colors.HexColor("#444444"))
        c.setLineWidth(0.8)
        cx, cy = w / 2, h / 2
        if k == "n20":
            c.setFillColor(colors.HexColor("#c9a227")); c.rect(4 * mm, cy - 6 * mm, 12 * mm, 12 * mm, fill=1)   # redutor
            c.setFillColor(GRAY); c.rect(16 * mm, cy - 6 * mm, 20 * mm, 12 * mm, fill=1)                        # motor
            c.setFillColor(colors.HexColor("#dddddd")); c.rect(0, cy - 1.5 * mm, 4 * mm, 3 * mm, fill=1)        # eixo D
            c.setFont("Helvetica", 6); c.setFillColor(colors.black)
            c.drawString(2 * mm, cy - 10 * mm, "12 x 10 x 25 mm, eixo D 3 mm")
        elif k == "switch":
            c.setFillColor(colors.HexColor("#d9c36a")); c.rect(8 * mm, cy - 5 * mm, 26 * mm, 10 * mm, fill=1)
            c.line(10 * mm, cy + 5 * mm, 36 * mm, cy + 11 * mm)
            c.setFillColor(colors.black); c.circle(13 * mm, cy, 1 * mm, fill=1); c.circle(29 * mm, cy, 1 * mm, fill=1)
            c.setFont("Helvetica", 6); c.drawString(4 * mm, cy - 9 * mm, "13 x 6 x 7 mm, com alavanca")
        elif k == "rod":
            c.setFillColor(colors.HexColor("#c8ccd2")); c.roundRect(3 * mm, cy - 2 * mm, w - 6 * mm, 4 * mm, 2, fill=1)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(8 * mm, cy - 7 * mm, "haste de aço, 4 mm de diâmetro")
        elif k == "screw":
            c.setFillColor(colors.HexColor("#b9bec6")); c.circle(8 * mm, cy, 4 * mm, fill=1); c.rect(12 * mm, cy - 1.5 * mm, 18 * mm, 3 * mm, fill=1)
            c.setFillColor(colors.HexColor("#d0d4da")); c.rect(33 * mm, cy - 4 * mm, 6 * mm, 8 * mm, fill=1)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(4 * mm, cy - 9 * mm, "parafuso M3/M2 + porca")
        elif k == "tcs":
            c.setFillColor(colors.HexColor("#b03a3a")); c.rect(cx - 9 * mm, cy - 9 * mm, 18 * mm, 18 * mm, fill=1)
            c.setFillColor(colors.white); c.rect(cx - 3 * mm, cy - 3 * mm, 6 * mm, 6 * mm, fill=1)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(cx - 17 * mm, cy - 13 * mm, "placa de sensor de cor, I2C")
        elif k == "vl53":
            c.setFillColor(colors.HexColor("#2e5e3e")); c.rect(cx - 7 * mm, cy - 5 * mm, 14 * mm, 10 * mm, fill=1)
            c.setFillColor(colors.black); c.rect(cx - 2 * mm, cy - 2 * mm, 4 * mm, 4 * mm, fill=1)
            c.setFont("Helvetica", 6); c.drawString(cx - 17 * mm, cy - 10 * mm, "sensor de distância por laser, I2C")
        elif k == "lipo":
            c.setFillColor(BLUE); c.roundRect(5 * mm, cy - 7 * mm, 30 * mm, 14 * mm, 2, fill=1)
            c.setStrokeColor(colors.red); c.line(35 * mm, cy + 3 * mm, 40 * mm, cy + 3 * mm)
            c.setStrokeColor(colors.black); c.line(35 * mm, cy - 3 * mm, 40 * mm, cy - 3 * mm)
            c.setFont("Helvetica-Bold", 7); c.setFillColor(colors.white); c.drawString(10 * mm, cy - 1 * mm, "LiPo 2S 7,4 V")
        elif k == "eva":
            c.setFillColor(colors.HexColor("#6aa84f")); c.rect(5 * mm, cy - 8 * mm, 32 * mm, 16 * mm, fill=1)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(5 * mm, cy - 12 * mm, "folha de EVA/silicone, 1 a 2 mm")
        elif k == "adapter":
            c.setFillColor(colors.HexColor("#1b4b72")); c.rect(5 * mm, cy - 8 * mm, 32 * mm, 16 * mm, fill=1)
            c.setFillColor(colors.HexColor("#eeeeee")); c.rect(7 * mm, cy - 3 * mm, 8 * mm, 6 * mm, fill=1); c.rect(27 * mm, cy - 3 * mm, 8 * mm, 6 * mm, fill=1)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(4 * mm, cy - 12 * mm, "placa USB/TTL para servo serial")
        elif k == "wires":
            c.setLineWidth(1.6)
            for i, col in enumerate(["#d33", "#e90", "#3a3", "#36c"]):
                c.setStrokeColor(colors.HexColor(col)); c.bezier(4 * mm, cy + (i - 1.5) * 3 * mm, 15 * mm, cy + 10 * mm, 28 * mm, cy - 10 * mm, 38 * mm, cy + (i - 1.5) * 3 * mm)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(4 * mm, 1 * mm, "jumpers, termo-retrátil, fita dupla face")
        elif k == "filament":
            c.setFillColor(colors.HexColor("#e8e8e8")); c.circle(cx, cy, 12 * mm, fill=1); c.setFillColor(colors.white); c.circle(cx, cy, 4 * mm, fill=1)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(cx - 14 * mm, 1 * mm, "filamento PLA/PETG (+ TPU opcional)")
        elif k == "fuse":
            c.setFillColor(colors.HexColor("#d9534f")); c.roundRect(5 * mm, cy - 3 * mm, 14 * mm, 6 * mm, 1, fill=1)
            c.setFillColor(colors.HexColor("#333333")); c.roundRect(23 * mm, cy - 6 * mm, 14 * mm, 12 * mm, 2, fill=1)
            c.setFillColor(colors.white); c.rect(28 * mm, cy - 2 * mm, 4 * mm, 4 * mm, fill=1)
            c.setFont("Helvetica", 6); c.setFillColor(colors.black); c.drawString(4 * mm, cy - 10 * mm, "fusível 5 A, chave liga/desliga, fio 18 AWG")


PLA_G_CM3 = 1.24


def stl_mass_g(name: str) -> float:
    """Massa maciça (PLA) de um STL ASCII em cad/modelo_robo/stl/ (soma dos tetraedros com a origem)."""
    pts = [tuple(map(float, ln.split()[1:])) for ln in (ROOT / "stl" / name).read_text().splitlines() if "vertex" in ln]
    vol = 0.0
    for i in range(0, len(pts), 3):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = pts[i:i + 3]
        vol += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6
    return abs(vol) / 1000 * PLA_G_CM3


def g(x: float) -> str:
    return f"{x:.1f} g".replace(".", ",")


def photo(name, w=42 * mm, h=30 * mm):
    path = IMG / name
    from PIL import Image as PI
    iw, ih = PI.open(path).size
    s = min(w / iw, h / ih)
    return Image(str(path), width=iw * s, height=ih * s)


def card(pic, title, qty, spec, where, note=""):
    body = [Paragraph(title, NAME),
            Paragraph(f"<b>Quantidade:</b> {qty}", BODY),
            Paragraph(f"<b>Especificação:</b> {spec}", BODY)]
    if where != "-":
        body.append(Paragraph(f"<b>Buscar como:</b> {where}", BODY))
    if note:
        body.append(Paragraph(f"<i>{note}</i>", BODY))
    t = Table([[pic, body]], colWidths=[46 * mm, 124 * mm])
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "MIDDLE"), ("BOX", (0, 0), (-1, -1), 0.5, colors.HexColor("#cccccc")),
                           ("LEFTPADDING", (0, 0), (-1, -1), 5), ("TOPPADDING", (0, 0), (-1, -1), 4), ("BOTTOMPADDING", (0, 0), (-1, -1), 4)]))
    return KeepTogether([t, Spacer(1, 3)])


def build():
    doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm, topMargin=16 * mm, bottomMargin=16 * mm,
                            title="Lista de materiais - Garra RCB Challenge 2026", author="Equipe")
    s = []
    s += [Paragraph("Lista de materiais - Garra RCB Challenge 2026", H1),
          Paragraph("19/10/2026. Garra de pinça com elevador de cremalheira, para o chassi novo com servos STS3215 (um por lado, "
                    "2 rodas por servo) e Arduino Mega. Itens com foto real são fotos; os demais tem desenho esquemático.", SUB), Spacer(1, 4)]

    s.append(Paragraph("1. Já temos", H2))
    s.append(card(photo("chassi_rodas.jpg"), "Chassi novo: placas de MDF, engrenagens e 4 rodas", "1 conjunto",
                  "um servo STS3215 por lado, engrenagem central e uma engrenagem por roda", "-",
                  "Foto da equipe."))
    s.append(card(photo("sts3215_etiqueta.jpg"), "Servo de barramento STS3215 (Feetech)", "2 (um por lado)",
                  "16,5 kg.cm a 6 V e 19,5 kg.cm a 7,4 V; 0,192 s por 60 graus a 7,4 V; corpo 45,22 x 24,7 x 35 mm; sinal serial digital",
                  "-", "Foto da etiqueta da equipe."))
    s.append(card(photo("sts3215_etiqueta.jpg"), "Servo STS3215 extra, para o elevador da garra", "1 (o terceiro)",
                  "mesmo modelo do chassi, 45,22 x 24,7 x 35 mm; sobe e desce a garra pelo pinhão", "-",
                  "Substitui o motor N20, os fins de curso e o L298N."))

    s.append(Paragraph("2. Comprar: garra", H2))
    s.append(card(photo("sg90.jpg"), "Micro servo SG90 ou MG90S", "2 (1 + 1 reserva)", "9 g, 4,8 a 6 V, com braco simples; MG90S (engrenagem de metal) é mais forte",
                  "micro servo MG90S", "Abre e fecha a pinça."))
    s.append(card(Schem("rod"), "Hastes de aço para guia", "2", "4 mm de diâmetro, 125 mm de comprimento (cortar em 120 mm)",
                  "eixo de aco 4mm", "O professor já vai comprar os eixos."))
    s.append(card(Schem("screw"), "Parafusos e porcas", "M3: cerca de 20; M2: cerca de 10", "M3 de 8, 10 e 12 mm; M2 de 8 a 10 mm; porcas correspondentes",
                  "parafuso M3 sextavado interno / M2", "M3 prende a base no chassi; M2 prende o servo."))
    s.append(card(Schem("eva"), "Borracha para os dedos", "1 folha pequena", "EVA ou silicone de 1 a 2 mm, colada nos dedos para dar atrito",
                  "folha EVA 2mm", "Alternativa: imprimir em TPU."))
    s.append(card(Schem("wires"), "Diversos", "1 kit", "jumpers macho-femea, termo-retrátil, fita dupla face, cola cianoacrilato", "-"))
    s.append(card(Schem("filament"), "Filamento", "cerca de 80 g", "PLA ou PETG para as peças da garra (3 perímetros); TPU opcional", "-",
                  "Peças pesam cerca de 100 g maciças (ver seção 5)."))

    s.append(Paragraph("3. Comprar: sensores", H2))
    s.append(card(Schem("tcs"), "Sensor de cor do cubo TCS34725", "2 (1 + 1 reserva)", "I2C, 3,3 a 5 V; plano B: TCS3200",
                  "sensor de cor TCS34725", "O sensor de cor do chão não le o cubo."))
    s.append(card(Schem("vl53"), "Sensor de distância VL53L0X ou VL53L1X", "2", "laser, I2C; plano B: 3 x HC-SR04",
                  "VL53L0X módulo", "Regra exige sensor de distância (obstaculos e outros robos)."))
    s.append(card(photo("hcsr04.jpg"), "Plano B: sensor ultrassônico HC-SR04", "3", "5 V, 2 a 400 cm, 4 pinos", "HC-SR04"))

    s.append(Paragraph("4. Comprar: energia e controle do chassi novo", H2))
    s.append(card(Schem("adapter"), "Adaptador de barramento para servo serial", "1", "tipo Waveshare Bus Servo Adapter (A) ou Feetech FE-URT-1; liga o Mega aos STS3215",
                  "Waveshare Bus Servo Adapter / FE-URT-1", "Provavelmente só por encomenda."))
    s.append(card(Schem("lipo"), "Bateria LiPo 2S", "1", "7,4 V, 1500 a 2200 mAh, conector XT60; levar carregador balanceado e saco anti-chama",
                  "bateria LiPo 2S 7.4V XT60", "Os STS3215 pedem 7,4 V para o torque maximo."))
    s.append(card(photo("lm2596.jpg"), "Conversor buck de 5 V", "1", "entrada 7,4 V, saida 5 V, ao menos 3 A (módulo LM2596 ou similar)",
                  "step down LM2596 3A", "Alimenta o Mega, o SG90 e os sensores."))
    s.append(card(Schem("fuse"), "Proteção", "1 de cada", "fusível de 5 A, chave liga/desliga, fio 18 a 20 AWG", "porta fusível + chave gangorra"))

    s.append(Paragraph("5. Peças impressas em 3D", H2))
    m = {n: stl_mass_g(n + ".stl") for n in ("mast", "carriage", "slider_a", "slider_b", "pinion", "horn")}
    total = sum(m.values())
    data = [["Peça", "Qtd", "Massa (PLA maciço)"], ["Mastro (parede, base, cremalheira)", "1", g(m["mast"])],
            ["Carriage + suporte da pinça", "1", g(m["carriage"])], ["Sliders com dedos", "2", g(m["slider_a"] + m["slider_b"])],
            ["Pinhão 14 dentes", "1", g(m["pinion"])], ["Horn de 2 pinos", "1", g(m["horn"])], ["Total", "", g(total)]]
    t = Table(data, colWidths=[100 * mm, 20 * mm, 40 * mm])
    t.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#bbbbbb")), ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e8eef7")),
                           ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"), ("FONTNAME", (0, -1), (-1, -1), "Helvetica-Bold"), ("FONTSIZE", (0, 0), (-1, -1), 9)]))
    s += [t, Spacer(1, 3), Paragraph("Arquivos em cad/modelo_robo/stl/. A massa real depende do preenchimento (infill) escolhido.", SMALL), Spacer(1, 10)]

    s.append(KeepTogether([Paragraph("Créditos das fotos", H2), Paragraph("Fotos redimensionadas, da Wikimedia Commons: SG90 por Suyash Dwivedi (CC BY-SA 4.0); "
                       "HC-SR04 por SparkFun (CC BY 2.0); módulo LM2596 por Matthew Berardi (CC BY-SA 4.0). "
                       "Fotos do chassi e da etiqueta do STS3215: equipe. Desenhos esquemáticos: gerados para este documento.", SMALL)]))
    doc.build(s)
    print("ok", OUT)


if __name__ == "__main__":
    build()
