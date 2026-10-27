"""Cinematica, regra dos 300 x 300 mm e torque do braco paralelogramo (opcao 2). Fonte unica: params.json.

Uso: python3 cad/braco/cinematica.py   (imprime o relatorio e gera cad/braco/params_gen.scad para o modelo 3D)
"""
from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
P = json.loads((HERE / "params.json").read_text())
C, A, M, CL = P["chassis"], P["arm"], P["mass_g"], P["claw"]

# ---- pinca rotativa: tudo derivado de params.json["claw"] ----
cl_rc = CL["gm"] * CL["gz"] / 2                                   # pivo -> centro (raio primitivo das engrenagens)
cl_ra = A["cube"] * math.sqrt(2) / 2                              # raio do arco interno da mandibula (meia diagonal do cubo)


def _half_open() -> float:
    """Meia largura com as mandibulas abertas: maior x do arco (face externa) girado psi_open em torno do pivo."""
    cy = CL["stop_y"] + CL["stop_t"] + A["cube"] / 2
    a1 = math.degrees(math.asin(CL["hook_dy"] / cl_ra))
    psi = math.radians(CL["psi_open"])
    pts = [(r * math.cos(math.radians(-45 + (a1 + 45) * i / 16)) - cl_rc, cy + r * math.sin(math.radians(-45 + (a1 + 45) * i / 16)))
           for i in range(17) for r in (cl_ra, cl_ra + CL["jaw_t"])]
    return max(cl_rc + x * math.cos(psi) + y * math.sin(psi) for x, y in pts)


cl_half_open = _half_open()                                       # meia largura com as mandibulas abertas
cl_cube_center = A["carrier_front"] + CL["rear_gap"] + CL["stop_y"] + CL["stop_t"] + A["cube"] / 2   # eixo das barras -> centro do cubo preso
cl_tip = A["carrier_front"] + CL["rear_gap"] + CL["jaw_len"]                                          # eixo das barras -> ponta das mandibulas

z_s = C["z_deck_top"] + A["base_t"] + A["shoulder_above_base"]   # eixo do ombro (altura do chao)


def alpha_for_finger_bottom(z_fb: float) -> float:
    """Angulo das barras (graus) para o fundo dos dedos ficar em z_fb."""
    return math.degrees(math.asin((z_fb - z_s + A["carrier_below"]) / A["L1"]))


z_fb_pick = A["finger_bottom_pick"]
z_fb_place = A["platform_h"] + A["place_clearance"] + A["cube_grip_offset"]
a_pick, a_place = alpha_for_finger_bottom(z_fb_pick), alpha_for_finger_bottom(z_fb_place)


def u(alpha_deg: float) -> float:
    return A["L1"] * math.cos(math.radians(alpha_deg))


def torque_kgcm(alpha_deg: float) -> float:
    uc = u(alpha_deg) / 1000
    n_m = (M["claw_group"] / 1000 * 9.81 * (uc + 0.035) + M["cube"] / 1000 * 9.81 * (uc + cl_cube_center / 1000)
           + M["carrier_bars"] / 1000 * 9.81 * (uc / 2))
    return n_m / 9.81 * 100


def report() -> str:
    rows = []
    for name, a in (("pegar no chao", a_pick), ("soltar na plataforma", a_place)):
        uc = u(a)
        r_center = uc + cl_cube_center
        beyond = r_center - A["cube"] / 2 - (C["half_l"] - C["pivot_y"])
        length = C["half_l"] + C["pivot_y"] + r_center + A["cube"] / 2
        rows.append((name, a, uc, r_center, beyond, length, torque_kgcm(a)))
    out = [f"Eixo do ombro a {z_s:.0f} mm do chao; barras L1 = {A['L1']} mm, claw pendurada {A['carrier_below']} mm abaixo da barra de baixo.", "",
           "| pose | angulo | alcance do carrier | centro do cubo | cubo alem da frente | comprimento total |  torque no ombro |",
           "|---|---|---|---|---|---|---|"]
    for n, a, uc, rc, bey, ln, tq in rows:
        out.append(f"| {n} | {a:.1f} graus | {uc:.0f} mm | {rc:.0f} mm | {bey:.0f} mm | {ln:.0f} mm | {tq:.1f} kg.cm |")
    worst = max(r[6] for r in rows)
    out += ["", f"Pinca rotativa: aperto por mandibula {clamp_force_n(1.8):.1f} N com SG90 (1,8 kg.cm) e {clamp_force_n(2.4):.1f} N com MG90S; "
            f"minimo para segurar o cubo com margem 3x: {required_clamp_n():.1f} N. Meia largura aberta: {cl_half_open:.0f} mm."]
    out += ["", f"Regra: comprimento <= {P['rule_box_mm']} mm: {'OK' if max(r[5] for r in rows) <= P['rule_box_mm'] else 'VIOLA'}; "
            f"largura com a pinca aberta {2 * cl_half_open:.0f} mm: {'OK' if 2 * cl_half_open <= P['rule_box_mm'] else 'VIOLA'} (sem giro: a pinca fica sempre a frente).",
            f"Torque estatico no ombro, pior caso {worst:.1f} kg.cm; com fator 2 para aceleracao: {2 * worst:.1f} kg.cm.", "",
            "| servo | torque | margem sobre o ombro |", "|---|---|---|"]
    ratio = A["shoulder_ratio"]
    del out[-2:]                                                  # troca o cabecalho antigo (so servo direto) pelo com reducao
    out += [f"| servo | torque do servo | no braco com reducao {ratio:g}:1 | margem sobre o ombro (direto / com reducao) |", "|---|---|---|---|"]
    for s, t in P["servo_kgcm"].items():
        out.append(f"| {s} | {t} kg.cm | {t * ratio:.1f} kg.cm | {t / (2 * worst):.1f}x / {t * ratio / (2 * worst):.1f}x |")
    width = 2 * max(C["wheel_outer_half"], C["upper_deck_half"], cl_half_open)
    out += ["", f"Largura total do robo (rodas externas {2 * C['wheel_outer_half']} mm, convés de cima com asas {2 * C['upper_deck_half']:.0f} mm, pinca aberta {2 * cl_half_open:.0f} mm): "
            f"{width:.0f} mm, limite {P['rule_box_mm']} mm: {'OK' if width <= P['rule_box_mm'] else 'VIOLA'}; folga no comprimento: {P['rule_box_mm'] - max(r[5] for r in rows):.0f} mm."]
    return "\n".join(out)


def summary() -> dict[str, float]:
    """Numeros-chave para a ilustracao: unica fonte de verdade."""
    poses = {n: u(a) for n, a in (("pick", a_pick), ("place", a_place))}
    return {
        "beyond_min": min(poses[n] + cl_cube_center - A["cube"] / 2 - (C["half_l"] - C["pivot_y"]) for n in poses),
        "beyond_max": max(poses[n] + cl_cube_center - A["cube"] / 2 - (C["half_l"] - C["pivot_y"]) for n in poses),
        "length_max": max(C["half_l"] + C["pivot_y"] + poses[n] + cl_cube_center + A["cube"] / 2 for n in poses),
        "torque_x2": 2 * max(torque_kgcm(a_pick), torque_kgcm(a_place)),
    }


def clamp_force_n(servo_kgcm: float) -> float:
    """Forca de aperto em cada mandibula: o torque do servo vai ao pivo (engrenagens iguais) e age a clamp_arm_mm."""
    return servo_kgcm * 9.81 / 100 / (CL["clamp_arm_mm"] / 1000)


def required_clamp_n() -> float:
    """Forca minima para segurar o cubo por atrito, com margem 3x para aceleracao."""
    return 3 * M["cube"] / 1000 * 9.81 / (2 * CL["friction"])


def write_scad() -> None:
    lines = ["// GERADO por cad/braco/cinematica.py a partir de params.json. Nao editar.",
             f"z_s = {z_s};", f"L1 = {A['L1']};", f"h_v = {A['h_v']};", f"carrier_below = {A['carrier_below']};",
             f"claw_cube_center = {cl_cube_center};", f"cr_gm = {CL['gm']};", f"cr_gz = {CL['gz']};", f"cr_jt = {CL['jaw_t']};",
             f"cr_jlen = {CL['jaw_len']};", f"cr_jback = {CL['jaw_back']};", f"cr_jh = {CL['jaw_h']};", f"cr_psi_open = {CL['psi_open']};",
             f"cr_gz0 = {CL['gear_z0']};", f"cr_gt = {CL['gear_t']};", f"cr_bz0 = {CL['bridge_z0']};", f"cr_bt = {CL['bridge_t']};",
             f"cr_stop_y = {CL['stop_y']};", f"cr_stop_t = {CL['stop_t']};", f"cr_rear = {CL['rear_gap']};", f"cr_bl = {CL['bl']};", f"cr_cube = {A['cube']};", f"cube_grip = {A['cube_grip_offset']};", f"cr_hook = {CL['hook_dy']};",
             f"carrier_front = {A['carrier_front']};", f"alpha_pick = {a_pick:.3f};", f"alpha_place = {a_place:.3f};", f"pivot_y = {C['pivot_y']};",
             f"z_deck = {C['z_deck_top']};", f"base_t = {A['base_t']};"]
    (HERE / "params_gen.scad").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    print(report())
    write_scad()
