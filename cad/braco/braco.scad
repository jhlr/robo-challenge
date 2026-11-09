// Opcao 2: braco paralelogramo no topo do carrinho. 2 servos: ombro na base (STS3215 direto, sem reducao nem giro), pinca rotativa (SG90) na ponta.
// Parametros em params_gen.scad (gerado por cad/braco/cinematica.py a partir de params.json).
// Uso: openscad -o braco.png --camera=330,330,260,0,110,70 -D 'pose="pick"' braco.scad
include <../modelo_robo/robot_completo.scad>
include <params_gen.scad>
include <claw_rot.scad>
scene = "arm";
open = true;                  // pinca aberta?
psi_chk = 0;
claw_psi = -1;                // >= 0 sobrescreve a abertura da pinca (animacao)
pose = "pick";                // "pick" (cubo no chao), "mid" (levantado) ou "place" (cubo sobre a plataforma)
alpha_mid = -16;                                         // braco levantado o bastante para girar sem tocar nos sensores do chassi
alpha = pose == "pick" ? alpha_pick : pose == "mid" ? alpha_mid : alpha_place;

bar_w = 12; bar_t = 2.3;   // bar_t = espessura do MDF das barras (antes 3, das barras impressas)
 bar_x = 15; col_x = 12;
include <sts3215.scad>
include <ombro_sts.scad>                     // usa bar_x e z_s
u_arm = L1 * cos(alpha);                      // alcance horizontal do carrier a partir do eixo do giro
zl_arm = z_s + L1 * sin(alpha);               // altura do pivo de baixo do carrier

module bar(p0, p1, x) {                       // barra entre dois pontos (y, z), eixo dos pinos ao longo de x
    hull() for (p = [p0, p1]) translate([x - bar_t / 2, p[0], p[1]]) rotate([0, 90, 0]) cylinder(d = bar_w, h = bar_t);
}

module bars_only() {
    color("#374151") for (x = [-bar_x, bar_x]) {
        bar([0, z_s], [u_arm, zl_arm], x);                                             // barras de baixo
        bar([0, z_s + h_v], [u_arm, zl_arm + h_v], x);                                 // barras de cima (paralelas)
    }
    color("silver") {                                                                     // parafusos dos pivos da base
        translate([-bar_x - 6, 0, z_s + h_v]) rotate([0, 90, 0]) cylinder(d = 3, h = 2 * bar_x + 12);
        translate([-bar_x + bar_t / 2, 0, z_s]) rotate([0, 90, 0]) cylinder(d = 3, h = 2 * bar_x + 6 - bar_t / 2);   // embaixo: do lado -x o eixo e o do servo, so ha parafuso a partir da barra
    }
    color("#374151") translate([-bar_x - bar_t / 2, 0, z_s]) rotate([alpha, 0, 0]) rotate([0, 90, 0]) cylinder(d = sts_boss_d, h = bar_t, $fn = 48);   // pivo alargado da barra de baixo do lado -x
}
module bars() { bars_only(); sts_drive(); }          // STS3215 direto no pivo da barra -x

module carrier_geom() {                       // carrier (peca impressa): origem no eixo do pivo de baixo (y = u_arm, z = zl_arm)
    zb = -(carrier_below + 3);
    difference() {
        union() {
            for (sx = [-col_x, col_x]) box(sx - 1.5, sx + 1.5, -12, 12, zb, h_v + 8);        // placas laterais
            box(-18, 18, 10, 14, zb, h_v + 8);                                                 // placa frontal
        }
        for (z = [0, h_v]) translate([-20, 0, z]) rotate([0, 90, 0]) cylinder(d = 3.2, h = 40, $fn = 24);   // pivos das barras
        for (x = [-14, 14]) translate([x, 9, -carrier_below + cr_bz0 + 1.5]) rotate([-90, 0, 0]) cylinder(d = 3.2, h = 7, $fn = 24);   // M3 da moldura da pinca
    }
}

module carrier_and_claw() {
    zb = zl_arm - carrier_below - 3;                 // fundo das placas do carrier: igual ao fundo da pinca, fica acima do convés
    color("#6b7280") translate([0, u_arm, zl_arm]) carrier_geom();
    color("silver") for (z = [0, h_v]) translate([-bar_x - 6, u_arm, zl_arm + z]) rotate([0, 90, 0]) cylinder(d = 3, h = 2 * bar_x + 12);
    // pinca rotativa: origem local y = 0 na linha dos pivos; a placa traseira (y = -cr_rear) encosta na placa frontal do carrier
    translate([0, u_arm + carrier_front + cr_rear, zl_arm - carrier_below]) {
        color("tomato") cr_frame();
        color("khaki") cr_jaws(claw_psi >= 0 ? claw_psi : (open ? cr_psi_open : 0));
        cr_dummies();
    }
}

module arm() {
    translate([0, pivot_y, 0]) { bars(); carrier_and_claw(); }
}

module scene_arm() {
    cube_y = pivot_y + u_arm + claw_cube_center;                    // centro do cubo preso na pinca
    color("#e5e7eb") translate([-110, -100, -1]) cube([220, 330, 1]);                                       // piso
    robot_base(true);
    arm();
    if (pose == "pick") color("#dc2626") translate([-25, cube_y - 25, 0]) cube([50, 50, 50]);
    else {
        color("#16a34a") translate([-80, cube_y - 25, 0]) cube([160, 50, 50]);                              // plataforma 160 x 50 x 50
        color("#dc2626") translate([-25, cube_y - 25, 50]) cube([50, 50, 50]);                              // cubo sobre ela
    }
}

// animacao: o cubo anda junto com o carrier depois de agarrado (cube_mode "floor" fica no chao no ponto de pegar)
cube_mode = "carry"; show_platform = false;
module scene_anim() {
    zl_pick = z_s + L1 * sin(alpha_pick);
    cy = pivot_y + u_arm + claw_cube_center;
    cy_pick = pivot_y + L1 * cos(alpha_pick) + claw_cube_center;
    color("#e5e7eb") translate([-110, -100, -1]) cube([220, 330, 1]);
    robot_base(true);
    arm();
    if (cube_mode == "floor") color("#dc2626") translate([-25, cy_pick - 25, 0]) cube([50, 50, 50]);
    else if (cube_mode == "placed") color("#dc2626") translate([-25, pivot_y + L1 * cos(alpha_place) + claw_cube_center - 25, L1 * (sin(alpha_place) - sin(alpha_pick))]) cube([50, 50, 50]);
    else color("#dc2626") translate([-25, cy - 25, zl_arm - zl_pick]) cube([50, 50, 50]);
    if (show_platform) color("#16a34a", 0.5) translate([-80, pivot_y + L1 * cos(alpha_place) + claw_cube_center - 25, 0]) cube([160, 50, 50]);
}

if (scene == "arm") scene_arm();
if (scene == "anim") scene_anim();

// ---------- checagens (resultado vazio = sem colisao) ----------
module cube_at() { translate([-25, pivot_y + u_arm + claw_cube_center - 25, 0]) cube([50, 50, 50]); }
if (scene == "arm_only") arm();
if (scene == "chk_arm_chassis") intersection() { arm(); robot_base(true); }
if (scene == "chk_arm_cube") intersection() { arm(); cube_at(); }

// gears: setores acoplados nao podem se interpenetrar em nenhum angulo
if (scene == "chk_gears") intersection() { cr_jaw_right(psi_chk, 0); cr_jaw_left(psi_chk); }
if (scene == "jaws_top") { cr_frame(); cr_jaws(open ? cr_psi_open : 0); color("red") translate([-25, cr_cy - 25, 0]) cube([50, 50, 30]); }

// ---------- pecas para impressao (um STL por peca): openscad -o stl/x.stl -D 'scene="part_..."' braco.scad ----------
module jaw_print(right) { if (right) cr_jaw_right(0, 0); else cr_jaw_left(0); }   // ja saem na orientacao da montagem: engrenagem na mesa (com os dentes do lado do cubo cortados)
if (scene == "part_coupler") translate([0, 0, cr_cp_dz]) cr_coupler_local();   // como no modelo: dentes do grampo para baixo (imprimir com suporte so sob o disco); mandibula esquerda = espelho
if (scene == "part_frame") cr_frame();
if (scene == "part_jaw_r") jaw_print(true);
if (scene == "part_jaw_l") jaw_print(false);
if (scene == "part_carrier") translate([0, 0, 14]) rotate([-90, 0, 0]) carrier_geom();      // deitado sobre a placa frontal: sem suporte

// Extensor do eixo do servo: femea estriada (21 dentes, 4,8 x 4,3 mm, 5 mm de fundo) que entra no eixo + macho com a MESMA estria do eixo para onde o horn/engrenagem encaixa como se fosse o eixo original.
// ext_extra = comprimento que o eixo ganha. M2 do eixo passa por dentro (furo de 2,2): precisa de parafuso M2 comprido (ext_extra + cr_sp_depth + espessura do horn + ~3 mm).
ext_d = 8; ext_extra = 5; ext_male_len = 4;               // diametro externo, trecho cheio entre a femea e o macho, altura do macho (= saliencia do eixo original, 4 mm)
module shaft_extender() difference() {
    union() {
        cylinder(d = ext_d, h = cr_sp_depth + ext_extra, $fn = 64);
        translate([0, 0, cr_sp_depth + ext_extra - 0.01]) linear_extrude(height = ext_male_len + 0.01)
            polygon([for (i = [0 : 2 * cr_sp_n - 1]) let(a = i * 180 / cr_sp_n, r = (i % 2 == 0 ? cr_sp_od : cr_sp_id) / 2 - 0.05) [r * cos(a), r * sin(a)]]);   // macho, 0,05 mm menor que o eixo
    }
    translate([0, 0, -1]) cr_spline_pocket(cr_sp_depth + 1);                                                       // femea (cr_sp_fit de folga)
    translate([0, 0, -1]) cylinder(d = 2.2, h = cr_sp_depth + ext_extra + ext_male_len + 2, $fn = 24);             // M2 passante
}
if (scene == "part_ext_eixo") shaft_extender();   // em pe, femea (aberta) na mesa, macho para cima
