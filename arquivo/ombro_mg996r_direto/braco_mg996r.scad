// Opcao 2: braco paralelogramo no topo do carrinho. 2 servos: ombro na base (MG996R direto no pivo, sem giro), pinca rotativa (SG90) na ponta.
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

bar_w = 12; bar_t = 3; bar_x = 15; col_x = 12;
include <ombro_direto.scad>                  // usa bar_x e z_s
u_arm = L1 * cos(alpha);                      // alcance horizontal do carrier a partir do eixo do giro
zl_arm = z_s + L1 * sin(alpha);               // altura do pivo de baixo do carrier

module bar(p0, p1, x) {                       // barra entre dois pontos (y, z), eixo dos pinos ao longo de x
    hull() for (p = [p0, p1]) translate([x - bar_t / 2, p[0], p[1]]) rotate([0, 90, 0]) cylinder(d = bar_w, h = bar_t);
}

module mount_geom() {                         // suporte fixo (peca impressa): prato, berco do STS3215, torres do paralelogramo
    zt = z_deck; zp = zt + base_t; tw = 4;     // tw: espessura das torres (cresce para dentro)
    difference() {
        union() {
            box(-53, 20, -22, 22, zt, zp);                                                   // prato
            box(-51.5, -16.5, -12.35, 12.35, zp, z_s - 11.3);                                // apoio sob o servo
            for (s = [-1, 1]) box(-51.5, -16.5, s > 0 ? 12.35 : -14.85, s > 0 ? 14.85 : -12.35, zp, z_s + 2);   // abas laterais do berco
            for (s = [-1, 1]) {
                x0 = s > 0 ? col_x + 1.5 - tw : -col_x - 1.5;                                // torre: face externa em |x| = col_x + 1.5
                box(x0, x0 + tw, -12, 12, zp, z_s + h_v + 8);
                translate([x0, 0, zp]) rotate([90, 0, 90]) linear_extrude(height = tw) polygon([[-22, 0], [-12, 0], [-12, 45]]);   // reforco traseiro
            }
        }
        for (z = [z_s, z_s + h_v]) translate([-15, 0, z]) rotate([0, 90, 0]) cylinder(d = 3.2, h = 30, $fn = 24);   // eixos dos pivos
        for (x = [-50.5, 16.5], y = [-19, 19]) translate([x, y, zt - 1]) cylinder(d = 3.4, h = base_t + 2, $fn = 24); // M3 no convés
        for (x = [-44, -24], s = [-1, 1]) box(x - 3, x + 3, s > 0 ? 16 : -18.5, s > 0 ? 18.5 : -16, zt - 1, zp + 1);   // fendas para abracadeira de nylon do servo
    }
}
module mount() {
    color("#9ca3af") mount_geom();
}

module bars() {
    color("#374151") for (x = [-bar_x, bar_x]) {
        bar([0, z_s], [u_arm, zl_arm], x);                                             // barras de baixo
        bar([0, z_s + h_v], [u_arm, zl_arm + h_v], x);                                 // barras de cima (paralelas)
    }
    color("silver") for (z = [0, h_v]) translate([-bar_x - 6, 0, z_s + z]) rotate([0, 90, 0]) cylinder(d = 3, h = 2 * bar_x + 12);   // parafusos dos pivos da base
    sh_drive(alpha);                                                                      // MG996R do ombro, direto no pivo (corpo fora da barra esquerda)
}

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
    translate([0, pivot_y, 0]) { mount(); bars(); carrier_and_claw(); }
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
module bar_flat() difference() {                // barra do paralelogramo, deitada: 12 x (L1 + 12) x 3, furos M3 nos pivos
    hull() for (y = [0, L1]) translate([0, y, 0]) cylinder(d = bar_w, h = bar_t, $fn = 40);
    for (y = [0, L1]) translate([0, y, -1]) cylinder(d = 3.2, h = bar_t + 2, $fn = 24);
}
module jaw_print(right) { if (right) cr_jaw_right(0, 0); else cr_jaw_left(0); }   // ja saem na orientacao da montagem: engrenagem na mesa (com os dentes do lado do cubo cortados)
if (scene == "part_coupler") translate([0, 0, cr_cp_dz]) cr_coupler_local();   // como no modelo: dentes do grampo para baixo (imprimir com suporte so sob o disco); mandibula esquerda = espelho
if (scene == "part_frame") cr_frame();
if (scene == "part_jaw_r") jaw_print(true);
if (scene == "part_jaw_l") jaw_print(false);
if (scene == "part_bar") bar_flat();
if (scene == "part_mount") mount_geom();
if (scene == "part_carrier") translate([0, 0, 14]) rotate([-90, 0, 0]) carrier_geom();      // deitado sobre a placa frontal: sem suporte
