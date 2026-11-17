// Versao em MDF encaixavel (corte a laser/CNC 2D) do braco da opcao 2. Mesma geometria de braco.scad, pecas planas com juntas tab-slot.
// Uso 3D:  openscad -o mdf_arm.png --camera=330,330,260,0,110,70 -D 'pose="pick"' braco_mdf.scad
// Uso 2D:  openscad -o peca.svg -D 'scene="2d_tower"' braco_mdf.scad   (nomes em PARTS_2D; mdf_nest.py gera as chapas)
// Todas as pecas 2D usam coordenadas naturais do plano em que ficam (XY, XZ ou YZ do braco); mdf_nest.py reposiciona ao cortar.
include <braco.scad>

mt = 2.3;             // espessura do MDF (cupom: rasgo nominal de 2,4 mm ficou levemente solto; material ~2,3 mm)
fit = 0;              // folga nominal dos rasgos: 0, porque o kerf do laser ja abre ~0,1-0,2 mm (acertado pelo cupom)

// ---------- derivados ----------
mz_b = z_deck; mz_p = mz_b + mt; mz_t = z_s + h_v + 8;               // prato do suporte: embaixo, em cima; topo das torres
m_pl = [sh_ear_x - sh_ear_t - mt, sh_ear_x - sh_ear_t];       // x da chapa do MG996R (ombro_direto.scad): as orelhas apoiam na face de +x, o corpo passa pela janela
m_plt = [[-21, -9], [3, 15]];                                 // y das abas da chapa do servo atraves do prato (janelas fechadas)
m_plz = sh_z1 + 8;                                            // topo da chapa do servo
m_backz = [[mz_p + 5, mz_p + 15], [mz_p + 30, mz_p + 40]];       // abas do fundo atraves das torres
m_ct = [[-30, -18], [-6, 6], [18, 30], [38, 50]];              // abas das placas laterais do carrier atraves da frontal
mz_br = cr_bz0;                                              // base da ponte = a da moldura impressa (acima das engrenagens das mandibulas impressas)
mz_top = mz_br + mt;                                             // topo da ponte
m_cf0 = 14 - mt;                                             // face de tras da placa frontal do carrier (a de fora fica em y = 14, onde encosta a moldura)
m_tw = col_x - mt / 2;                                       // face interna das torres e das placas laterais (|x|)

module R(x0, x1, y0, y1) { translate([x0, y0]) square([x1 - x0, y1 - y0]); }
module Rs(x0, x1, y0, y1) { R(x0 - fit / 2, x1 + fit / 2, y0 - fit / 2, y1 + fit / 2); }   // rasgo: folga nos 4 lados
module hole(x, y, d = 3.2) { translate([x, y]) circle(d = d, $fn = 24); }

// ---------- pecas 2D ----------
module d_base() difference() {                               // prato (XY)
    R(-53, 20, -22, 22);
    for (s = [-1, 1]) Rs(s * col_x - mt / 2, s * col_x + mt / 2, -9, 9);                // abas das torres
    for (x0 = [-8, 3]) Rs(x0, x0 + 5, -9, -9 + mt);                                // abas do fundo
    for (t = m_plt) Rs(m_pl[0], m_pl[1], t[0], t[1]);                              // abas da chapa do servo do ombro (janelas fechadas)
    for (x = [-50.5, 16.5], y = [-19, 19]) hole(x, y, 3.4);                        // M3 no conves
}
module d_tower() difference() {                              // torre (YZ): coordenadas (y, z)
    union() { R(-12, 12, mz_p, mz_t); R(-9, 9, mz_b, mz_p + 0.01); }
    for (z = [z_s, z_s + h_v]) hole(0, z);                                          // pivos das barras
    for (zr = m_backz) Rs(-9, -9 + mt, zr[0], zr[1]);                       // rasgos do fundo
}
module d_back() union() {                                    // fundo entre as torres (XZ): (x, z)
    R(-m_tw, m_tw, mz_p, mz_p + 45);
    for (x0 = [-8, 3]) R(x0, x0 + 5, mz_b, mz_p + 0.01);
    for (zr = m_backz) { R(m_tw - 0.1, col_x + mt / 2, zr[0], zr[1]); R(-(col_x + mt / 2), -(m_tw - 0.1), zr[0], zr[1]); }
}
module d_servoplate() difference() {                        // chapa do MG996R (YZ): coordenadas (y, z); abas embaixo no prato, janela fechada para o corpo, 4 furos M4 das orelhas
    union() { R(sh_y0 - (sh_ear_len - sh_L) / 2 - 3, sh_y1 + (sh_ear_len - sh_L) / 2 + 3, mz_p, m_plz); for (t = m_plt) R(t[0], t[1], mz_b, mz_p + 0.01); }
    R(sh_y0 - 0.3, sh_y1 + 0.3, sh_z0 - 0.3, sh_z1 + 0.3);
    for (sy = [-1, 1], sz = [-1, 1]) hole((sh_y0 + sh_y1) / 2 + sy * sh_hole[0] / 2, z_s + sz * sh_hole[1] / 2, sh_hole_d);
}
module d_side() difference() {                               // placa lateral do carrier (YZ), origem no pivo de baixo
    union() { R(-12, m_cf0, -(carrier_below + 3), h_v + 8); for (zr = m_ct) R(m_cf0 - 0.1, 14, zr[0], zr[1]); }
    for (z = [0, h_v]) hole(0, z);
}
module d_front() difference() {                              // placa frontal do carrier (XZ)
    R(-18, 18, -(carrier_below + 3), h_v + 8);
    for (s = [-1, 1], zr = m_ct) Rs(s * col_x - mt / 2, s * col_x + mt / 2, zr[0], zr[1]);
    for (x = [-14, 14], zl = [8, 18]) hole(x, -carrier_below + zl);                 // M3 da moldura da pinca
}
// Moldura da pinca sem cola: a prateleira, a ponte e o anteparo formam um "C" (o anteparo prende a ponte por rasgos fechados e a prateleira por
// janelas fechadas); o C desliza para tras (-y) e as abas da prateleira e da ponte entram em JANELAS FECHADAS da placa traseira.
// Ai a prateleira nao sai nem para baixo, a ponte nao sai para cima e o anteparo fica preso pelos dois. Sai so deslizando o C para a frente (+y).
module d_cshelf() difference() {                             // prateleira da pinca (XY), origem na linha dos pivos
    union() {
        R(-24, 24, -24 + mt, 13 - mt);                                                                                   // corpo
        R(-31.5, -24 + 0.01, -5, 5);                                                                                     // aba sob o pivo da mandibula esquerda (suporte do M3 x 30 de baixo)
        for (s = [-1, 1]) {
            R(s > 0 ? 6 : -18, s > 0 ? 18 : -6, -24, -24 + mt + 0.01);                                                   // abas de tras: janelas da placa traseira
        }
    }
    hole(-cr_rc, 0, 3.4);                                                                                              // pivo de baixo da mandibula esquerda: M3 x 30 por baixo
}
module d_rear() difference() {                               // placa traseira da pinca (XZ) com 4 janelas fechadas
    R(-36, 36, -mt - 3, mz_top + 3);
    for (s = [-1, 1]) for (zr = [[-mt, 0], [mz_br, mz_top]]) Rs(s > 0 ? 6 : -18, s > 0 ? 18 : -6, zr[0], zr[1]);
    for (x = [-14, 14], zl = [8, 18]) hole(x, zl);                                                                          // M3 no carrier
}
m_ear = [[21.6, 24], [-24, -21.6]];                       // orelhas do anteparo: fora da zona das engrenagens (|x| < ~20 aberta) e dos bracos das mandibulas (|x| > 24.25)
module d_stop() difference() {                               // anteparo do cubo (XZ): comeca em z = 4,5 (os setores das mandibulas passam por baixo, z 0 a 4) e vai ate o topo do cubo (z = 30); so as orelhas sobem ate a ponte
    union() {
        R(-24, 24, 4.5, cr_cube - cube_grip);
        for (e = m_ear) { R(e[0], e[1], cr_cube - cube_grip - 0.01, mz_br); R(e[0], e[1], mz_br - 0.01, mz_top); }       // orelhas + abas de cima (rasgos da ponte)
    }
    hole(0, 13, 10);
}
module d_bridge() difference() {                             // ponte da pinca (XY): fica acima do cubo, entao pode passar da face do anteparo (y = 13)
    union() { R(-36, 36, -24 + mt, 13 + 4); for (s = [-1, 1]) R(s > 0 ? 6 : -18, s > 0 ? 18 : -6, -24, -24 + mt + 0.01); }
    for (e = m_ear) Rs(e[0], e[1], 13 - mt, 13);                                                                            // rasgos fechados das orelhas do anteparo
    for (s = [-1, 1]) hole(s * cr_rc, 0, 3.4);                                      // pinos M3 dos pivos
    hole(-cr_rc, 0, 8);                                                              // eixo do SG90
}
module d_bar() difference() {                                // barra do paralelogramo
    hull() for (y = [0, L1]) translate([0, y]) circle(d = bar_w, $fn = 40);
    for (y = [0, L1]) hole(0, y);
    for (y = sh_horn_y) hole(0, y, sh_horn_d);               // parafusos do horn do MG996R (iguais nas 4 barras)
}
module d_coupon() difference() {                             // cupom de espessura: 7 rasgos de 2.0 a 2.6 mm (passo 0.1); o rasgo n tem n furinhos em cima
    R(0, 100, 0, 30);
    for (i = [0 : 6]) {
        R(8 + i * 13, 8 + i * 13 + 2.0 + i * 0.1, 4, 16);
        for (k = [0 : i]) translate([8 + i * 13 + 1, 22 + (k % 4) * 1 + floor(k / 4) * 3]) circle(d = 0.8, $fn = 12);
    }
}

// lista das pecas e quantidades (usada pelo nesting)
PARTS_2D = [["base", 1], ["tower", 2], ["back", 1], ["servoplate", 1], ["side", 2], ["front", 1], ["cshelf", 1], ["rear", 1],
            ["stop", 1], ["bridge", 1], ["bar", 4]];
if (scene == "2d_base") d_base();
if (scene == "2d_tower") d_tower();
if (scene == "2d_back") d_back();
if (scene == "2d_servoplate") d_servoplate();
if (scene == "2d_side") d_side();
if (scene == "2d_front") d_front();
if (scene == "2d_cshelf") d_cshelf();
if (scene == "2d_rear") d_rear();
if (scene == "2d_stop") d_stop();
if (scene == "2d_bridge") d_bridge();
if (scene == "2d_bar") d_bar();
if (scene == "2d_coupon") d_coupon();

// ---------- montagem 3D ----------
claw_expl = 0;        // >0 afasta as pecas da moldura para a vista explodida (mm)
module XY(z0) translate([0, 0, z0]) linear_extrude(height = mt) children();
module XZ(y0) translate([0, y0 + mt, 0]) rotate([90, 0, 0]) linear_extrude(height = mt) children();
module YZ(x0) translate([x0, 0, 0]) rotate([90, 0, 90]) linear_extrude(height = mt) children();

module mdf_mount() {
    color("#c8a165") {
        XY(mz_b) d_base();
        for (s = [-1, 1]) YZ(s * col_x - mt / 2) d_tower();
        XZ(-9) d_back();
        YZ(m_pl[0]) d_servoplate();
    }
}
module mdf_claw(psi) {                                       // origem: linha dos pivos, z = 0 no fundo das mandibulas
    color("#c8a165") {
        translate([0, claw_expl * 0.6, -claw_expl]) XY(-mt) d_cshelf();
        translate([0, -claw_expl, 0]) XZ(-24) d_rear();
        translate([0, claw_expl * 1.4, 0]) XZ(13 - mt) d_stop();
        translate([0, claw_expl * 0.6, claw_expl]) XY(mz_br) d_bridge();
    }
    color("khaki") cr_jaws(psi);                                  // mandibulas IMPRESSAS (claw_rot.scad)
}
module mdf_carrier_claw() {
    color("#c8a165") {
        for (s = [-1, 1]) translate([0, u_arm, zl_arm]) YZ(s * col_x - mt / 2) d_side();
        translate([0, u_arm, zl_arm]) XZ(m_cf0) d_front();
    }
    color("silver") for (z = [0, h_v]) translate([-bar_x - 6, u_arm, zl_arm + z]) rotate([0, 90, 0]) cylinder(d = 3, h = 2 * bar_x + 12);
    translate([0, u_arm + carrier_front + cr_rear, zl_arm - carrier_below]) mdf_claw(open ? cr_psi_open : 0);
}

module arm() {                                               // redefine o arm() de braco.scad: o resto da cena (chassi, cubo) e reaproveitado
    translate([0, pivot_y, 0]) { mdf_mount(); bars(); mdf_carrier_claw(); }
}

if (scene == "claw_only") mdf_claw(open ? cr_psi_open : 0);

// checagem: partes MDF da moldura contra as mandibulas impressas (vazio ou so contato plano = ok)
if (scene == "chk_claw_jaws") intersection() {
    union() { XY(-mt) d_cshelf(); XZ(-24) d_rear(); XZ(13 - mt) d_stop(); XY(mz_br) d_bridge(); }
    cr_jaws(psi_chk);
}

// ---------- checagens do ombro com MG996R direto (resultado vazio = sem colisao; bbox plano = contato de face) ----------
module sh_moving() { sh_servo_body(); sh_horn(alpha); }
if (scene == "chk_sh_mount") translate([0, pivot_y, 0]) intersection() { sh_moving(); mdf_mount(); }
if (scene == "chk_sh_chassis") intersection() { translate([0, pivot_y, 0]) sh_moving(); robot_base(true); }
if (scene == "chk_sh_arm") translate([0, pivot_y, 0]) intersection() { union() { sh_servo_body(); sh_horn(alpha); } union() { for (x = [-bar_x, bar_x]) { bar([0, z_s], [u_arm, zl_arm], x); bar([0, z_s + h_v], [u_arm, zl_arm + h_v], x); } mdf_carrier_claw(); } }

// ---------- checagem do acoplador do SG90 na mandibula esquerda (vazio ou so contato plano = ok) ----------
module cr_coupler_world(psi) mirror([1, 0, 0]) translate([cr_rc, 0, 0]) rotate([0, 0, -psi]) translate([0, 0, cr_gz0 + cr_gt]) cr_coupler_local();
module cr_left_jaw_plain(psi) cr_jaw_left(psi, false);
if (scene == "chk_cp_frame") intersection() {
    cr_coupler_world(psi_chk);
    union() { XY(-mt) d_cshelf(); XZ(-24) d_rear(); XZ(13 - mt) d_stop(); XY(mz_br) d_bridge(); }
}
if (scene == "chk_cp_jaws") intersection() { cr_coupler_world(psi_chk); union() { cr_left_jaw_plain(psi_chk); cr_jaw_right(psi_chk, 0); } }
if (scene == "claw_cp") { mdf_claw(psi_chk); }

// ---------- checagem do conjunto do ombro (chapa do servo + MG996R) contra o chassi inteiro ----------
module sh_whole() translate([0, pivot_y, 0]) { mdf_mount(); sh_moving(); }
if (scene == "chk_sh_whole") intersection() { sh_whole(); robot_base(true); }
