// Versao em MDF encaixavel (corte a laser/CNC 2D) do braco da opcao 2. Mesma geometria de braco.scad, pecas planas com juntas tab-slot.
// Uso 3D:  openscad -o mdf_arm.png --camera=330,330,260,0,110,70 -D 'pose="pick"' braco_mdf.scad
// Uso 2D:  openscad -o peca.svg -D 'scene="2d_tower"' braco_mdf.scad   (nomes em PARTS_2D; mdf_nest.py gera as chapas)
// Todas as pecas 2D usam coordenadas naturais do plano em que ficam (XY, XZ ou YZ do braco); mdf_nest.py reposiciona ao cortar.
include <braco.scad>

mt = 2.3;             // espessura do MDF (cupom: rasgo nominal de 2,4 mm ficou levemente solto; material ~2,3 mm)
fit = 0;              // folga nominal dos rasgos: 0, porque o kerf do laser ja abre ~0,1-0,2 mm (acertado pelo cupom)

// ---------- derivados ----------
mz_b = z_deck; mz_p = mz_b + mt; mz_t = z_s + h_v + 8;               // prato do suporte: embaixo, em cima; topo das torres
mf_clr = 0.3;                                                // folga da janela do corpo do servo, cada lado
mf_web = 3;                                                  // alma de MDF entre a janela do corpo e o rasgo do tampo
mf_pad = 8;                                                  // margem do quadro alem do corpo, em y
m_fx = [sts_top - mt, sts_x0];                               // x dos 2 quadros do servo: um rente a face do disco, outro rente a face de tras (o corpo passa pelas duas janelas)
m_shz = sts_z1 + mf_clr + mf_web;                            // z de baixo do tampo que une os quadros (acima do corpo)
m_ftop = m_shz + mt + mf_web;                                // topo dos quadros
m_ft = [[sts_y0 - 4, sts_y0 + 8], [sts_y1 - 8, sts_y1 + 4]];   // y dos 2 pes de cada quadro atraves do prato (janelas fechadas), o mais afastados possivel
m_tfeet = [[-20, -12], [-4, 4], [12, 20]];                  // y dos 3 pes de cada torre atraves do prato (janelas fechadas)
m_tfl = [20, 30];                                            // base alargada da torre: meia largura no prato e altura em que volta a +-12
m_ttz = mz_t - mt - 1.5;                                     // z de baixo do tampo das torres (1,5 mm abaixo do topo, acima do parafuso do pivo de cima)
m_br = [-15.5, -15.5 + mt];                                  // y da escora (atras do pivo alargado da barra, que chega a y = -11; dentro do trapezio da torre)
m_brz = [mz_p + 2, mz_p + 8];                                // z da escora: abaixo do corpo do servo (a janela comeca em z0 - 0,3)
m_bext = -60;                                                // fim da extensao do prato para tras (os furos do conves em y = -56 ficam dentro)
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
module d_base() difference() {                               // prato (XY), com a extensao para tras que leva o berco do servo do ombro
    union() { R(-53, 20, -22, 22); R(-53, 20, m_bext, -22 + 0.01); }
    for (s = [-1, 1], f = m_tfeet) Rs(s * col_x - mt / 2, s * col_x + mt / 2, f[0], f[1]);   // pes das torres
    for (x0 = [-8, 3]) Rs(x0, x0 + 5, -9, -9 + mt);                                // abas do fundo
    for (fx = m_fx, f = m_ft) Rs(fx, fx + mt, f[0], f[1]);                            // pes dos 2 quadros do servo (janelas fechadas)
    for (x = [-50.5, -16.5, 16.5], y = [-19, 19]) hole(x, y, 3.4);                 // M3 no conves: cantos + um ao lado de cada torre (x = -16,5 e 16,5)
    for (x = [-50.5, -16.5, 16.5]) hole(x, -56, 3.4);                              // M3 no conves, extensao (furar o convés de cima ai): 3 na fileira de tras
}
module d_tower() difference() {                              // torre (YZ): coordenadas (y, z); base alargada (trapezio) com 3 pes
    union() {
        R(-12, 12, mz_p, mz_t);
        polygon([[-m_tfl[0], mz_p], [m_tfl[0], mz_p], [12, mz_p + m_tfl[1]], [-12, mz_p + m_tfl[1]]]);
        for (f = m_tfeet) R(f[0], f[1], mz_b, mz_p + 0.01);
    }
    for (z = [z_s, z_s + h_v]) hole(0, z);                                          // pivos das barras
    for (zr = m_backz) Rs(-9, -9 + mt, zr[0], zr[1]);                       // rasgos do fundo
    Rs(-9, 9, m_ttz, m_ttz + mt);                                                   // rasgo do tampo
    Rs(m_br[0], m_br[1], m_brz[0], m_brz[1]);                                       // rasgo da escora do servo
}
module d_back() union() {                                    // fundo entre as torres (XZ): (x, z)
    R(-m_tw, m_tw, mz_p, mz_p + 45);
    for (x0 = [-8, 3]) R(x0, x0 + 5, mz_b, mz_p + 0.01);
    for (zr = m_backz) { R(m_tw - 0.1, col_x + mt / 2, zr[0], zr[1]); R(-(col_x + mt / 2), -(m_tw - 0.1), zr[0], zr[1]); }
}
module d_stsframe() difference() {                           // quadro do STS3215 (YZ): janela fechada no corpo, rasgo fechado para o tampo, pes no prato. Sao 2 iguais
    union() { R(sts_y0 - mf_pad, sts_y1 + mf_pad, mz_p, m_ftop); for (f = m_ft) R(f[0], f[1], mz_b, mz_p + 0.01); }
    R(sts_y0 - mf_clr, sts_y1 + mf_clr, sts_z0 - mf_clr, sts_z1 + mf_clr);
    Rs(sts_y0, sts_y1, m_shz, m_shz + mt);
    Rs(m_br[0], m_br[1], m_brz[0], m_brz[1]);                              // rasgo da escora
    for (y = [sts_y0 - mf_pad / 2, sts_y1 + mf_pad / 2]) hole(y, z_s, 3.4);   // 2 M3 de ligacao entre os quadros: espacador M3 femea-femea de 30 mm entre eles (corpo 35 - 2 x 2,3 de MDF = 30,4)
}
module d_stsshelf() difference() {                           // tampo (XY): une os 2 quadros por cima, as pontas passam pelos rasgos
    R(sts_x0, sts_top, sts_y0, sts_y1);
}
module d_towertop() R(-(col_x + mt / 2), col_x + mt / 2, -9, 9);   // tampo das torres (XY): liga as duas no topo, pontas nos rasgos das torres
module d_stsbrace() R(sts_x0, -(col_x - mt / 2), m_brz[0], m_brz[1]);   // escora (XZ): do quadro de tras do servo, pelo da frente, ate a torre -x; passa por rasgos fechados nos 3
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
module d_bar(sts = false) difference() {                     // barra do paralelogramo; sts = barra do ombro presa direto no disco do STS3215 (sem reducao)
    union() {
        hull() for (y = [0, L1]) translate([0, y]) circle(d = bar_w, $fn = 40);
        if (sts) circle(d = sts_disc_d + 2, $fn = 64);       // pivo alargado: o circulo de furos (14) nao cabe nos 12 da barra
    }
    for (y = [0, L1]) hole(0, y, sts && y == 0 ? sts_center_d : 3.2);
    if (sts) for (a = [0 : 90 : 270]) rotate(a) hole(sts_hole_r, 0, sts_screw_d);   // 4 x M3 direto no disco do servo
}
module d_coupon() difference() {                             // cupom de espessura: 7 rasgos de 2.0 a 2.6 mm (passo 0.1); o rasgo n tem n furinhos em cima
    R(0, 100, 0, 30);
    for (i = [0 : 6]) {
        R(8 + i * 13, 8 + i * 13 + 2.0 + i * 0.1, 4, 16);
        for (k = [0 : i]) translate([8 + i * 13 + 1, 22 + (k % 4) * 1 + floor(k / 4) * 3]) circle(d = 0.8, $fn = 12);
    }
}

// lista das pecas e quantidades (usada pelo nesting)
PARTS_2D = [["base", 1], ["tower", 2], ["back", 1], ["stsframe", 2], ["stsshelf", 1], ["towertop", 1], ["stsbrace", 1], ["side", 2], ["front", 1], ["cshelf", 1], ["rear", 1],
            ["stop", 1], ["bridge", 1], ["bar", 3], ["bar_sts", 1]];
if (scene == "2d_base") d_base();
if (scene == "2d_tower") d_tower();
if (scene == "2d_back") d_back();
if (scene == "2d_stsframe") d_stsframe();
if (scene == "2d_stsshelf") d_stsshelf();
if (scene == "2d_towertop") d_towertop();
if (scene == "2d_stsbrace") d_stsbrace();
if (scene == "2d_side") d_side();
if (scene == "2d_front") d_front();
if (scene == "2d_cshelf") d_cshelf();
if (scene == "2d_rear") d_rear();
if (scene == "2d_stop") d_stop();
if (scene == "2d_bridge") d_bridge();
if (scene == "2d_bar") d_bar();
if (scene == "2d_bar_sts") d_bar(true);
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
        for (fx = m_fx) YZ(fx) d_stsframe();
        XY(m_shz) d_stsshelf();
        XY(m_ttz) d_towertop();
        XZ(m_br[0]) d_stsbrace();
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

// ---------- checagens do ombro com STS3215 direto (resultado vazio = sem colisao) ----------
module sts_whole() translate([0, pivot_y, 0]) { mdf_mount(); sts_servo_dummy(); }
if (scene == "chk_sts_chassis") intersection() { translate([0, pivot_y, 0]) { sts_servo_dummy(); mdf_mount(); } robot_base(true); }   // so contato plano em z = mz_b (pes no prato) e ok
if (scene == "chk_sts_bars") translate([0, pivot_y, 0]) intersection() { bars_only(); union() { mdf_mount(); sts_servo_dummy(); } }

// ---------- checagem do acoplador do SG90 na mandibula esquerda (vazio ou so contato plano = ok) ----------
module cr_coupler_world(psi) mirror([1, 0, 0]) translate([cr_rc, 0, 0]) rotate([0, 0, -psi]) translate([0, 0, cr_gz0 + cr_gt]) cr_coupler_local();
module cr_left_jaw_plain(psi) cr_jaw_left(psi, false);
if (scene == "chk_cp_frame") intersection() {
    cr_coupler_world(psi_chk);
    union() { XY(-mt) d_cshelf(); XZ(-24) d_rear(); XZ(13 - mt) d_stop(); XY(mz_br) d_bridge(); }
}
if (scene == "chk_cp_jaws") intersection() { cr_coupler_world(psi_chk); union() { cr_left_jaw_plain(psi_chk); cr_jaw_right(psi_chk, 0); } }
if (scene == "claw_cp") { mdf_claw(psi_chk); }
