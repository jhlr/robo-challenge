// Engrenagem (setor) da garra que encaixa no disco de metal do STS3215. Mesmo dente da pinca (cr_tooth/cr_sector_2d de ../braco/claw_rot.scad).
// Disco do servo (medido 2026-11-06): 20 mm, 4 furos de 3 mm em cruz num circulo de 14 mm, parafuso M3 no centro. Ver docs/2026-11-06_sts3215-disco.md.
// Encaixe: 4 pinos nos furos do disco + flange que cobre o disco; furo central largo deixa o parafuso do eixo acessivel.
// Frame: origem no eixo do servo; z = 0 face do disco (lado do servo), dentes de z = 0 a g_t. Gira com o servo; os pinos saem para -z.
use <../braco/claw_rot.scad>
include <../braco/sts3215.scad>

g_m = 1.5; g_z = 35;               // modulo e dentes (iguais ao setor da pinca: raio primitivo 26,25)
g_a0 = -60; g_a1 = 60;             // setor, graus (multiplos de 5)
g_t = 4;                           // espessura dos dentes
g_bl = 0.1;                        // folga lateral do dente (como na pinca; ajustar na impressora)
g_fl_d = sts_disc_d + 2; g_fl_t = 1.5;         // flange atras dos dentes (cobre o disco de 20)
g_pin_r = sts_hole_r; g_pin_d = sts_hole_d - 0.2; g_pin_l = 2.5;   // pinos: folga 0,2 no furo, comprimento (profundidade do furo: MEDIR)
g_scr_d = 7;                       // furo central: passa a cabeca do parafuso M3 do eixo

module g_gear() difference() {
    union() {
        translate([0, 0, 0]) linear_extrude(g_t) union() { cr_sector_2d(g_z, g_m, g_bl, g_a0, g_a1); circle(d = g_fl_d + 2, $fn = 64); }
        translate([0, 0, -g_fl_t]) cylinder(d = g_fl_d, h = g_fl_t + 0.01, $fn = 64);
        for (a = [0 : 90 : 270]) rotate(a) translate([g_pin_r, 0, -g_fl_t - g_pin_l]) cylinder(d = g_pin_d, h = g_pin_l + 0.02, $fn = 24);
    }
    translate([0, 0, -g_fl_t - 1]) cylinder(d = g_scr_d, h = g_t + g_fl_t + 2, $fn = 40);
}
g_gear();
