// Ombro com servo pequeno (SG90/MG90S) e reducao 4:1: pinhao z12 no servo + setor z48 preso a barra de baixo (lado -x), mesmo modulo m1,25.
// O servo fica ATRAS das torres (a sr_D do pivo, na direcao -y) porque o torque do ombro (4,2 kg.cm com folga 2x) e ~2x o de um SG90/MG90S direto.
// Reusa cr_tooth / cr_sector_2d / cr_spline_pocket de claw_rot.scad (mesmo dente da pinca, mesmo bolso estriado de 21 dentes).
// Eixo do servo = x, apontando para +x; pivo das barras em (y = 0, z = z_s). Pecas locais usam (u, v, w) = (y, z, x) via rotate([90,0,90]).
sr_m = 1.25; sr_zp = 12; sr_zs = 48; sr_bl = 0.15;
sr_D = sr_m * (sr_zp + sr_zs) / 2;                  // 37,5 mm entre o pivo e o eixo do servo
sr_ratio = sr_zs / sr_zp;                           // 4:1
sr_t = 3.5;                                         // espessura das engrenagens
sr_bar_face = bar_x + 1.15;                         // face externa da barra de MDF de 2,3 mm (centro em |x| = bar_x)
sr_x0 = -(sr_bar_face + sr_t);                      // -19,65: face de tras das engrenagens (lado do servo)
sr_th = 165;                                        // direcao do pivo ao eixo do servo no plano y-z (0 = +y, 180 = -y). Um pouco acima de -y: com 180 a ponta baixa do setor bate no prato quando a barra sobe
sr_py = sr_D * cos(sr_th); sr_pz = sr_D * sin(sr_th);   // posicao do eixo do servo em (y, z - z_s)
sr_arc = [sr_th - 35, sr_th + 25];                  // faixa de dentes do setor, no referencial da barra: cobre o contato em [th - 25,5; th + 16] (alpha de +25,5 a -16) com 1 dente de folga
sr_pad_r = 15; sr_peg_y = [6, 12];                  // calco do setor e pinos (entram em furos de 3,2 mm da barra, a 6 e 12 mm do pivo)
sr_hub_d = 12; sr_hub_len = 7.5;                    // cubo do pinhao: bolso estriado de cr_sp_depth no lado do servo, M2 do horn por fora
// SG90 (medidas de catalogo, MEDIR): corpo 22,5 x 12,2 x 22,7, eixo a 5,5 mm de uma ponta, orelhas 2,5 mm a 6,2 mm abaixo do topo, 5 mm alem de cada ponta
sr_sv_L = 22.5; sr_sv_W = 12.2; sr_sv_H = 22.7; sr_sv_off = 5.5; sr_sv_top = sr_x0 - 0.5;
sr_sv_y0 = sr_py - (sr_sv_L - sr_sv_off); sr_sv_y1 = sr_py + sr_sv_off;        // extensao do corpo em y (rumo a -y, longe das torres)
sr_sv_z0 = z_s + sr_pz - sr_sv_W / 2;               // fundo do corpo = topo da prateleira do berco

module sr_gear2d(z) {
    circle(r = sr_m * z / 2 - 1.25 * sr_m, $fn = 64);
    for (i = [0 : z - 1]) rotate(i * 360 / z) cr_tooth(z, sr_m, sr_bl);
}

module sr_sector_local() difference() {             // w = 0: face de tras (dentes na mesa); pinos para cima, atravessam a barra
    union() {
        linear_extrude(sr_t) { cr_sector_2d(sr_zs, sr_m, sr_bl, sr_arc[0], sr_arc[1]); circle(r = sr_pad_r, $fn = 64); }
        for (y = sr_peg_y) translate([y, 0, sr_t - 0.01]) cylinder(d = 3.2, h = 2.31, $fn = 24);
    }
    translate([0, 0, -1]) cylinder(d = 3.2, h = sr_t + 2, $fn = 24);                      // parafuso do pivo
}

module sr_pinion_local() difference() {             // w = 0: face contra o servo (aberta, bolso estriado); dentes em w = 0 .. sr_t
    union() {
        linear_extrude(sr_t) sr_gear2d(sr_zp);
        cylinder(d = sr_hub_d, h = sr_hub_len, $fn = 48);
    }
    translate([0, 0, -1]) cr_spline_pocket(cr_sp_depth + 1);                               // bolso do eixo (21 dentes)
    translate([0, 0, cr_sp_depth - 0.5]) cylinder(d = 2.2, h = sr_hub_len, $fn = 24);      // M2 do horn
    translate([0, 0, sr_hub_len - 1.2]) cylinder(d = 4.2, h = 2, $fn = 24);                // rebaixo da cabeca
}

// alpha = angulo da barra; o pinhao gira -4 x alpha; fase: a alpha = 0 o dente do setor (em sr_th) entra no vao do pinhao (que olha para sr_th + 180)
module sr_sector(alpha) translate([sr_x0, 0, z_s]) rotate([alpha, 0, 0]) rotate([90, 0, 90]) sr_sector_local();
sr_phase = sr_th + 180 - 180 / sr_zp;
module sr_pinion(alpha) translate([sr_x0, sr_py, z_s + sr_pz]) rotate([-sr_ratio * alpha + sr_phase, 0, 0]) rotate([90, 0, 90]) sr_pinion_local();
module sr_servo_dummy() {                           // so ilustracao/colisao
    color("royalblue") {
        box(sr_sv_top - sr_sv_H, sr_sv_top, sr_sv_y0, sr_sv_y1, sr_sv_z0, sr_sv_z0 + sr_sv_W);
        box(sr_sv_top - 6.2 - 2.5, sr_sv_top - 6.2, sr_sv_y0 - 5, sr_sv_y1 + 5, sr_sv_z0, sr_sv_z0 + sr_sv_W);   // orelhas
    }
}
module sr_drive(alpha) {
    color("orange") { sr_sector(alpha); sr_pinion(alpha); }
    sr_servo_dummy();
}
