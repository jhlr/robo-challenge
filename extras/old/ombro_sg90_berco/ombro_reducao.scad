// Ombro com servo pequeno (SG90/MG90S) e reducao 4:1: pinhao z12 no servo + setor z48 preso a barra de baixo (lado -x), mesmo modulo m1,25.
// O servo fica ATRAS das torres (a sr_D do pivo, na direcao -y) porque o torque do ombro (4,2 kg.cm com folga 2x) e ~2x o de um SG90/MG90S direto.
// Reusa cr_tooth / cr_sector_2d de claw_rot.scad (mesmo dente da pinca). O pinhao NAO tem bolso estriado: parafusa no horn redondo do servo.
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
// Pinhao PARAFUSADO no horn redondo de 20 mm do servo (disco com coroa de furinhos): flange do pinhao encosta no horn, 3 pinos entram na coroa, M2 do eixo prende tudo.
// MEDIR no horn real: espessura, raio da coroa, numero e diametro dos furos, altura do cubo central. Valores abaixo vem de foto (+-0,5 mm).
sr_horn_d = 20; sr_horn_t = 5; sr_horn_disc_t = 2;   // MEDIDO: disco de 20 mm x 2 mm; cubo central de 3 mm virado para o SERVO (total 5 mm); a face do disco virada para o pinhao e plana
sr_hz_n = 12; sr_hz_r = 8; sr_hz_d = 1.3;           // MEDIDO: furos laterais a 8 mm do centro (medianos a 7 mm). Quantidade e diametro dos furos ainda supostos
sr_peg_n = 4; sr_peg_d = 1.1; sr_peg_l = 2.3;       // 4 pinos impressos (1 a cada sr_hz_n / 4 furos: so 4 furos do horn sao usados): 0,5 de folga ate o horn + 1,8 dentro do disco de 2 mm
sr_fl_d = 22; sr_fl_t = 2;                          // flange do pinhao (atras dos dentes, do lado do servo)
sr_hub_d = 8;                                      // diametro do cubo do horn (suposto, MEDIR); so aparece no dummy do servo
// SG90 (medidas de catalogo, MEDIR): corpo 22,5 x 12,2 x 22,7, eixo a 5,5 mm de uma ponta, orelhas 2,5 mm a 6,2 mm abaixo do topo, 5 mm alem de cada ponta
sr_sv_L = 22.5; sr_sv_W = 12.2; sr_sv_H = 22.7; sr_sv_off = 5.5; sr_sv_top = sr_x0 - sr_fl_t - sr_horn_t - 0.5;   // servo recuado: flange + horn entre ele e as engrenagens
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

module sr_pinion_local() difference() {             // w = 0: plano dos dentes (face do servo); w de -sr_fl_t a 0 = flange, pinos para -w (entram no horn)
    union() {
        linear_extrude(sr_t) sr_gear2d(sr_zp);
        translate([0, 0, -sr_fl_t]) cylinder(d = sr_fl_d, h = sr_fl_t + 0.01, $fn = 64);
        for (k = [0 : sr_peg_n - 1]) rotate(k * 360 / sr_peg_n) translate([sr_hz_r, 0, -sr_fl_t - sr_peg_l]) cylinder(d = sr_peg_d, h = sr_peg_l + 0.01, $fn = 16);
    }
    translate([0, 0, -sr_fl_t - 1]) cylinder(d = 2.2, h = sr_fl_t + sr_t + 2, $fn = 24);        // M2 do eixo
    translate([0, 0, sr_t - 1.5]) cylinder(d = 4.2, h = 2, $fn = 24);                           // rebaixo da cabeca
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
    color("black") translate([sr_sv_top, sr_py, z_s + sr_pz]) rotate([90, 0, 90]) { cylinder(d = sr_hub_d, h = sr_horn_t - sr_horn_disc_t, $fn = 32); translate([0, 0, sr_horn_t - sr_horn_disc_t]) cylinder(d = sr_horn_d, h = sr_horn_disc_t, $fn = 48); }   // horn: cubo de 3 mm sobre o servo + disco de 20 x 2 mm, face plana para o pinhao
}
module sr_drive(alpha) {
    color("orange") { sr_sector(alpha); sr_pinion(alpha); }
    sr_servo_dummy();
}
