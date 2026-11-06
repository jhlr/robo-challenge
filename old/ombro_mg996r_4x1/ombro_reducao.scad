// Ombro com servo MG996R (TowerPro) e reducao 4:1 (antes SG90/MG90S: old/ombro_sg90_berco): pinhao z12 no servo + setor z48 preso a barra de baixo (lado -x), mesmo modulo m1,25.
// O servo fica ATRAS das torres (a sr_D do pivo, na direcao -y) porque o torque do ombro (4,2 kg.cm com folga 2x) e ~2x o de um SG90/MG90S direto.
// Reusa cr_tooth / cr_sector_2d de claw_rot.scad (mesmo dente da pinca). O pinhao NAO tem bolso estriado: parafusa no horn redondo do servo.
// Eixo do servo = x, apontando para -x (servo atras das torres, dentro da largura do prato); pivo das barras em (y = 0, z = z_s). Pecas locais usam (u, v, w) = (y, z, x) via rotate([90,0,90]).
sr_m = 1.25; sr_zp = 12; sr_zs = 48; sr_bl = 0.15;
sr_D = sr_m * (sr_zp + sr_zs) / 2;                  // 37,5 mm entre o pivo e o eixo do servo
sr_ratio = sr_zs / sr_zp;                           // 4:1
sr_t = 3.5;                                         // espessura das engrenagens
sr_bar_face = bar_x + 1.15;                         // face externa da barra de MDF de 2,3 mm (centro em |x| = bar_x)
sr_x0 = -(sr_bar_face + sr_t);                      // -19,65: face de fora das engrenagens (plano do setor e do pinhao: x de sr_x0 a sr_x0 + sr_t)
sr_th = 180;                                        // direcao do pivo ao eixo do servo no plano y-z (0 = +y, 180 = -y). Um pouco acima de -y: com 180 a ponta baixa do setor bate no prato quando a barra sobe
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
// MG996R (TowerPro, medidas de catalogo, MEDIR): corpo 40,7 x 19,7, altura 42,9 ate a ponta do eixo (eixo ~4,5 mm acima da tampa), eixo a 10,3 mm de uma ponta,
// orelhas de 2,5 mm a 14 mm abaixo da ponta do eixo, 54,4 mm de ponta a ponta, furos M4 a 49,5 x 10 mm. O horn redondo de 20 mm assenta na tampa (sr_sv_top).
sr_sv_L = 40.7; sr_sv_W = 19.7; sr_sv_H = 38.4; sr_sv_off = 10.3; sr_sv_top = sr_x0 + sr_t + sr_fl_t + sr_horn_t + 0.5;   // sr_sv_H: altura sem o eixo (42,9 - 4,5); servo INVERTIDO: fica do lado de dentro (x maior que a tampa), atras das torres, com o eixo apontando para fora (-x); flange + horn entre a tampa e as engrenagens
sr_sv_spl = 4.5; sr_ear_below = 14; sr_ear_t = 2.5; sr_ear_len = 54.4; sr_ear_hole = [49.5, 10]; sr_ear_hole_d = 4.2;
sr_ear_x = sr_sv_top + sr_ear_below - sr_sv_spl;     // face das orelhas mais perto da tampa; a chapa de MDF fica do outro lado delas (x maior)
sr_sv_y0 = sr_py - (sr_sv_L - sr_sv_off); sr_sv_y1 = sr_py + sr_sv_off;        // extensao do corpo em y (rumo a -y, longe das torres)
sr_sv_z0 = z_s + sr_pz - sr_sv_W / 2;               // fundo do corpo

module sr_gear2d(z, bl = sr_bl, ext = 0) {
    circle(r = sr_m * z / 2 - 1.25 * sr_m, $fn = 64);
    for (i = [0 : z - 1]) rotate(i * 360 / z) cr_tooth(z, sr_m, bl, ext);
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
module sr_pinion(alpha) translate([sr_x0 + sr_t, sr_py, z_s + sr_pz]) rotate([-sr_ratio * alpha + sr_phase, 0, 0]) mirror([1, 0, 0]) rotate([90, 0, 90]) sr_pinion_local();   // dentes no plano do setor (x de sr_x0 a sr_x0 + sr_t); flange e pinos do lado de dentro (+x): a peca impressa e a mesma, so montada virada
module sr_servo_dummy() {                           // so ilustracao/colisao
    color("royalblue") {
        box(sr_sv_top, sr_sv_top + sr_sv_H, sr_sv_y0, sr_sv_y1, sr_sv_z0, sr_sv_z0 + sr_sv_W);
        box(sr_ear_x, sr_ear_x + sr_ear_t, sr_sv_y0 - (sr_ear_len - sr_sv_L) / 2, sr_sv_y1 + (sr_ear_len - sr_sv_L) / 2, sr_sv_z0, sr_sv_z0 + sr_sv_W);   // orelhas
    }
    color("black") translate([sr_sv_top, sr_py, z_s + sr_pz]) mirror([1, 0, 0]) rotate([90, 0, 90]) { cylinder(d = sr_hub_d, h = sr_horn_t - sr_horn_disc_t, $fn = 32); translate([0, 0, sr_horn_t - sr_horn_disc_t]) cylinder(d = sr_horn_d, h = sr_horn_disc_t, $fn = 48); }   // horn: cubo de 3 mm sobre o servo + disco de 20 x 2 mm, face plana para o pinhao
}
module sr_drive(alpha) {
    color("orange") { sr_sector(alpha); sr_pinion(alpha); }
    sr_servo_dummy();
}

// Pinhao ESTRIADO direto no eixo do MG996R (sem horn), a partir da engrenagem_trilho do usuario: disco de base 24 + engrenagem + flange 20 formam um trilho (canal) onde corre o setor;
// cubo de 10 mm com bolso estriado de 25 dentes. Mesmos dentes do pinhao z12 (sr_gear2d, modulo 1,25), entao engrena com o setor z48 a 37,5 mm entre eixos.
// MEDIR no eixo real (paquimetro): diametro externo e de raiz da estria. Valores abaixo sao de catalogo (MG996R: 25 dentes, ~5,9 mm).
sp_n = 25; sp_major = 5.9; sp_minor = 5.3; sp_clr = -0.4;          // furo 1 mm menor em diametro que a versao frouxa (clr +0,1 / deep 0,2: 6,5 e 5,5 mm): 5,5 mm nas pontas e 4,5 mm na raiz, MENOR que o eixo (5,9): entra a forca, raspar/lixar
sp_deep = 0.2; sp_hw = [0.20, 0.15];                              // entalhes da estria: +0,2 mm no raio, meia largura na raiz e na ponta
sp_ext = 1.0;                                                     // dentes do pinhao alongados 1 mm (pedido do usuario): ponta a 9,75 mm do centro; a raiz do setor z48 fica a 9,06 mm, entao a ponta invade ~0,69 mm: raspar no setor ao montar (sp_ext = 0,2 e o maximo sem interferencia)
sp_bl = 0.05;                                                     // folga lateral do pinhao (antes sr_bl = 0,15): dentes mais gordos; o setor continua com sr_bl, folga total entre dentes 0,2 mm          // estria do MG996R + folga de impressao por lado
sp_depth = 3;                                                     // bolso com 3 mm de fundo (cubo de 4 mm, 1 mm de teto); o eixo sai ~4,5 mm da tampa, entao a estria engata so 3 mm
sp_base_d = 24; sp_base_t = 1.5; sp_ch_h = 7; sp_fl_d = 20; sp_fl_t = 2; sp_hub_d = 10; sp_hub_h = 4;   // canal de 7 mm: o setor (3,5 mm) corre solto entre o disco e o flange
module sp_spline2d(clr = sp_clr) {
    rm = sp_major / 2 + clr + sp_deep; rr = sp_minor / 2 + clr;
    union() {
        circle(r = rr, $fn = 64);
        for (i = [0 : sp_n - 1]) rotate(i * 360 / sp_n) polygon([[rr - 0.05, -sp_hw[0]], [rm, -sp_hw[1]], [rm, sp_hw[1]], [rr - 0.05, sp_hw[0]]]);
    }
}
module sr_pinion_spline_local() difference() {                    // z = 0: face do disco de base (na mesa); cubo e bolso estriado para cima
    union() {
        cylinder(d = sp_base_d, h = sp_base_t, $fn = 96);
        translate([0, 0, sp_base_t]) linear_extrude(sp_ch_h) sr_gear2d(sr_zp, sp_bl, sp_ext);
        translate([0, 0, sp_base_t + sp_ch_h]) cylinder(d = sp_fl_d, h = sp_fl_t, $fn = 96);
        translate([0, 0, sp_base_t + sp_ch_h + sp_fl_t]) cylinder(d = sp_hub_d, h = sp_hub_h, $fn = 96);
    }
    translate([0, 0, sp_base_t + sp_ch_h + sp_fl_t + sp_hub_h - sp_depth]) linear_extrude(sp_depth + 1) sp_spline2d();   // bolso da estria
    translate([0, 0, -1]) cylinder(d = 3.4, h = 40, $fn = 32);                                                           // M3 do eixo
    translate([0, 0, -1]) cylinder(d = 6.4, h = 1 + 3, $fn = 32);                                                        // rebaixo da cabeca do M3 (pela base)
}

// Peca de teste do encaixe: 5 cubos com bolso estriado de folgas diferentes (so o cubo, imprime em minutos). O numero de marcas na borda = folga da lista.
sp_test_clr = [-0.10, -0.05, 0.0, 0.05, 0.10];
module sr_spline_test() {
    for (k = [0 : len(sp_test_clr) - 1]) translate([k * 14, 0, 0]) difference() {
        union() {
            cylinder(d = sp_hub_d + 2, h = sp_depth + 2, $fn = 64);
            for (i = [0 : k]) rotate(i * 25) translate([sp_hub_d / 2 + 1, 0, 0]) cylinder(d = 1.2, h = sp_depth + 2, $fn = 12);   // k + 1 marcas
        }
        translate([0, 0, 2]) linear_extrude(sp_depth + 1) sp_spline2d(sp_test_clr[k]);
    }
}
