// Pinca de mandibulas rotativas (opcao 2). Parametros: params_gen.scad (cr_*), gerado de params.json.
// Duas mandibulas giram em torno de pivos verticais e fecham em angulo sobre o cubo; setores de engrenagem iguais
// acoplam as duas, e o SG90 (de cabeca para baixo, acima da ponte) aciona uma delas por um acoplador impresso a parte (cr_coupler_local). Um anteparo segura a face de tras do cubo.
// Frame local: origem no meio da linha dos pivos (x=0, y=0); z = 0 no fundo das mandibulas (o cubo vai de z = -20 a 30).

cr_rc = cr_gm * cr_gz / 2;                     // pivo -> centro (raio primitivo)

// MONTAGEM DAS MANDIBULAS (conferida na foto do usuario): como saem da impressao, com a engrenagem EMBAIXO, apoiada na prateleira (z = 0 a 4); a barra e o arco
// ficam em z = 5 a 35 e o topo do boss e da barra e plano em z = 35, 1 mm sob a ponte. O setor, de raio 27,75, passaria para dentro do cubo (y > 13, z 0 a 4) e
// do anteparo (y 10,7 a 13): por isso os dentes do lado do cubo sao CORTADOS (cr_trim_y) e o anteparo comeca em z = 4,5 (braco_mdf.scad).
cr_trim_y = 12.8;                              // nenhum ponto do setor passa de y = 12,8 (face do cubo em 13) em psi de -1 a 20 graus
module cr_trim_2d() union() for (p = [-1 : 1 : 20]) rotate(p) translate([-200, cr_trim_y]) square([400, 400]);

// Acoplador do SG90 na mandibula esquerda JA IMPRESSA (so cortar os dentes, nao reimprimir): disco fino sobre o topo plano (boss + barra, z = 35), dois dentes
// que descem de cada lado da barra (grampo: trava a rotacao por contato), cubo que sobe pelo furo de 8 mm da ponte e bolso estriado (21 dentes) passante
// para a estria do SG90 (de cabeca para baixo, apoiado a cr_cp_top). Pivo de baixo: parafuso M3 x 30 por baixo da prateleira.
// ASSUNCOES (medir): estria 4,8 x 4,3 mm com ~4 mm de saliencia, ponte de MDF de 2,3 mm (calco de 0,7 mm sob o servo), barra de 4 mm (cr_jt).
cr_cp_top = cr_bz0 + cr_bt;                    // topo do cubo = face de apoio do servo (z = 39; a ponte de MDF termina em 38,3: calco de 0,7)
cr_cp_t = 0.8; cr_cp_r = 9;                    // disco: 0,8 mm (o vao entre o topo da mandibula e a ponte e 1 mm), raio 9
cr_cp_gap = 0.15; cr_cp_wall = 2; cr_cp_y0 = 5.8; cr_cp_y1 = 9.5; cr_cp_dz = 3.8;   // dentes do grampo: folga na barra, espessura, y de 5,8 a 9,5 (a orelha do anteparo fica a 10,9 mm do pivo), descem 3,8 mm
cr_hub_d = 7.6;                                // passa folgado no furo de 8 mm da ponte
cr_sp_od = 4.8; cr_sp_id = 4.3; cr_sp_fit = 0.1; cr_sp_n = 21; cr_sp_depth = 5;
module cr_spline_pocket(h) linear_extrude(height = h)
    polygon([for (i = [0 : 2 * cr_sp_n - 1]) let(a = i * 180 / cr_sp_n, r = (i % 2 == 0 ? cr_sp_od : cr_sp_id) / 2 + cr_sp_fit) [r * cos(a), r * sin(a)]]);
module cr_cp_teeth_2d() for (sx = [-1, 1]) translate([sx > 0 ? cr_jt / 2 + cr_cp_gap : -(cr_jt / 2 + cr_cp_gap + cr_cp_wall), cr_cp_y0]) square([cr_cp_wall, cr_cp_y1 - cr_cp_y0]);
module cr_coupler_local() {                    // referencial da mandibula (direita); z = 0 = topo da mandibula. A esquerda e o espelho disto (a barra e simetrica)
    hub_top = cr_cp_top - (cr_gz0 + cr_gt);
    difference() {
        union() {
            linear_extrude(cr_cp_t) { circle(r = cr_cp_r, $fn = 64); translate([-(cr_jt / 2 + cr_cp_gap + cr_cp_wall), cr_cp_y0]) square([cr_jt + 2 * (cr_cp_gap + cr_cp_wall), cr_cp_y1 - cr_cp_y0]); }
            translate([0, 0, -cr_cp_dz]) linear_extrude(cr_cp_dz + 0.01) cr_cp_teeth_2d();
            translate([0, 0, cr_cp_t - 0.01]) cylinder(d = cr_hub_d, h = hub_top - cr_cp_t + 0.01, $fn = 48);
        }
        translate([0, 0, -1]) cr_spline_pocket(hub_top + 2);                              // estria passante
    }
}

function cr_inv(a) = tan(a) - a * PI / 180;
function cr_half(r, z, m, bl) =                // semi-angulo do dente evolvente em r (graus)
    let(rp = m * z / 2, rb = rp * cos(20), rr = max(r, rb))
    (90 / z) + cr_inv(20) * 180 / PI - cr_inv(acos(rb / rr)) * 180 / PI - (bl / rp) * 180 / PI;

module cr_tooth(z, m, bl, ext = 0) {              // ext: alonga a ponta do dente (mm, so o pinhao do ombro usa; a raiz do par fica a 0,25 m da ponta)
    rp = m * z / 2; rroot = rp - 1.25 * m; rtip = rp + m + ext;
    hw = function(r) ext > 0 ? max(cr_half(r, z, m, bl), 0.3 / r * 180 / PI) : cr_half(r, z, m, bl);   // com ext a evolvente passaria da ponta e o dente se cruzaria: mantem uma ponta de 0,6 mm de largura
    rs = [for (i = [0 : 8]) rroot - 0.3 + (rtip - rroot + 0.3) * i / 8];
    up = [for (r = rs) [r * cos(hw(r)), r * sin(hw(r))]];
    dn = [for (i = [len(rs) - 1 : -1 : 0]) [rs[i] * cos(hw(rs[i])), -rs[i] * sin(hw(rs[i]))]];
    polygon(concat(up, dn));
}
module cr_sector_2d(z, m, bl, a0, a1) {        // setor de engrenagem entre a0 e a1 (graus, multiplos de 5)
    rp = m * z / 2;
    intersection() {
        union() { circle(r = rp - 1.25 * m); for (i = [0 : z - 1]) rotate(i * 360 / z) cr_tooth(z, m, bl); }
        polygon(concat([[0, 0]], [for (a = [a0 : 5 : a1]) 2 * rp * [cos(a), sin(a)]]));
    }
}

// Curva da mandibula: arco concentrico ao cubo (raio = meia diagonal), toca as 4 quinas e envolve as quinas da frente.
cr_cy = cr_stop_y + cr_stop_t + cr_cube / 2;    // y do centro do cubo preso
cr_ra = cr_cube * sqrt(2) / 2;                  // raio interno do arco
function cr_arc(r, a0, a1) = [for (i = [0 : 16]) let(a = a0 + (a1 - a0) * i / 16) [r * cos(a), cr_cy + r * sin(a)]];   // coords do frame global, x > 0
module cr_jaw_2d() {                            // perfil do lado +x em coordenadas globais (pivo em x = cr_rc)
    a1 = asin(cr_hook / cr_ra);
    translate([-cr_rc, 0]) {
        translate([cr_rc - cr_jt / 2, -cr_jback]) square([cr_jt, cr_jback + cr_cy - cr_cube / 2]);          // barra reta ate a quina de tras
        polygon(concat(cr_arc(cr_ra, -45, a1), [for (i = [16 : -1 : 0]) cr_arc(cr_ra + cr_jt, -45, a1)[i]]));
    }
}
module cr_jaw_body(phase, trim) {              // como impressa: engrenagem em z = 0 a 4, barra + arco em z = 5 a 35, boss em z = 0 a 35
    difference() {
        union() {
            translate([0, 0, cr_gz0 + cr_gt]) mirror([0, 0, 1]) {
                linear_extrude(height = cr_jh) translate([0, 0]) cr_jaw_2d();                       // barra + arco curvo (modelados de z = 0 a 30; aqui espelhados)
                cylinder(d = 10, h = cr_gz0 + cr_gt, $fn = 32);                                    // boss do pivo
            }
            linear_extrude(height = cr_gt) difference() {                                          // setor de engrenagem rumo ao outro pivo
                rotate(phase) cr_sector_2d(cr_gz, cr_gm, cr_bl, 135, 225);
                if (trim) cr_trim_2d();
            }
        }
        translate([0, 0, -1]) cylinder(d = 3.2, h = cr_gz0 + cr_gt + 2, $fn = 24);                 // pino M3
    }
}
module cr_jaw_right(psi, phase, drive = false, trim = true) {   // mandibula do lado +x; psi = 0 fechada (toca o cubo), psi > 0 abre
    translate([cr_rc, 0, 0]) rotate([0, 0, -psi]) {
        cr_jaw_body(phase, trim);
        if (drive) color("seagreen") translate([0, 0, cr_gz0 + cr_gt]) cr_coupler_local();         // acoplador do SG90 (peca separada)
    }
}
module cr_jaw_left(psi, drive = false, trim = true) { mirror([1, 0, 0]) cr_jaw_right(psi, 180 / cr_gz, drive, trim); }   // meio passo de fase: dente encontra vao
module cr_jaws(psi) { cr_jaw_right(psi, 0); cr_jaw_left(psi, true); }

module cr_frame() {                            // ponte, placa traseira, prateleira e anteparo (peca unica impressa)
    difference() {
        union() {
            box(-36, 36, -cr_rear, 14, cr_bz0, cr_bz0 + cr_bt);                                     // ponte (sobre as engrenagens)
            box(-36, 36, -cr_rear, -cr_rear + 4, -3, cr_bz0 + cr_bt);                               // placa traseira, encosta na placa do carrier
            box(-24, 24, -cr_rear, cr_stop_y + cr_stop_t, -3, 0);                                   // prateleira sob as mandibulas
            box(-24, 24, cr_stop_y, cr_stop_y + cr_stop_t, -3, 30);                                 // anteparo: o cubo encosta aqui
        }
        for (s = [-1, 1]) translate([s * cr_rc, 0, cr_bz0 - 1]) cylinder(d = 3.4, h = cr_bt + 2, $fn = 24);   // pinos dos pivos
        translate([-cr_rc, 0, cr_bz0 - 1]) cylinder(d = 8, h = cr_bt + 2, $fn = 24);                            // eixo do SG90
        box(-10.2, 10.2, cr_stop_y - 1, cr_stop_y + 3, 3, 23.4);                                    // recesso da placa TCS34725 (20 x 20), por tras
        translate([0, cr_stop_y - 2, 13]) rotate([-90, 0, 0]) cylinder(d = 10, h = cr_stop_t + 4, $fn = 32);   // janela do chip
        for (x = [-14, 14]) translate([x, -cr_rear - 1, cr_bz0 + 1.5]) rotate([-90, 0, 0]) cylinder(d = 3.2, h = 8, $fn = 24);   // M3 na placa do carrier
    }
}

module cr_dummies() {                          // so para ilustracao
    color("royalblue") box(-cr_rc - 6.1, -cr_rc + 6.1, -16.5, 6, cr_bz0 + cr_bt, cr_bz0 + cr_bt + 22.7);     // SG90 de cabeca para baixo
    color("#f59e0b") box(-10, 10, cr_stop_y - 0.2, cr_stop_y + 1.4, 3, 23);                                   // TCS34725 atras do anteparo
}
