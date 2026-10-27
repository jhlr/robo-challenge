// Garra RCB Challenge 2026 — v2 leve, parametrico (OpenSCAD)
// Eixos: x = lateral, y = frente (alcance), z = cima. Unidades: mm.
// Pinca paralela (1 servo SG90, jugo escoces) + elevador de cremalheira: STS3215 na carriage, pinhao evolvente, cremalheira fixa no mastro.
// Uso: openscad -D 'part="assembly"' -D 'lift=40' -D 'open=true' garra_challenge.scad
// part: assembly | mast | carriage | claw_housing | pinion | slider_a | slider_b | horn | chk_*

$fn = 48;

part = "assembly";
lift = 0;          // 0..stroke, subida da carriage (mm)
open = true;       // pinca aberta?
show_dummies = true;

// ---------- pinca ----------
grip_half_c  = 24.5;   // meia-abertura fechada (< 25 => aperta o cubo de 50)
grip_travel  = 7;      // curso de cada dedo
r_h          = 14;     // raio do braco do horn
th_c         = acos(1 - grip_travel / r_h);
theta        = open ? 0 : th_c;
finger_len   = 40;
finger_h     = 30;
finger_t     = 3;
plate_t      = 3;      // espessura dos sliders

// ---------- elevador ----------
stroke       = 62;     // plataforma tem 50 mm de altura: 62 cobre a 1a camada (112 para empilhar a 2a)
rod_d        = 4;
rod_x        = 18;
rod_y        = 38;     // mancais atras do corpo do STS3215
yc           = rod_y + 22;   // y do centro do horn/servo da pinca
tof_y0       = yc + 9;       // inicio do suporte do ToF (sob a base da pinca)
deck_h       = 25;     // altura do chassi de baixo ate o chao (novo chassi, ~1 polegada, aproximado)
zc_low       = 6;      // z mundo (sobre o chassi) do z=0 local da carriage, com lift=0; dedos ficam a deck_h+zc_low do chao
zp_l         = 12;     // z local do eixo do pinhao/STS3215
wall_t       = 3;
// STS3215 (etiqueta: 45,22 x 24,7 x 35 mm). Eixo ao longo de +x; o eixo de saida fica a sts_shaft_z da ponta de baixo (SUPOSICAO, conferir no servo real)
sts_x        = 35.0;   // altura do corpo, ao longo do eixo
sts_y        = 24.7;   // espessura
sts_z        = 45.22;  // comprimento
sts_shaft_z  = 11.3;
sts_face     = 13;     // x da face de saida do servo
mast_h       = zc_low + stroke + (zp_l + sts_z - sts_shaft_z + 2) + 4;

// cremalheira/pinhao: modulo 1.5, 14 dentes (66 mm por volta > curso de 62 mm), evolvente 20 graus
m = 1.5;  zt = 14;
p = PI * m;
bl = 0.15;                          // folga por flanco, ajustar pela impressora
r_p = m * zt / 2;
rack_bar = 3;
yr = wall_t + rack_bar + 1.25 * m;  // linha primitiva da cremalheira
yp = yr + r_p;                      // y do eixo do pinhao
pin_x0 = 15; pin_w = 8;
rack_x0 = 14.5; rack_w = 9;

// ---------- helpers ----------
module box(x0, x1, y0, y1, z0, z1) {
    translate([x0, y0, z0]) cube([x1 - x0, y1 - y0, z1 - z0]);
}
module yz_extrude(x0, w) {
    translate([x0, 0, 0]) rotate([90, 0, 90]) linear_extrude(height = w) children();
}

// ---------- cremalheira (fixa no mastro) ----------
module rack_teeth() {
    z_c  = zc_low + zp_l;
    n    = ceil((stroke + 2 * r_p + 8) / p);
    root = yr - 1.25 * m;
    hw_r = p / 4 + tan(20) * 1.25 * m - bl;
    hw_t = p / 4 - tan(20) * 1.0 * m - bl;
    tip  = yr + 1.0 * m;
    yz_extrude(rack_x0, rack_w)
        for (k = [0 : n]) {
            zk = z_c + p / 2 + (k - 4) * p;
            polygon([[root - 0.2, zk - hw_r], [tip, zk - hw_t],
                     [tip, zk + hw_t], [root - 0.2, zk + hw_r]]);
        }
}

// ---------- mastro ----------
module wall_open() {   // parede 3 mm com janelas de alivio (nervuras de 4 mm)
    row_h = 18; rib = 4; z0 = 8;
    rows = floor((mast_h - 3 - z0 - rib) / (row_h + rib));
    difference() {
        box(-24, 24, 0, wall_t, 0, mast_h);
        for (i = [0 : rows - 1])
            box(-20, 10, -1, wall_t + 1, z0 + i * (row_h + rib), z0 + i * (row_h + rib) + row_h);
    }
}
module mast() {
    difference() {
        union() {
            difference() {
                box(-30, 30, 0, rod_y + 16, 0, 3);
                box(-12, 12, 8, rod_y + 10, -1, 4);                          // janela central
            }
            wall_open();
            box(-24, 24, 0, rod_y + 4, mast_h - 3, mast_h);                  // tampa (segura as hastes)
            box(rack_x0, rack_x0 + rack_w, wall_t - 0.5, wall_t + rack_bar, 3, mast_h - 3);   // barra da cremalheira
            rack_teeth();
            translate([24.5, 0, 0]) rotate([90, 0, 90])                      // reforco do lado da cremalheira
                linear_extrude(height = 2.5) polygon([[0, 3], [20, 3], [0, 40]]);
        }
        for (s = [-1, 1]) translate([s * rod_x, rod_y, -1]) cylinder(d = rod_d + 0.1, h = mast_h + 2);
        translate([-8, 16, mast_h - 4]) cylinder(d = 12.3, h = 6);                                          // buzzer ativo 12 mm
        translate([6, 16, mast_h - 4]) cylinder(d = 5.1, h = 6);                                            // LED 5 mm
        for (pt = [[26, 10], [26, rod_y + 10], [-26, rod_y + 10], [-26, rod_y - 2]]) translate([pt[0], pt[1], -1]) cylinder(d = 3.4, h = 6);  // M3 no chassi
    }
}

// ---------- carriage + housing da pinca (frame local) ----------
module claw_housing(link = true) {   // base da pinca: guias dos sliders, apoios do SG90 e suporte do ToF (reusado pelo braco)
    difference() {
        union() {
            box(-36, 36, rod_y + 0.5, yc + 14 + 0.3 + 2, -2.5, 0);                               // base
            for (y = [yc - 14 - 0.3 - 2, yc + 14 + 0.3]) box(-22, 22, y, y + 2, 0, plate_t + 0.2);  // guias dos sliders
            box(-8.3, 8.3, yc - 21.5, yc - 16.5, 0, 24);                                         // apoio traseiro do servo
            if (link) box(-8.3, 8.3, rod_y - 3, yc - 21.5 + 0.5, -1, 24);                        // liga ao carriage
            for (s = [-1, 1]) box(s > 0 ? 6.3 : -8.3, s > 0 ? 8.3 : -6.3, yc - 21.5, yc + 11, 8, 24); // bracos
            box(-8.3, 8.3, yc + 6.3, yc + 11, 8, 24);                                            // apoio dianteiro do servo
            box(-11, 11, tof_y0, yc + 14 + 0.3 + 2, -16.5, -2.5);                                // suporte do sensor de distancia (ToF)
        }
        for (y = [yc - 19, yc + 8.5]) translate([0, y, 10]) cylinder(d = 1.9, h = 20);           // M2 do servo
        box(-9.1, 9.1, tof_y0 + 3, yc + 14 + 0.3 + 2 + 1, -15.5, -2.5 - 0.5);                      // bolso da placa VL53L1X (18 x 13 mm), aberto na frente
        for (x = [-7, 7]) translate([x, tof_y0 - 1, -9]) rotate([-90, 0, 0]) cylinder(d = 2.0, h = 10);  // M2 da placa
    }
}
module carriage() {
    difference() {
        union() {
            for (s = [-1, 1]) translate([s * rod_x, rod_y, -1]) cylinder(d = rod_d + 5, h = 22);   // mancais
            box(-rod_x, rod_x, rod_y - 4, rod_y - 1, -1, 21);                                    // alma entre mancais
            box(sts_face - sts_x - 2.3, 14, yp - sts_y / 2 - 2.3, rod_y - 3.5, zp_l - sts_shaft_z - 2.3, zp_l + sts_z - sts_shaft_z + 2.3);   // berco do STS3215 + ponte
            claw_housing(link = true);
        }
        for (s = [-1, 1]) translate([s * rod_x, rod_y, -10]) cylinder(d = rod_d + 0.3, h = 40);
        box(sts_face - sts_x - 0.3, 15, yp - sts_y / 2 - 0.3, yp + sts_y / 2 + 0.3, zp_l - sts_shaft_z - 0.3, zp_l + sts_z - sts_shaft_z + 0.3);   // bolso do STS3215 (aberto em +x); fixar com M3 nas orelhas
        for (s = [-1, 1]) box(s * 9 - 3, s * 9 + 3, rod_y - 5, rod_y, 2, 18);                    // janelas de alivio na alma
    }
}
// ---------- pinhao evolvente ----------
function inv(a) = tan(a) - a * PI / 180;
r_b = r_p * cos(20);
r_root = r_p - 1.25 * m;
r_tip  = r_p + m;
function half_ang(r) =
    let(rr = max(r, r_b))
    (90 / zt) + inv(20) * 180 / PI - inv(acos(r_b / rr)) * 180 / PI - (bl / r_p) * 180 / PI;
module tooth_2d() {
    rs = [for (i = [0 : 8]) r_root - 0.3 + (r_tip - r_root + 0.3) * i / 8];
    up = [for (r = rs) [r * cos(half_ang(r)), r * sin(half_ang(r))]];
    dn = [for (i = [len(rs) - 1 : -1 : 0]) [rs[i] * cos(half_ang(rs[i])), -rs[i] * sin(half_ang(rs[i]))]];
    polygon(concat(up, dn));
}
module pinion_2d() {
    circle(r = r_root);
    for (i = [0 : zt - 1]) rotate(180 + i * 360 / zt) tooth_2d();
}
module pinion() {   // eixo ao longo de +x, centro em (y,z)=(0,0)
    difference() {
        yz_extrude(0, pin_w) pinion_2d();
        translate([-1, 0, 0]) rotate([90, 0, 90]) linear_extrude(height = pin_w + 2) {
            circle(d = 6.0);                                                                  // furo central: casar com o horn do STS3215
            for (a = [0 : 90 : 270]) rotate(a) translate([6.5, 0]) circle(d = 2.2);            // 4 furos M2: padrao PROVISORIO, medir o horn
        }
    }
}

// ---------- sliders + dedos (placa de 3 mm com janelas) ----------
cs_y = yc + 14 + 20;   // y do sensor de cor no dedo A (centro da janela do dedo)
cs_z = 15;
module slider_a() {   // pose fechada; slot em x=7, y positivo; dedo A leva o sensor de cor do cubo
    ys = yc - 14; ye = yc + 14;
    difference() {
        union() {
            box(3, grip_half_c + finger_t, ys, ye, 0, plate_t);
            box(grip_half_c, grip_half_c + finger_t, ye, ye + finger_len, 0, finger_h);
            box(grip_half_c + finger_t, grip_half_c + finger_t + 4.5, cs_y - 12, cs_y + 12, cs_z - 12, cs_z + 12);  // boss da placa
        }
        box(grip_half_c + finger_t + 2, grip_half_c + finger_t + 5, cs_y - 10.2, cs_y + 10.2, cs_z - 10.2, cs_z + 10.2);  // recesso da placa 20,4 x 20,4
        translate([grip_half_c - 2, cs_y, cs_z]) rotate([0, 90, 0]) cylinder(d = 9, h = 12);              // janela do chip
        box(r_h * cos(th_c) - 1.6, r_h * cos(th_c) + 1.6, yc - 1.6, yc + r_h * sin(th_c) + 1.6, -1, plate_t + 1);
        box(11, 22, yc - 9, yc + 9, -1, plate_t + 1);
        box(grip_half_c - 1, grip_half_c + finger_t + 1, ye + 6, ye + finger_len - 6, 6, finger_h - 6);
    }
}
module slider_b_pre() {   // slot com y negativo; espelhado em x vira o slider B
    ys = yc - 14; ye = yc + 14;
    difference() {
        union() {
            box(3, grip_half_c + finger_t, ys, ye, 0, plate_t);
            box(grip_half_c, grip_half_c + finger_t, ye, ye + finger_len, 0, finger_h);
        }
        box(r_h * cos(th_c) - 1.6, r_h * cos(th_c) + 1.6, yc - r_h * sin(th_c) - 1.6, yc + 1.6, -1, plate_t + 1);
        box(11, 22, yc - 9, yc + 9, -1, plate_t + 1);
        box(grip_half_c - 1, grip_half_c + finger_t + 1, ye + 6, ye + finger_len - 6, 6, finger_h - 6);
    }
}
module slider_b() { mirror([1, 0, 0]) slider_b_pre(); }
module horn() {       // centro em (0,yc), z=3.4..6.4, pinos descem ate z=0.6
    translate([0, yc, 0]) rotate([0, 0, theta])
        difference() {
            union() {
                translate([0, 0, plate_t + 0.4]) linear_extrude(height = 3)
                    hull() { translate([-r_h, 0]) circle(d = 6); translate([r_h, 0]) circle(d = 6); }
                for (s = [-1, 1]) translate([s * r_h, 0, 0.6]) cylinder(d = 2.9, h = 3);
            }
            translate([0, 0, plate_t]) cylinder(d = 4.8, h = 4);
        }
}

// ---------- montagem ----------
zc = zc_low + lift;
dx = r_h * (cos(theta) - cos(th_c));

module sliders_posed() {
    translate([dx, 0, 0]) slider_a();
    translate([-dx, 0, 0]) slider_b();
}
module carriage_world() { translate([0, 0, zc]) carriage(); }
module sliders_world()  { translate([0, 0, zc]) sliders_posed(); }
module horn_world()     { translate([0, 0, zc]) horn(); }
module servo_world()    { translate([0, 0, zc]) box(-6.1, 6.1, yc - 16.5, yc + 6, 8, 30.7); }
module motor_world()    { translate([0, 0, zc]) box(sts_face - sts_x, sts_face, yp - sts_y / 2, yp + sts_y / 2, zp_l - sts_shaft_z, zp_l + sts_z - sts_shaft_z); }
module pinion_world() {
    translate([pin_x0, yp, zc + zp_l]) rotate([lift / r_p * 180 / PI, 0, 0]) pinion();
}

module dummies() {
    color("silver") for (s = [-1, 1]) translate([s * rod_x, rod_y, 0]) cylinder(d = rod_d, h = mast_h);
    color("gold") motor_world();           // STS3215, eixo +x
    color("royalblue") servo_world();      // SG90
}

if (part == "assembly") {
    color("lightgray") mast();
    color("tomato") carriage_world();
    color("khaki") sliders_world();
    color("black") horn_world();
    color("orange") pinion_world();
    if (show_dummies) dummies();
}
if (part == "mast") mast();
if (part == "carriage") carriage();
if (part == "claw_housing") claw_housing(link = false);
if (part == "pinion") pinion();
if (part == "slider_a") slider_a();
if (part == "slider_b") slider_b();
if (part == "horn") translate([0, -yc, -plate_t - 0.4]) horn();

// checagens de interferencia (resultado vazio = sem colisao)
if (part == "chk_mast_carriage")   intersection() { mast(); carriage_world(); }
if (part == "chk_mast_sliders")    intersection() { mast(); sliders_world(); }
if (part == "chk_mast_pinion")     intersection() { mast(); pinion_world(); }
if (part == "chk_carr_pinion")     intersection() { carriage_world(); pinion_world(); }
if (part == "chk_slider_ab")       intersection() { translate([dx, 0, 0]) slider_a(); translate([-dx, 0, 0]) slider_b(); }
if (part == "chk_horn_sliders")    intersection() { horn(); sliders_posed(); }
if (part == "chk_horn_carriage")   intersection() { horn(); carriage(); }
if (part == "chk_servo_sliders")   intersection() { servo_world(); sliders_world(); }
if (part == "chk_servo_mast")      intersection() { servo_world(); mast(); }
if (part == "chk_servo_carriage")  intersection() { servo_world(); carriage_world(); }
if (part == "chk_motor_mast")      intersection() { motor_world(); mast(); }
if (part == "chk_motor_pinion")    intersection() { motor_world(); pinion_world(); }
if (part == "chk_carr_sliders")    intersection() { carriage_world(); sliders_world(); }
