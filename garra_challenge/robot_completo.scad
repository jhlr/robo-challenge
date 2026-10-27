// Ilustracao conceitual do robo completo: chassi (estimado), eletronica, sensores e a garra.
// Medidas do chassi e da eletronica sao APROXIMADAS (chassi novo ainda nao foi medido). A garra e a do modelo real.
// Uso: openscad -o robo.png --camera=330,360,230,0,40,40 -D 'lift=30' robot_completo.scad
include <garra_challenge.scad>
part = "none";            // sobrescreve o include: so usamos os modulos da garra
show_dummies = true;
open = true;

// ---------- chassi: deck medido 156 x 94, roda 66; resto estimado ----------
half_w = 47;              // meia largura do deck (chassi novo, medido: 156 x 94 mm)
half_l = 78;              // meio comprimento do deck (medido)
wheel_d = 66; wheel_w = 24; wheel_y = 45; axle_z = wheel_d / 2;
upper_z = deck_h + 40;    // segundo convés (vao de 4 cm do chassi antigo; ajustar ao novo)
gar_y = half_l - (rod_y + 16);   // base do mastro rente a frente do chassi

wing = 40;                // asas do convés de cima (chassi/alargar_superior.py, WING): largura 94 -> 174; as rodas externas chegam a |x| = 91, entao a largura total do robo nao muda
module deck(z0, z1, y0, y1, col = "#d9b382", w = half_w) { color(col) box(-w, w, y0, y1, z0, z1); }

module side_module(s) {
    // placas de MDF em sanduiche, engrenagens e rodas (4 rodas, 1 STS3215 por lado)
    for (k = [0, 1]) color("#d9b382") box(s > 0 ? half_w + 3 * k : -half_w - 3 * k - 3, s > 0 ? half_w + 3 * k + 3 : -half_w - 3 * k, -62, 62, 12, 56);
    color("#202020") for (y = [-wheel_y, 0, wheel_y])
        translate([s * (half_w + 8), y, axle_z]) rotate([0, 90, 0]) cylinder(d = y == 0 ? 40 : 46, h = 6, center = true);
    color("#1a1a1a") for (y = [-wheel_y, wheel_y])
        translate([s * (half_w + 6 + 12 + 2), y, axle_z]) rotate([0, 90, 0]) cylinder(d = wheel_d, h = wheel_w, center = true);
    color("#6b7280") for (y = [-wheel_y, wheel_y])                                                   // cubo da roda
        translate([s * (half_w + 6 + 12 + 2 + wheel_w / 2), y, axle_z]) rotate([0, 90, 0]) cylinder(d = 38, h = 1, center = true);
    color("#555b66") box(s > 0 ? half_w - 24.7 : -half_w, s > 0 ? half_w : -half_w + 24.7, -22.6, 22.6, axle_z - 17.5, axle_z + 17.5);   // STS3215
}

scene = "linear";          // "linear" = garra do mastro; "arm" = braco paralelogramo (braco/braco.scad)

module robot_base(arm_mode = false) {
    ua0 = arm_mode ? -85 : -75;                  // convés de cima: no braco ele vai ate atras e mais para a frente
    ua1 = arm_mode ? 42 : 30;
    deck(deck_h - 3, deck_h, -half_l, half_l);
    deck(upper_z, upper_z + 3, ua0, ua1, w = half_w + wing);
    color("#999") for (x = [-45, 45], y = [ua0 + 7, ua1 - 6]) translate([x, y, deck_h]) cylinder(d = 6, h = upper_z - deck_h);
    side_module(1); side_module(-1);

    // eletronica
    if (arm_mode) {                                                                                  // ESP32 DevKit (51,5 x 28,6), pinos MACHO para BAIXO: encaixa em 2 soquetes femea (8,5 mm) sobre o convés; faixa de tras (y < -55) para nao bater no berco do servo do ombro
        color("#111") for (y = [-82.7, -59.4]) box(-8, 39.5, y, y + 2.5, upper_z + 3, upper_z + 11.5);   // soquetes femea (2 x 30 posicoes, 2,54 mm); linhas a 25,4 mm
        color("#1f5fbf") box(-10, 41.5, -84, -55.4, upper_z + 11.5, upper_z + 13.1);                 // PCB do DevKit
        color("#cbd5e1") box(26, 40, -80, -60, upper_z + 13.1, upper_z + 16.2);                       // modulo ESP-WROOM
        color("#9ca3af") box(-14, -10, -72, -66, upper_z + 13.1, upper_z + 15.6);                     // micro USB (para fora da borda, lado -x)
        color("#f1c40f") translate([38, -12, upper_z + 3]) cylinder(d = 12, h = 9);                  // buzzer
        color("#e11d48") translate([-30, -70, upper_z + 3]) cylinder(d = 5, h = 8);                  // LED (faixa de tras do convés, fora do berco do servo)
        color("#1d4ed8") box(-22.5, 22.5, half_l - 4, half_l - 2, deck_h, deck_h + 20);               // ultrassom HC-SR04 frontal (PCB 45 x 20, em pe, rente a frente do deck)
        color("#cbd5e1") for (x = [-12.5, 12.5]) translate([x, half_l - 2, deck_h + 10]) rotate([-90, 0, 0]) cylinder(d = 16, h = 10);   // transdutores (emissor e receptor)
    } else {
        color("#1f5fbf") box(-26.7, 26.7, -70, 31.6, upper_z + 3, upper_z + 7.5);                   // Arduino Mega
        color("#111") for (x = [-24, 22]) box(x, x + 3, -66, 28, upper_z + 7.5, upper_z + 15);       // headers
    }
    color("#8e44ad") box(30, 46, 8, 22, upper_z + 3, upper_z + 6);                                   // IMU
    color("#2a4d9b") box(-17.5, 17.5, -55, 35, deck_h, deck_h + 20);                                 // LiPo 2S
    color("#e74c3c") translate([0, 36, deck_h + 12]) box(-0.5, 0.5, -1, 1, -1, 1);
    color("#2b8a3e") box(22, 44, -72, -27, deck_h, deck_h + 12);                                     // buck 5 V
    color("#16a085") box(-44, -22, -72, -32, deck_h, deck_h + 7);                                    // adaptador de barramento
    color("#c0392b") box(0, 14, -82, -72, deck_h, deck_h + 12);                                      // chave + fusivel

    // sensores do chassi (sob o convés e nos cantos)
    color("#a31f1f") box(-35, 35, 62, 77, 7, 10);                                                    // linha de IR (8 canais)
    color("#111") for (i = [0 : 7]) box(-33 + i * 9, -33 + i * 9 + 5, 66, 72, 3, 7);
    color("#d63384") for (x = [-22, 22]) box(x - 7.5, x + 7.5, 42, 57, 7, 10);                       // sensor de cor do chao
    color("#84cc16") for (s = [-1, 1]) box(s * 47 - 1, s * 47 + 1, 58, 71, deck_h, deck_h + 13);     // ToF laterais

}

module linear_gripper() {
    // garra (modelo real), pose pronta para pegar
    translate([0, gar_y, deck_h]) {
        color("lightgray") mast();
        color("tomato") carriage_world();
        color("khaki") sliders_world();
        color("black") horn_world();
        color("orange") pinion_world();
        dummies();
        color("#f1c40f") translate([-8, 16, mast_h]) cylinder(d = 12, h = 9);                          // buzzer
        color("#e11d48") translate([6, 16, mast_h]) cylinder(d = 5, h = 8);                            // LED
        translate([0, 0, zc]) {
            color("#84cc16") box(-9, 9, tof_y0 + 3, tof_y0 + 5, -15.5, -3);                            // ToF VL53L1X
            color("#f59e0b") translate([dx, 0, 0]) box(grip_half_c + finger_t + 2, grip_half_c + finger_t + 4, cs_y - 10, cs_y + 10, cs_z - 10, cs_z + 10);  // TCS34725 do cubo
        }
    }
}

module robot() { robot_base(false); linear_gripper(); }

if (scene == "linear") {
    color("#e5e7eb") translate([-100, -100, -1]) cube([200, 270, 1]);                       // piso
    color("#dc2626") translate([-25, gar_y + 80, 0]) cube([50, 50, 50]);                    // cubo de 50 mm para escala
    robot();
}
