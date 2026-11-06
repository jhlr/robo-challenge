// Ombro com servo TowerPro MG996R ligado DIRETO ao pivo da barra de baixo do lado -x (sem reducao). Substitui o SG90 4:1 (ombro_reducao.scad).
// Torque: ~10 kg.cm (6 V) contra 4,2 kg.cm do ombro = 2,4x de folga. Eixo do servo = x, apontando para +x; pivo das barras em (y = 0, z = z_s).
// O horn do servo fica entre o corpo e a face de fora da barra, aparafusado nela; o M3 do eixo entra por dentro (furo de pivo da barra). Corpo no lado de fora (x < 0).
// Medidas de catalogo do MG996R (MEDIR com paquimetro, principalmente sh_ear_below e sh_off): corpo 40,7 x 19,7, altura total 42,9 ate o topo do eixo, orelhas de 54,4 mm.
sh_bar_face = bar_x + 1.15;                         // face externa da barra de MDF de 2,3 mm (centro em |x| = bar_x)
sh_horn_t = 3; sh_gap = 0.3;                        // horn entre a barra e o topo do eixo; folga
sh_top = -(sh_bar_face + sh_horn_t + sh_gap);       // x do topo do cubo do eixo (face de cima do servo)
sh_L = 40.7; sh_W = 19.7; sh_H = 42.9;              // corpo: comprimento (y), largura (z), altura (x) ate o topo do eixo
sh_off = 10.3;                                      // eixo a 10,3 mm de uma ponta do corpo
sh_ear_below = 14;                                  // face de cima das orelhas, abaixo do topo do eixo
sh_ear_t = 2.5; sh_ear_len = 54.4;                  // espessura e comprimento total com as orelhas
sh_hole = [49.5, 10]; sh_hole_d = 4.2;              // furos das orelhas (y, z), M4
sh_y1 = sh_off; sh_y0 = sh_off - sh_L;              // extensao do corpo em y: para tras (-y), longe das torres
sh_z0 = z_s - sh_W / 2; sh_z1 = z_s + sh_W / 2;
sh_ear_x = sh_top - sh_ear_below;                   // x da face de cima das orelhas (a chapa de MDF fica por baixo)
sh_horn_y = [8, 16];                                // M2,5 do horn na barra, a 8 e 16 mm do pivo (MEDIR o horn de 2 bracos do kit)
sh_horn_d = 2.7;

module sh_servo_body() color("royalblue") {         // corpo + orelhas (fixos)
    box(sh_top - sh_H, sh_top, sh_y0, sh_y1, sh_z0, sh_z1);
    box(sh_ear_x - sh_ear_t, sh_ear_x, sh_y0 - (sh_ear_len - sh_L) / 2, sh_y1 + (sh_ear_len - sh_L) / 2, sh_z0, sh_z1);
}
module sh_horn(alpha) color("orange") translate([0, 0, z_s]) rotate([alpha, 0, 0]) translate([0, 0, -z_s]) box(sh_top, sh_top + sh_horn_t, -6, 30, z_s - 6, z_s + 6);
module sh_drive(alpha) { sh_servo_body(); sh_horn(alpha); }
