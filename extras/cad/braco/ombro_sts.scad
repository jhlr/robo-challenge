// Ombro com o STS3215 ligado DIRETO ao pivo da barra de baixo do lado -x (sem reducao: 19,5 kg.cm sobra para o ombro). Substitui MG996R + 4:1 (old/ombro_mg996r_4x1).
// Eixo do servo = x, pivo das barras em (y = 0, z = z_s). A barra fica entre as torres e o disco: o disco encosta na face de DENTRO do servo, o corpo fica do lado de fora (x < 0),
// com o comprimento de 45,22 mm ao longo de y (para tras, ate -35,11) e a largura de 24,72 mm ao longo de z. Parafusos: 4 x M3 FRESADOS da barra para o disco (a cabeca nao pode passar da face da barra: a torre esta a 0,7 mm).
sts_top = -(bar_x + bar_t / 2 + sts_disc_prot);     // x da face do corpo do lado do disco (face externa da barra em |x| = bar_x + bar_t / 2)
sts_x0 = sts_top - sts_H;                           // x da face de tras (segunda ponta do eixo duplo)
sts_y0 = -(sts_L - sts_off); sts_y1 = sts_off;      // extensao do corpo em y
sts_z0 = z_s - sts_W / 2; sts_z1 = z_s + sts_W / 2;
sts_boss_d = sts_disc_d + 2;                        // pivo alargado da barra do ombro

module sts_servo_dummy() color("royalblue") {       // so ilustracao/colisao
    box(sts_x0, sts_top, sts_y0, sts_y1, sts_z0, sts_z1);
    translate([sts_top, 0, z_s]) rotate([0, 90, 0]) cylinder(d = sts_disc_d, h = sts_disc_prot, $fn = 48);
}
module sts_drive() sts_servo_dummy();
