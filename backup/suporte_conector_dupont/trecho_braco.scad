
// Suporte do conector do servo (Dupont femea 1x3, 3 vias): bloco com bolso para a carcaca do conector + canal do cabo + 2 passagens de abracadeira de nylon na base (prende no berco/prateleira). Medidas da carcaca Dupont padrao, MEDIR.
dc_w = 3 * 2.54 + 0.5; dc_t = 2.54 + 0.4; dc_h = 14;       // carcaca: 7,62 x 2,54 x ~14 mm, com folga
dc_wall = 1.6; dc_base = 2;
module dupont_holder() difference() {
    union() {
        translate([-(dc_w / 2 + dc_wall + 4), -(dc_t / 2 + dc_wall + 4), 0]) cube([dc_w + 2 * dc_wall + 8, dc_t + 2 * dc_wall + 8, dc_base]);   // base com abas para as abracadeiras
        translate([-(dc_w / 2 + dc_wall), -(dc_t / 2 + dc_wall), 0]) cube([dc_w + 2 * dc_wall, dc_t + 2 * dc_wall, dc_base + 10]);                // bloco
    }
    translate([-dc_w / 2, -dc_t / 2, dc_base + 0.01]) cube([dc_w, dc_t, 20]);                                    // bolso do conector (entra por cima)
    translate([-(dc_w / 2 - 1), -dc_t / 2 - dc_wall - 1, dc_base + 0.01]) cube([dc_w - 2, dc_wall + 2, 20]);     // saida lateral do cabo
    for (s = [-1, 1]) translate([s * (dc_w / 2 + dc_wall + 2), 0, -1]) cube([1.6, 4.5, dc_base + 2], center = false);    // passagens de abracadeira de 2,5 mm
}
if (scene == "part_sv_conector") dupont_holder();
