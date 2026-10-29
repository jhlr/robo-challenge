// ESP32 DevKit tray (CP2102, 38-pin class), 2026-10-29. Open top so the Dupont wires on the headers leave upward; USB end notched; two M3 tabs on the long sides.
// The board rests on its own pin tips on the flat floor (no standoffs: the pin rows sit right at the long edges). All sizes are parameters: measure YOUR board and adjust.
$fn = 48;

board_l = 51.5;   // board length (same as cad/modelo_robo/robot_completo.scad)
board_w = 28.6;   // board width
board_t = 1.6;
pin_tip = 3.0;    // pin tips below the PCB (board stands on them)
header_h = 2.5;   // plastic header above the PCB
clr = 0.3;        // clearance per side
wall = 2.0;
floor_t = 2.0;

usb_w = 12.0;     // notch width at the USB end (x = 0)
tab_len = 10.0;   // tab size along the box length
tab_out = 8.0;    // how far the tab sticks out of the wall
tab_t = 3.0;
screw_d = 3.4;    // M3 clearance

// Hold-down lip on the end opposite the USB, centered: it overhangs the bare PCB end (the antenna keep-out has no parts there).
// Fit: lay the board level with its far edge `slide` short of the wall, then push it `slide` toward the far wall so the lip covers the PCB end.
lip_w = 8.0;      // lip width
lip_over = 2.0;   // overhang over the board end
lip_t = 1.2;      // lip thickness
lip_gap = 0.5;    // air between the PCB top and the lip
slide = 2.0;      // extra cavity length (>= lip_over) to get the board under the lip

cav_l = board_l + 2 * clr + slide;
cav_w = board_w + 2 * clr;
out_l = cav_l + 2 * wall;
out_w = cav_w + 2 * wall;
H = floor_t + pin_tip + board_t + header_h - 0.1;   // walls end just under the header tops

module tab(side) {   // side = +1 (y = out_w) or -1 (y = 0)
  y0 = side > 0 ? out_w - wall : -tab_out;
  difference() {
    translate([out_l / 2 - tab_len / 2, y0, 0]) cube([tab_len, tab_out + wall, tab_t]);
    translate([out_l / 2, side > 0 ? out_w + tab_out / 2 : -tab_out / 2, -1]) cylinder(d = screw_d, h = tab_t + 2);
  }
}

union() {
difference() {
  union() {
    cube([out_l, out_w, H]);
    tab(1);
    tab(-1);
  }
  translate([wall, wall, floor_t]) cube([cav_l, cav_w, H]);                                   // cavity
  translate([-1, out_w / 2 - usb_w / 2, floor_t + pin_tip - 0.5]) cube([wall + 2, usb_w, H]);   // USB notch, open to the top
}
translate([out_l - wall - lip_over, out_w / 2 - lip_w / 2, floor_t + pin_tip + board_t + lip_gap]) cube([lip_over + wall / 2, lip_w, lip_t]);   // hold-down lip
}
