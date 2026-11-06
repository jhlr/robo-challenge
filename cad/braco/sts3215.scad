// Feetech STS3215 (fonte unica: barra do ombro, estrutura MDF do ombro e engrenagem da garra usam isto).
// Corpo: desenho 2D da Waveshare (docs/2026-11-06_sts3215-disco.md). Disco de metal: MEDIDO no real em 2026-11-06.
sts_L = 45.22; sts_W = 24.72; sts_H = 35;      // corpo: comprimento, largura, altura entre as faces do disco (eixo ao longo da altura)
sts_off = 10.11;                               // eixo a 10,11 mm de uma ponta do comprimento
sts_disc_prot = 1.125;                         // quanto o disco sai da face: (37,25 do desenho - 35) / 2. MEDIR no real
sts_disc_d = 20;                               // diametro do disco
sts_hole_r = 7; sts_hole_d = 3;                // 4 furos roscados M3 em cruz, a 7 mm do eixo (circulo de 14)
sts_screw_d = 3.2;                             // folga do M3 passante (MDF/impresso)
sts_center_d = 8;                              // furo central da barra: deixa passar a cabeca do parafuso do eixo. MEDIR a cabeca
