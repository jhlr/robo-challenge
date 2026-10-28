# Robo Challenge 2026

Robo autonomo para o desafio (regras: 300 x 300 mm com tudo estendido, totalmente autonomo). Chassi com 2 conveses, braco paralelogramo em MDF com ombro 4:1 e garra rotativa, sensores de linha, cor e ultrassom.

```
firmware/                 codigo que roda no robo
  src/ include/ test/       ESP32 DevKit (PlatformIO): bringup (menu serial) e follow (seguidor com juncoes); testes de src/core rodam no PC
  SeguidorRC/               biblioteca Arduino do seguidor de linha (ESP32-S3 + TB6612 + 8 sensores RC): sensores, decisao, ponte H; exemplo em examples/
cad/                      modelos e pecas
  braco/                    braco em OpenSCAD: params.json -> cinematica.py -> params_gen.scad; pecas de MDF (mdf/*.dxf) e STL impressos
  chassi/                   conves de cima alargado (script + DXF)
  modelo_robo/              modelo do robo completo, garra de mastro, lista de materiais
sim/                      simulador da pista e das estrategias
docs/                     decisoes, estado atual, checklist de bancada, imagens
old/                  versoes descartadas (ombro SG90, MG996R direto, conves baixo) e pecas soltas do plotter
```

Comecar por `docs/2026-10-24_braco-opcao2-estado-atual.md` e `docs/2026-10-24_checklist-bancada.md`.
O plotter de desenho e outro projeto (`jhlr/plotter`). Os scripts Python de `cad/` rodam da raiz do repo (ex.: `python3 cad/braco/mdf_nest.py ombro`).
