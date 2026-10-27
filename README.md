# Robo Challenge 2026

Robo autonomo para o desafio (regras: 300 x 300 mm com tudo estendido, totalmente autonomo). Chassi com 2 conveses, braco paralelogramo em MDF com ombro 4:1 e garra rotativa, sensores de linha, cor e ultrassom.

| Pasta | O que tem |
|---|---|
| `firmware/` | Firmware ESP32 (PlatformIO): `bringup` (menu serial de todos os sensores e atuadores) e `follow` (seguidor de linha com juncoes), testes de `src/core` no PC |
| `seguidor_linha/` | Seguidor de linha simples do colega (ESP32-S3 + TB6612 + 8 sensores RC), corrigido; original no historico do git |
| `braco/` | Braco em OpenSCAD: parametros (`params.json`), cinematica, pecas de MDF 2D (DXF), STL das pecas impressas (pinhao, setor, estriado) |
| `garra/`, `garra_challenge/` | Pinca/garra e o modelo do robo completo |
| `chassi/` | Convés de cima alargado (script + DXF) |
| `sim/` | Simulador da pista e das estrategias |
| `docs/` | Decisoes, estado atual, checklist de bancada, imagens |
| `backup/` | Versoes descartadas (ombro SG90, MG996R direto, convés baixo) |

Comecar por `docs/2026-10-24_braco-opcao2-estado-atual.md` e `docs/2026-10-24_checklist-bancada.md`. O plotter de desenho e outro projeto (`jhlr/plotter`).
