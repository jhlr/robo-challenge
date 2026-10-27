# Braço opção 2: estado atual (2026-10-24)

Consolida `2026-10-20_braco-opcao2.md`, `2026-10-21_chassi-superior-alargado.md` e `2026-10-22_ombro-mg996r.md` (esta última é uma decisão descartada, mantida como histórico). Onde houver conflito, vale este documento. Nada abaixo foi testado em hardware.

Eixos: x lateral, y para frente, z para cima; pivô das barras em (y = 0, z = 93). Fonte única dos números: `cad/braco/params.json` → `python3 cad/braco/cinematica.py` (relatório + `params_gen.scad`).

## 1. Mecanismo

1. **Braço:** paralelogramo fixo à frente do robô, sem giro (quem mira é o chassi). Barras de 100 mm (`L1`), 4 barras de MDF 2,3 mm, carrier vertical (a pinça sobe e desce nivelada). Pegar: -9,8°; erguer: -16°; soltar na plataforma: +25,5°.
2. **Ombro: SG90 ou MG90S (TowerPro) com redução 4:1.** Pinhão z12 no servo, setor z48 preso à barra de baixo do lado -x, módulo 1,25, 37,5 mm entre centros, servo atrás das torres a 165° do pivô. Fonte: `cad/braco/ombro_reducao.scad`. O MG996R direto no pivô foi modelado e descartado em 2026-10-22 (usuário: o TowerPro não aguenta direto); arquivos em `old/ombro_mg996r_direto/`.
3. **Pinhão parafusado no horn redondo do servo** (em vez do bolso estriado impresso): horn de 20 mm, disco de 2 mm, cubo de 3 mm virado para o servo (total 5 mm), plano para o pinhão. Flange de 22 x 2 mm no pinhão, 4 pinos impressos de 1,1 mm entram nos 4 furos do horn a 8 mm do centro, a 90°; M2 do eixo prende. O servo fica recuado (`sr_sv_top` = -27,15) e a prateleira do berço acaba antes do horn (`m_cx` = [-48,5; -27,55]).
4. **Extensor do eixo** (opcional): `cad/braco/stl/part_ext_eixo.stl`, Ø8 mm, 14 mm no total (10 mm de corpo, com 5 mm de estria fêmea de 21 dentes e 4 mm de macho). O modelo do ombro ainda não conta o extensor.
5. **Pinça rotativa:** SG90 de cabeça para baixo na mandíbula esquerda por `part_coupler.stl` (impresso e encaixado, servo preso); setores iguais (m1,5, 35 dentes) giram a outra mandíbula ao contrário; mandíbulas impressas com a engrenagem embaixo, dentes do lado do cubo cortados (gabarito `docs/2026-10-21_gabarito-corte-1a1.svg`); arco concêntrico ao cubo (raio 35,4 mm) que toca as 4 quinas; pino M3 x 30 por baixo no pivô da esquerda. Regra de MDF: só janelas fechadas, nunca entalhe aberto.

## 2. Chassi

1. **Deck de baixo (Esboço30):** 156 x 99 mm. **Convés de cima:** `cad/chassi/alargar_superior.py` gera `chassi_superior_alargado.dxf` (99 mm) e **`chassi_superior_asas.dxf`** (asas de 40 mm por lado, 179 mm; entalhes abertos até a borda nova). Modelo: `wing = 40` em `cad/modelo_robo/robot_completo.scad`.
2. **Largura total do robô: 182 mm** (as rodas externas ficam a 91 mm do centro; as asas chegam a 89,5 mm e não aumentam). Pinça aberta: 108 mm.
3. **Folga sobre as rodas: 2 mm** (convés a z = 68, topo das rodas a 66).
4. Eletrônica: ESP32 DevKit deitado na faixa de trás do convés; Mega/Uno não cabem ali.

## 3. Regra dos 300 x 300 mm (medida com tudo estendido)

| Dimensão | Valor | Limite | Folga |
|---|---|---|---|
| Comprimento (pegando, cubo preso) | 288 mm | 300 | **12 mm** |
| Largura total | 182 mm | 300 | 118 mm |

O comprimento é o limitante: o chassi não pode crescer para a frente ou para trás sem encurtar o braço. Esses números vêm do modelo, não do robô real.

## 4. Torque (estático, pior caso 2,1 kg.cm; com folga 2x, 4,2 kg.cm)

| Servo | Direto | Com 4:1 |
|---|---|---|
| MG996R (6 V) | 2,4x | 9,4x |
| MG90S (6 V) | 0,6x | 2,3x |
| SG90 (5 V) | 0,4x | 1,7x |
| STS3215 (7,4 V) | 4,6x | 18,4x |

## 5. Firmware (ESP32, PlatformIO)

1. 4 envs compilam (`bringup`, `follow`, `bringup_dc`, `follow_dc`); `pio test -e native`: 10 testes passam.
2. Ombro: `core::shoulderServoDeg` (`src/core/arm.h`, testado): servo = 109 - 4 x ângulo do braço; pegar 148°, erguer 173°, soltar 7° (clampado em 0 a 180). Sinal -1 e centro 109° valem com a engrenagem de redução montada com as barras horizontais.
3. Não há código Bluetooth: a regra do robô autônomo não exige nenhuma flag `COMPETITION` por enquanto.
4. Pinos e parâmetros: `firmware/include/pins.h` e `params.h` (tudo suposição até medir).

## 6. O que cortar e imprimir

- **MDF 2,3 mm (`~/Desktop/corte_mdf/`):** `sheet_ombro_1.dxf` (prato, 2 bochechas, prateleira, 4 barras), `sheet_torres_1.dxf` (2 torres, fundo, 2 placas laterais, frontal), `sheet_moldura_1.dxf` (prateleira, traseira, anteparo, ponte da pinça), `chassi_superior_asas.dxf`, `furo_15x10.dxf` (furo avulso). As antigas `sheet_1`/`sheet_2` estão obsoletas.
- **Impressão:** `part_sr_pinion.stl` (versão nova com flange e 4 pinos; imprimir flange e pinos para cima), `part_ext_eixo.stl` (opcional). Já impressos e que ficam: `part_sr_sector.stl`, `part_coupler.stl`, mandíbulas.

## 7. Pendências e riscos

1. Medir: diâmetro dos 4 furos do horn (pino de 1,1 mm), diâmetro do cubo do horn (suposto 8 mm), corpo do SG90 (22,5 x 12,2 x 22,7, eixo a 5,5 mm da ponta), espessura real do MDF (cupom `cupom_espessura.dxf` ainda não cortado; `mt` = 2,3).
2. Altura chão-deck (~25 mm) e vão do convés de cima (40 mm) ainda são estimativas (`z_deck_top` = 68 em `params.json`).
3. Rigidez: asas de 40 mm em balanço (MDF 2,3 mm); chapas do berço presas só por abas em janelas fechadas.
4. Parede da estria fêmea do extensor: ~1,5 mm (imprimir com 3 perímetros ou PETG).
5. Risco de tombar com o braço estendido não calculado; rodas STS na lateral engrenada: razão, bitola e velocidade por medir (amarelo é ~4x mais rápido que o STS direto, 0,18 m/s).
6. O modelo do convés de cima é uma caixa lisa de 127 mm de comprimento, enquanto o DXF tem 156 mm.
