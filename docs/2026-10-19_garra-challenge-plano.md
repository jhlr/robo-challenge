# Garra RCB Challenge 2026 — plano (2026-10-19)

Fonte das regras: `Regras_Challenge_2026_Portugues-V1.0.pdf` (nesta pasta).

## Restrições das regras que mandam no design

| # | Regra | Efeito na garra |
|---|-------|-----------------|
| 1 | Robô ≤ 300×300 mm, **medido com a garra totalmente estendida** | A garra precisa caber no envelope junto com o chassi |
| 2 | Cubo 50×50×50 mm, oco, liso, sem furo/ranhura, 25–40 g | Pegada só por atrito nas laterais ou por baixo |
| 3 | Máx. 2 cubos por viagem; entregar tudo antes de reentrar | Capacidade 1 ou 2 é decisão aberta |
| 4 | Plataforma 160×50 mm, 400–500 g, cabe até 5 cubos | 3 cubos em linha (150 mm); o 4º e o 5º só empilhados |
| 5 | Derrubar cubo já entregue: -45; cubo perdido: -10 | Soltar com precisão e sem bater nos cubos vizinhos |
| 6 | Sem câmera / LiDAR; outros sensores livres | Cor do cubo por sensor de cor ou IR |
| 7 | Cubos marrons e brancos não identificados: não mover | A garra não pode empurrar cubos ao se aproximar |

## Decisões tomadas

1. Pegada: pinça paralela, 1 servo (SG90 / MG90S).
2. Elevação: elevador linear. v0 era cremalheira + pinhão; a v1 troca por correia GT2 (mais leve).

3. Escopo de hoje: só a garra, como módulo independente. O carro (motores de tração) e o controlador ficam para depois; a garra expõe uma interface simples (ver abaixo).
4. Cubos por viagem: começar com 1; a pinça é projetada para poder crescer.
5. Atuador do elevador (confirmado): motor DC N20 com redução + 2 fins de curso (topo e base), agora com correia GT2.

## Interface do módulo (proposta)

- 1 servo da pinça: 5 V, 1 sinal PWM.
- 1 motor do elevador: 2 fios para um driver H-bridge (ex.: DRV8833), mais 2 fins de curso.
- Total: 1 PWM + 2 saídas de driver + 2 entradas digitais. Cabe em qualquer Uno, Nano ou ESP32.

## Decisões abertas

1. Sensor de cor do cubo. O sensor do chão do carro não lê o cubo. Falta um sensor na pinça ou à frente dela (ex.: TCS34725).
2. Confirmar o N20 + fins de curso como atuador do elevador.

## Dimensões de partida

- Abertura da pinça: ≥ 60 mm (cubo de 50 mm mais folga).
- Curso do elevador: 62 mm. A plataforma é um bloco de 50 mm de altura e o cubo é solto em cima dela, então o curso cobre a 1ª camada (plataforma + folga). Empilhar uma 2ª camada pede ~112 mm (`stroke` no .scad).
- Alcance além do chassi: o mínimo possível, contado nos 300 mm.
- Dedos com pad de borracha ou TPU para atrito (cubo leve, parede lisa).

## Reuso

A garra antiga está em `garra/` (STLs). É um braço articulado, então serve de referência de fixação no chassi e de servo, não de mecanismo.

## Carro existente (fotos de 2026-10-19)

- 170 mm (comprimento) x 160 mm (largura) x 120 mm (altura), Arduino Mega, driver L298N, 4 rodas, 2 sensores de linha na frente embaixo.
- Limite da regra: 300 x 300 mm com a garra estendida. Sobram 130 mm à frente do carro.
- Há 40 mm de altura entre os dois chassis: o N20 e a polia motora da garra cabem nesse vão. A base do mastro vai no chassi de baixo, na frente, e o resto do mastro sobe pela frente do chassi de cima.
- Chassi de baixo: topo a 16 mm do chão; o chassi inteiro pode ser furado, então a fixação é livre (furos M3 da base da garra são genéricos: furar o chassi para casar).
- Controlador: Mega, com pinos de sobra para 1 servo, 1 ponte H do N20 e 2 fins de curso.

## Modelo 3D v2, leve, com engrenagens (2026-10-19)

Fonte: `garra_challenge/garra_challenge.scad` (OpenSCAD, parametrico). STLs em `garra_challenge/stl/`; previa em `garra_challenge/preview_assembly.png`. A v1 (correia GT2) foi descartada a pedido; a impressora 3D da equipe cobre as engrenagens.

- Peso impresso (PLA maciço, sem infill): ver `mass.py`; a v0 pesava 196 g. O que mudou: mastro e carriage em moldura fina com janelas de alívio, mancais finos no lugar do bloco, dedos e sliders em moldura de 3 mm, módulo 1.25 no lugar de 1.5.
- Elevador: N20 na carriage, pinhão evolvente (m1.25, 12 dentes, 8 mm de largura) engrenado na cremalheira fixa no mastro; duas hastes de aço Ø4 x ~100 mm; curso 62 mm; 2 fins de curso.
- Pinça: servo SG90 no topo, horn de 2 pinos (jugo escocês) move dois sliders; cada dedo anda 7 mm (49 mm fechada, 63 mm aberta).
- Colisões: checadas no OpenSCAD em 5 poses; o engrenamento em 5 alturas. Sem interferência real; o servo só encosta no apoio traseiro (contato de face). Nada impresso nem testado com motor.
- Altura dos dedos: `deck_h + zc_low` do chão. `deck_h` era 16 mm no chassi antigo e precisa ser medido de novo no chassi novo.
- Pendente de teste de impressão: furo do spline do horn (4.8 mm), furo D do eixo do N20, backlash (`bl`), encaixe do N20 no bolso, folga das hastes (4.3 mm).
- Sensores modelados (v2.1, 68 g): suporte do ToF VL53L1X sob a base da pinça (olha para frente, entre os dedos); recesso para a placa TCS34725 no dedo A com janela para a face do cubo (~5 mm); furos na tampa do mastro para buzzer ativo de 12 mm e LED de 5 mm. Colisões: sem interferência real. Ajustar na impressão: bolso do ToF e recesso do TCS34725.
- Não modelado: linha de sensores IR, sensor de cor do chão e IMU (vão no chassi, que ainda não foi medido).

## Chassi novo (2026-10-19)

Lista de compras com fotos: `docs/2026-10-19_lista-de-materiais.pdf` (gerada por `garra_challenge/materiais_pdf.py`).


- Substitui o carro de 170 x 160 mm. Um servo de barramento STS3215 (Feetech) por lado, cada um ligado a 2 rodas.
- STS3215, da etiqueta: 16.5 kg.cm a 6 V e 19.5 kg.cm a 7.4 V; 0.238 s/60 graus a 6 V e 0.192 s/60 graus a 7.4 V (cerca de 52 rpm); corpo 45.22 x 24.7 x 35 mm; sinal: pacote digital (barramento serial).
- Já montado (fotos): placas de MDF em sanduíche, engrenagem central no servo e uma engrenagem por roda, 4 rodas com pneu. Falta medir o diâmetro da roda, o tamanho do chassi e a altura do chão ao topo do chassi.

## Ideias do usuário (ainda não decididas)

1. Percepção de distância: as regras exigem sensor de distância (obstáculo, outros robôs, cubos marrons). Candidatos: VL53L0X/VL53L1X (ToF, I2C) frontal na garra e laterais no carro.
2. Memória de posições com ML: as posições dos cubos (quadrados X/Y/Z, cinzas A-G) e das plataformas (5 slots por lado) são fixas no mapa; o que varia é quais estão ocupados. Isso é uma tabela/grade de ocupação, não precisa de ML. ML cabe em classificar a cor do cubo a partir do sensor de cor (kNN ou MLP pequeno treinado com leituras reais).

## Estratégia e simulador (2026-10-19)

- Decisões do usuário: o oponente empurra cubos com frequência (a geometria dos cubos não é fixa); sem ML (falta tempo); gostou do filtro de grade com mapa de crenças que decai.
- Arena: células de ~300 mm (recalculado da Figura 1; eu havia dito 365 mm antes, errado), 6x5 intersecoes, 49 ruas.
- Simulador em `sim/` (ver `sim/README.md`). Resultado com parâmetros não calibrados: memória bate o passeio aleatório (~1030 contra ~780 pontos); o decaimento da crença importa por causa do oponente (Planner 824 para ~1000-1020); carregar 2 cubos não ganhou do vizinho mais próximo; o erro de localização custa pouco com os valores supostos.
- Sensores recomendados para o robô: linha de IR (5 a 8 canais) com 2 sensores externos para cruzamento, sensor de cor do chão, IMU (BNO055 ou MPU6050), odometria dos STS3215, ToF frontal, sensor de cor do cubo na pinça, buzzer e LED (a regra exige sinal para cubo branco). Falta atualizar o PDF de compras com IMU, linha de IR, sensor de cor do chão, buzzer e LED.

## Sensor decide, memória só guia (2026-10-19)

- Decisão do usuário: a memória é só guia; os sensores são decisivos. No simulador isso virou o modo `sensor_first`: a rua lembrada como bloqueada só custa +40 s e o ToF confere antes de entrar. Custo: ~4% de pontos (dentro do ruído) e quase metade das aproximações desperdiçadas. Detalhes em `sim/README.md`.
- Ilustração do robô completo: `docs/2026-10-19_robo-completo.png` (gerada por `garra_challenge/robo_ilustracao.py` a partir de `robot_completo.scad`). Chassi, eletrônica e sensores estimados; a garra é o modelo real. Tamanho ~230 x 168 x 123 mm.
- Os sensores (linha de IR, sensor de cor do chão, IMU, buzzer, LED) já existem no inventário da liga; o PDF de compras não foi atualizado.

## Elevador com STS3215 (2026-10-19, substitui o N20)

- Decisão do usuário: o peso não incomoda, então o elevador usa um STS3215 (o terceiro) no lugar do N20. Saem o N20, os 2 fins de curso e o L298N; a carriage perde as prateleiras e o gatilho.
- Modelo: berço do STS3215 na carriage (lado esquerdo da cremalheira), pinhão m1.5 com 14 dentes (66 mm por volta, maior que o curso de 62 mm), `rod_y` 38 e `yc` derivado dele, mastro de ~120 mm. Peso impresso 88,5 g (PLA maciço, sem o servo), contra 65 g da versão com N20.
- Colisões: sem interferência real nas 5 poses. Os dois únicos pontos que aparecem (servo com a carriage e carriage com os sliders) têm espessura zero: contato de face.
- Suposições a conferir no servo real: o eixo de saída a 11,3 mm da ponta de baixo, a face de saída em x = 13, o furo central de 6 mm e os 4 furos M2 do pinhão (padrão provisório, medir o horn), e a fixação do corpo nas orelhas com M3.
- Firmware: homing por travamento na base com limite de torque (o STS3215 lê posição e carga), e limite de torque para não quebrar a cremalheira. Cabo de 3 fios (7,4 V, terra, dados) sobe com a carriage.
- Alcance e tamanho: garra a 145 mm da traseira do chassi estimado, robô ~230 x 168 x 145 mm (limite 300 x 300 mm).
