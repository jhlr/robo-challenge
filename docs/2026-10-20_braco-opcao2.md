# Opção 2: braço paralelogramo com pinça rotativa (2026-10-20, revisado)

> **Parcialmente desatualizado.** O estado atual (ombro 4:1 com horn, chassi com asas, firmware, regra 300 x 300) esta em `2026-10-24_braco-opcao2-estado-atual.md`; onde houver conflito vale aquele.

Segunda opção de garra, ao lado da opção 1 (mastro com cremalheira). Poster: `docs/2026-10-20_braco-opcao2.png`; poses com o chassi novo: `docs/2026-10-20_braco-chassi-novo_pick.png` e `_place.png`.
Fonte: `cad/braco/` (`params.json` é a fonte única; `cinematica.py` calcula e gera `params_gen.scad`; `braco.scad` e `claw_rot.scad` são o modelo; `braco_ilustracao.py` gera o poster).

## Revisão (mesmo dia)

1. **Chassi novo medido:** deck 156 x 94 mm (`half_l` 78, `half_w` 47), roda 66 mm, altura chão-deck ~25 mm (aproximada, falta medir), vão até o convés de cima 40 mm.
2. **Sem giro:** o braço é fixo à frente do robô; quem mira é o chassi. O servo de yaw saiu.
3. **Mandíbula curva:** arco concêntrico ao cubo (raio 35,4 mm, meia diagonal), em vez de barra reta.
4. **Ultrassom HC-SR04** na frente do deck.

## Mecanismo (2 servos)

1. **Ombro:** STS3215 fixo no convés de cima (precisão), eixo horizontal, move 2 pares de barras paralelas (L1 = 100 mm). O carrier fica na vertical, então a pinça desce e sobe sempre nivelada.
2. **Pinça rotativa:** SG90 ou MG90S (de cabeça para baixo) aciona uma mandíbula; setores de engrenagem iguais (m1.5, 35 dentes) giram a outra no sentido oposto. Um anteparo segura a face de trás do cubo e fixa a posição. O sensor de cor do cubo fica atrás do anteparo.
3. **Mandíbula curva:** o arco interno toca as 4 quinas verticais do cubo (centra sozinho) e as pontas envolvem as quinas da frente (`claw.hook_dy` = 30 mm além do centro do cubo; ponta fechada em x = 18,7 mm). Com a barra reta e aperto a ~5 graus, só a quina de trás encostava e a ponta ficava ~5 mm solta. O contato é de linha nas quinas, não de face: se escorregar na impressão, usar pad de TPU/borracha nas quinas.

## Números (de `cinematica.py`)

- Eixo do ombro a 93 mm do chão; pegar: barras a -9,8 graus; soltar na plataforma: +25,5 graus.
- Cubo preso fica 73 a 82 mm além da frente do deck; comprimento total do robô até 288 mm (limite 300 mm).
- Largura: sem giro, a pinça aberta (psi 20 graus) tem meia largura 54 mm (108 mm no total), longe do limite de 300 mm.
- Torque no ombro 2,1 kg.cm estático, 4,2 com folga 2x: STS3215 (19,5) tem 4,6x; MG996R (10) tem 2,4x; SG90 e MG90S não servem no ombro.
- Aperto por mandíbula: 5,9 N com SG90 e 7,8 N com MG90S; o mínimo para segurar o cubo com margem 3x é 1,2 N.

## Verificações no modelo 3D (OpenSCAD)

- Engrenagens das mandíbulas: sem interferência em 0, 10 e 20 graus (checado antes da mandíbula curva; as engrenagens não mudaram).
- Braço contra o chassi novo (com o ultrassom): sem interseção em pegar e em soltar.
- Cubo com o braço: só contato plano com o anteparo (volume zero) em pegar; sem interseção em soltar.
- Mandíbula aberta: o cubo entra com folga larga. "Fechada" (psi = 0) apenas encosta nas quinas; o servo precisa apertar um pouco além disso (o setor de engrenagem permite, 135 a 225 graus).
- Altura mínima do braço no pegar: 17 mm do chão (conferir de novo com o `deck_h` real).

## Mudanças no resto do robô (modo `arm_mode` de `cad/modelo_robo/robot_completo.scad`)

- Convés de cima de y = -85 a 42; o Mega fica deitado no fundo; buzzer e LED ficam no convés.
- **Ultrassom HC-SR04 frontal** (placa 45 x 20 mm em pé, rente à frente do deck, transdutores ⌀16 mm saindo ~10 mm, a ~35 mm do chão), no lugar do ToF frontal. Detecta objetos e o oponente. Ponto cego: ao pegar o cubo, pinça e cubo tapam o feixe; na pose de soltar (pinça a 80 mm do chão) o feixe passa livre. Use-o para varrer a arena com o braço recolhido. Os ToF laterais continuam nos cantos.

## Suposições e pendências

1. Altura chão-deck (~25 mm) e vão do convés de cima (40 mm) são estimados; `z_deck_top` = 25 + 40 + 3 em `params.json`. Medir e rerodar `python3 cad/braco/cinematica.py`.
2. Convés de cima (±52 mm de largura, postes em x = ±45), rodas e engrenagens do chassi ainda são do desenho antigo; o deck novo tem ±47 mm, então ajustar.
3. STS3215: o eixo de saída a 11,3 mm da ponta do corpo é suposição; a fixação por orelhas M3 não foi modelada.
4. Acoplamento do SG90 (2026-10-20): sem horn. A mandíbula esquerda tem um cubo (d 7,6) que sobe pelo furo de 8 mm da ponte e um bolso estriado (21 dentes, 4,8 mm, 5 mm de fundo) recebe o eixo do SG90 de cabeça para baixo; M2 do horn sobe por baixo (rebaixo na face de baixo da mandíbula). Estria, saliência de 4 mm e ponte de 2,3 mm são premissas: imprimir antes `cad/braco/stl/part_hub_test.stl` (peça de teste de 12 mm) e conferir o encaixe. Pivô da mandíbula direita: parafuso M3 (já montado na foto).
5. Nada foi impresso nem testado em motor; o risco de tombar com o braço estendido não foi calculado.
6. Lista de compras da opção 2 (o PDF é da opção 1): 1 STS3215 do ombro + 1 SG90/MG90S da pinça + 2 STS3215 das rodas; HC-SR04; sem N20, fins de curso nem L298N.

## Protótipo em MDF encaixável (2026-10-20)

Versão em MDF 2,8 mm medido (cortadora 20 x 30 cm) de todo o braço **menos as mandíbulas, que continuam impressas** (`cad/braco/stl/claw_jaw_r.stl`, `claw_jaw_l.stl`). Fonte: `cad/braco/braco_mdf.scad` (peças 2D e montagem, `mt` = espessura real, `fit` = folga dos rasgos); `cad/braco/mdf_nest.py` gera DXF por peça e as chapas em `cad/braco/mdf/` (cópia em `~/Desktop/mdf_braco/`). Imagens: `docs/2026-10-20_braco-mdf_pick.png` e `_place.png`.

- **Peças (15 tipos, 22 cortes):** prato, 2 torres, fundo, 2 bochechas e prateleira do servo; 2 placas laterais e 1 frontal do carrier; prateleira, placa traseira, anteparo e ponte da moldura da pinça; 4 barras; cupom de teste. Todas as juntas são aba-rasgo; pivôs e parafusos em M3.
- **2 chapas de 300 x 200 mm** (a segunda quase vazia). O cupom (3 rasgos de mt, mt+0,1 e mt+0,2) serve para acertar `fit`/kerf com um retalho antes de cortar o resto.
- **Verificado no 3D:** sem colisão com o chassi nas poses pegar e soltar (só contato de face no convés), e o cubo só encosta no anteparo com a pinça aberta. Fechada, as mandíbulas tocam as quinas do cubo (como projetado).
- **Pendente:** espessura real do MDF (medir; todos os rasgos derivam de `mt`), kerf do laser, fixação do SG90 e do horn, e fixação do STS3215 (abraçadeira de nylon nas fendas do prato). Nada foi cortado ainda. Se a espessura mudar, rodar `python3 cad/braco/mdf_nest.py` depois de editar `mt`.

## Como os motores ligam nas pecas (2026-10-21, versao atual)

Correcao: uma versao anterior deste doc propunha pinhao + engrenagem 1:1 por eu ter lido os eixos de `box(x0,x1,y0,y1,z0,z1)` errado; foi descartada. Eixos do braco: x lateral, y para frente, z para cima; pivo das barras em (y = 0, z = z_s = 93).

1. [Em vigor: o MG996R direto foi testado no papel e descartado em 2026-10-22, ver `2026-10-22_ombro-mg996r.md`] **Ombro (decisao do usuario: SG90/MG90S no lugar do STS3215).** O torque do ombro e 2,1 kg.cm estatico (4,2 com folga 2x); SG90/MG90S dao 1,8 a 2,4 kg.cm, entao entra uma reducao **4:1** (modulo 1,25: pinhao z12 no servo, setor z48 preso a barra de baixo do lado -x; distancia entre centros 37,5 mm; MG90S fica em ~7,5 kg.cm no braco). Fonte: `cad/braco/ombro_reducao.scad` (reusa o dente e o bolso estriado da pinca). O servo fica atras das torres, direcao 165 graus a partir do pivo (a 180 graus a ponta baixa do setor batia no prato quando a barra sobe).
   - Pecas impressas: `stl/part_sr_pinion.stl` (bolso estriado de 21 dentes para o eixo do SG90, M2 pelo topo) e `stl/part_sr_sector.stl` (2 pinos de 3,2 mm entram em 2 furos de 3,2 mm da barra, a 6 e 12 mm do pivo; as 4 barras ja saem com os furos no DXF).
   - MDF refeito (`mdf/sheet_ombro_1.dxf`: prato com extensao de 42 mm para tras (y de -22 a -64), 2 bochechas, prateleira, 4 barras). Berco do SG90: prateleira + 2 bochechas com abas em janelas fechadas; o servo fica preso com cola quente/abracadeira (nao modelado).
   - Verificado no 3D (`chk_sr_*` em `braco_mdf.scad`): setor x pinhao sem interferencia de alpha = -20 a +28 graus (teste 2D que morde: com fase errada da HIT); sem colisao com o berco, torres, chassi e carrier em pegar, subir e soltar (so o servo apoiado na prateleira).
   - Montagem: servo em 109 graus e barras horizontais (alpha = 0) para engrazar; sentido invertido (1 engrenagem de reducao): `kShoulderSign = -1` no firmware. Angulos do braco -16 / -9,8 / +25,5 graus = servo 173 / 148 / 7 graus (dentro de 0 a 180, mas quase no limite).
   - Premissas a medir: medidas do corpo do SG90 (22,5 x 12,2 x 22,7, eixo a 5,5 mm da ponta), 4 mm de saliencia da estria, MG90S igual.
2. **Pinca (SG90 -> mandibula esquerda, sem reimprimir). Montagem real conferida na foto: as mandibulas estao com a ENGRENAGEM EMBAIXO, apoiada na prateleira** (como saem da impressao; o modelo anterior as tinha com a engrenagem em cima).
   - Com a engrenagem embaixo o setor (raio 27,75) bate no anteparo (y 10,7 a 13) e entra no cubo (y 13 a 24,7, z 0 a 4): checado no 3D. Solucao (decisao do usuario: manter embaixo): **cortar os dentes do lado do cubo** dos dois setores (nenhum ponto passa de y = 12,8 de psi = -1 a 20 graus; gabarito 1:1 em `docs/2026-10-21_gabarito-corte-1a1.svg`, vista em `docs/2026-10-21_corte-dentes-setor.png`) e **recortar o anteparo** para comecar em z = 4,5 (o setor passa por baixo, folga 0,5 mm). O corte nao toca a zona de engrenamento: raio de 3,5 mm em volta do ponto de contato intacto em todo psi; a 5 mm so a mandibula esquerda perde o dente de aproximacao, que so engrena com psi < -1 (nao usado).
   - Acoplador `stl/part_coupler.stl` (grampo): disco de 0,8 mm sobre o topo plano da mandibula esquerda (boss + barra, z = 35, 1 mm sob a ponte), dois dentes que descem de cada lado da barra de 4 mm (folga 0,15; travam a rotacao por contato), cubo de 7,6 mm pelo furo de 8 mm da ponte e bolso estriado passante para o eixo do SG90 (de cabeca para baixo, apoiado a z = 39: calco de 0,7 mm sobre a ponte de MDF). Os dentes param a 9,5 mm do pivo (a orelha do anteparo esta a 10,9 mm). Sem colisao com a moldura de psi = -1 a 22 graus; so contato plano com as mandibulas. Imprimir de cabeca para baixo com suporte so sob o disco.
   - Anteparo novo e prateleira sem as abas da frente: `mdf/sheet_moldura_1.dxf` (o anteparo agora fica preso so pelas orelhas na ponte; carga do cubo e minima).
   - Pivo de baixo da mandibula esquerda: aba nova na prateleira (x = -31,5 a -24) com furo de 3,4 mm para um M3 x 30 por baixo (a mandibula gira nele; a ponta do parafuso fica 7 mm abaixo do acoplador). Sem esse pino, o momento do cubo iria todo para o eixo do SG90.
3. **Movimento:** `python3 cad/braco/animar.py` gera `docs/2026-10-21_braco-movimento.mp4/.gif` (usa a versao MDF).
4. **Rodas:** cada STS3215 aciona 2 rodas por trem de engrenagens (fotos do chassi). Nao foi modelado: preciso das medidas/fotos do chassi.

## Conflito do berco do ombro com a placa de controle (2026-10-21)

O berco do servo do ombro (prato estendido + bochechas + prateleira) ocupa x = -53 a -18, y = -54 a -12 (global) sobre o convés de cima e colidia com a placa que o modelo antigo desenhava ali (Arduino Mega, 101,6 x 53,3 mm, y -83 a -29,7). A checagem anterior (`chk_sr_chassis`) so testava o servo e as engrenagens, nao o MDF do berco: agora `chk_sr_whole` testa o conjunto inteiro contra o chassi (so contato plano do prato no convés, z = 68). Solucao no modelo (`cad/modelo_robo/robot_completo.scad`): ESP32 DevKit (51,5 x 28,6, pinos macho para baixo: soquetes femea de 8,5 mm sobre o conves, PCB a 8,5 mm do conves) deitado na faixa livre de tras (y -84 a -55,4, x -10 a 41,5) e LED movido para (-30, -70). Atras do berco so cabe uma placa de ate ~29 mm de largura: um Arduino Mega ou Uno (53 mm) nao cabe ai; teria que ir no chassi de baixo.
