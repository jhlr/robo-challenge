# Checklist de bancada (2026-10-24)

Ordem pensada para gastar o mínimo de material: cada bloco só começa depois que o anterior passou. Estado de partida: `2026-10-24_braco-opcao2-estado-atual.md`. Marque o resultado ao lado de cada item e me passe os números.

## A. Medidas (só paquímetro, 10 min)

1. Horn redondo: diâmetro dos 4 furos a 8 mm do centro (o pino impresso tem 1,1 mm; furo esperado ~1,3 mm), diâmetro do cubo central (suposto 8 mm). Resultado: ______
2. Corpo do SG90 (ombro): comprimento, largura, altura até o topo do eixo, distância do eixo à ponta (modelo: 22,5 x 12,2 x 22,7, eixo a 5,5 mm). Resultado: ______
3. MDF: espessura real em 3 pontos (modelo usa 2,3 mm). Se não for 2,3 ± 0,1, parar e me avisar antes de cortar. Resultado: ______
4. Chassi: altura do chão ao topo do deck de baixo e vão até o convés de cima (modelo: 25 + 40 + 3 = 68 mm) e topo real das rodas (modelo: 66 mm; folga sob o convés com asas: 2 mm). Resultado: ______
5. Rodas STS: razão do trem de engrenagens (centro → roda), bitola (centro a centro das rodas), velocidade do amarelo a 100% (modelo: kGearRatio 1, kTrackMm 100). Resultado: ______

## B. Peças pequenas (teste de encaixe antes de imprimir tudo)

1. **Pinhão novo** (`part_sr_pinion.stl`, flange e pinos para cima): os 4 pinos entram nos 4 furos do horn sem forçar, com o pinhão assentado no disco? Pino folgado: gota de cola; pino que não entra: aumentar o furo do horn com broca de 1,2 mm, nunca forçar.
2. **Extensor do eixo** (`part_ext_eixo.stl`, só se for usar): a fêmea entra no eixo do SG90 sem folga visível e o macho encaixa no horn? Imprimir com 3 perímetros.
3. **Cupom de MDF** (`cupom_espessura.dxf`): qual rasgo (de 2,0 a 2,6 mm) fica justo? Se não for o de 2,3 mm, me diga o número.

## C. Corte do MDF (nessa ordem)

1. `sheet_ombro_1.dxf` primeiro: confira que as abas da prateleira entram nos furos do prato com folga de laser (justo, sem martelar).
2. `sheet_torres_1.dxf`, `sheet_moldura_1.dxf`, `chassi_superior_asas.dxf`. Se o DXF das asas for o primeiro corte de um convés largo, cortar antes um retalho e ver o balanço.
3. `furo_15x10.dxf` só se ainda for preciso o furinho avulso.

## D. Montagem do ombro (4:1)

1. Servo no berço, com o horn montado e o pinhão parafusado no horn (M2 pelo centro).
2. Barras horizontais (alpha = 0), servo em **109°** (`o 109` no menu) e só então engrazar os dentes. Fora dessa fase os dentes batem ou pulam.
3. Rodar à mão de alpha -16° a +25,5°: nenhum atrito nas torres, no prato ou no carrier. Registrar onde raspa: ______

## E. Firmware (ESP32, USB)

1. Gravar: `pio run -e bringup -t upload` (STS) ou `-e bringup_dc` (carro amarelo). Abrir o monitor a 115200.
2. `h` lista os comandos. `s` mostra todos os sensores; `S` liga o fluxo de 10 Hz.
3. Fiação x `firmware/include/pins.h`: linha QRE8D (36 39 35 32 33 19 23 4), sonar (trig 25, echo 34 com divisor 5 V → 3,3 V), I2C frente (21/22) e chão (13/14), garra 18, ombro 5 (pino de strapping: ligar a alimentação do servo depois do boot), buzzer 26, LED 27.
4. `b`: o buzzer e o LED respondem. `p`: os 2 IDs do STS respondem (só no carro STS). Resultado: ______
5. `g 90`: a garra vai ao centro; varrer 0 a 180 em passos de 10 e anotar o ângulo em que as mandíbulas fecham no cubo e o ângulo de aberta (modelo: psi aberta 20°). Resultado: ______
6. Ombro: `a 0` (barras horizontais), depois `a -9.8`, `a -16`, `a 25.5` (equivalem a servo 148, 173 e 7 graus). **Se o braço andar ao contrário, inverter `kShoulderSign` em `params.h`.** Anotar se o `(CLAMPED)` aparece: ______

## F. Rodas e linha

1. `v 0.3 0.3`: o carro vai para frente? Senão, inverter `kLeftSign`/`kRightSign`. `x` para.
2. `v 0.3 -0.3`: gira no lugar. Medir o tempo de uma volta completa (para calibrar `kTrackMm` e o giro por giroscópio). Resultado: ______
3. Sensor de linha sobre a fita preta: `s` marca `#` nos canais certos? Se o preto vier como `.`, inverter `kLineIsHigh`.
4. Sensor de cor do chão: conferir as 4 cores da pista (azul, vermelho, verde, amarelo) e o branco; anotar os valores brutos. Resultado: ______

## G. Antes de ligar tudo

1. Alimentação: o servo do ombro e o da garra **não** saem do 3V3/5V do ESP32; usar fonte própria de 5 a 6 V com GND comum. Corrente de pico de um SG90 em stall: ~0,7 A.
2. Regra da competição: medir o robô montado, com o braço e a garra totalmente estendidos, dentro de 300 x 300 mm (modelo: 288 x 182 mm).
3. O robô deve ser totalmente autônomo: sem ninguém controlando por Bluetooth durante a prova. Hoje o firmware não tem Bluetooth.

## Me passe depois

Os números das seções A, E.5, E.6 e F.2 (e o que raspou em D.3). Com eles eu atualizo `params.json`, `params.h` e o modelo.
