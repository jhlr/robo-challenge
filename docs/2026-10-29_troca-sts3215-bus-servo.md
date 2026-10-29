# Troca para 3x STS3215 + Waveshare Bus Servo Adapter (A) (2026-10-29)

## Decisao do usuario
- Chassi e motores novos: 3 STS3215. Esquerdo e direito, cada um acoplado a 2 rodas; o terceiro na torre (ombro).
- Placa: ESP32 DevKit comum (nao S3), `board = esp32dev` ja e o padrao do `platformio.ini`.
- Dimensoes finais mantidas (limite 300 x 300 mm estendido). O lado gordo e a dimensao lateral (a confirmar contra os 182 x 288 mm do doc 2026-10-21: lateral = largura entre rodas externas).

## Adaptador (fonte: wiki Waveshare Bus Servo Adapter (A))
- Jumper em **A** = controle por UART (ESP32/Arduino/STM32). **B** = USB.
- Ligacao documentada como RX-RX e TX-TX (literal, sem cruzar). Conferir com continuidade/esquematico: se o ping falhar, e a primeira coisa a trocar.
- Entrada 9 a 12,6 V (DC 5,5 x 2,1 ou borne), tem que casar com a tensao do servo. Ate 253 servos no mesmo barramento.
- O wiki nao diz nivel logico nem baud. Baud padrao dos STS: 1 Mbps (SCServo, `SMS_STS_1M = 0`). Nivel logico 3,3 V: confirmar no esquematico PDF antes de ligar.
- O adaptador faz a direcao do half-duplex sozinho: `kStsEchoes = false` esta certo.
- Exemplo oficial (Driver HAT, nao este adaptador): `Serial1.begin(1000000, SERIAL_8N1, 18, 19)`. Aqui a UART e a `Serial2` em 16/17 (`pins::kStsRx/kStsTx`).

## Protocolo (SCServo `SMS_STS.h`, conferido contra `hal/sts.cpp`)
Enderecos batem com o driver que ja existe: modo 33, torque 40, acc 41, posicao alvo 42-43, tempo 44-45, velocidade alvo 46-47, posicao atual 56-57. Pacote `FF FF id len inst params chk`, soma invertida; SYNC_WRITE `0x83`. Bit 15 = sinal (velocidade/posicao). Resolucao 4096 passos/volta.
Modo (reg 33): 0 = posicao, 1 = roda em malha fechada de velocidade, 2 = roda em PWM aberto.

## EEPROM (corrigido)
Conferido no SCServo: `Mode()` escreve o registrador 33 direto, sem unlock; so a troca de ID/baud exige `LOCK` (reg 55 = 0, escrever, reg 55 = 1). Antes eu tinha escrito que o modo precisava de unlock: errado. Implementado `sts::Bus::setId` (destrava no id antigo, escreve, trava no id novo).

## Implementado (2026-10-29)
- `hal/sts.*`: `setId`.
- `core/arm.h`: `shoulderTarget` (delta de graus reais a partir da posicao atual, 4096 passos/volta, recusa 0, >90 graus ou fora de 0-4095) no lugar de `spinDurationMs`; 2 testes novos, 63/63 no `pio test -e native`.
- `hal/shoulder.*`: ombro = STS id 3 em modo posicao, `shoulderBegin(bus)` nao move, `shoulderRotate(deg)` le a posicao, move e espera chegar (timeout 3 s).
- `params.h`: `kIdShoulder = 3`, `kShoulderUpSign = +1` (SUPOSTO, inverter se o primeiro movimento pequeno for ao contrario), velocidade 400 e acc 20; saiu todo o bloco do MG996R por PWM. `pins.h`: saiu `kShoulder`.
- `bringup`: `p` pinga 1-3, `i <velho> <novo>` troca ID, `d <deg>` move o ombro. Compila em `bringup`, `bringup_dc` e `follow`. NAO testado na bancada.

## Impacto no projeto
1. Ombro (feito, sem bancada): sai o MG996R de rotacao continua (sem feedback, calibrado a mao) e entra o STS3215 id 3 em modo posicao, com leitura real da posicao. Resolve a pendencia aberta do handoff 2026-10-28. `hal/shoulder.*` (PWM, ainda nao commitado) vira fina sobre `sts::Bus::moveTo`; `kShoulderRatio = 4` (z48/z12) continua valendo.
2. Rodas: `StsDrive` ja existe (modo roda, broadcast SYNC_WRITE, odometria pelo contador). Falta `kGearRatio` (servo -> roda) e `kTrackMm` medidos no chassi novo; `params.h` diz "looked 1:1".
3. Pinos: UART2 em 16/17. O pino 16 hoje esta emprestado ao ombro PWM (bancada): libera. GPIO12 (strapping) e a ligacao do STBY so existem no carro DC.
4. Seguidor de linha: com encoders (StsDrive) a odometria e o giro por contagem substituem a compensacao de assimetria por giroscopio (`seguidor/assimetria.h`, `ponte_h`) que era so do carro TT. Reavaliar no simulador depois de medir.
5. Alimentacao: servos a 9-12,6 V no adaptador; ESP32 a parte (5 V). GND comum.
6. Comando do ombro continua em deltas a partir da posicao atual (regra do usuario), agora lendo `readPosition`.

## Pendencias
- Ids 1, 2, 3 de fabrica = 1; gravar ids distintos um servo por vez no barramento.
- Esquematico do adaptador (nivel logico, silk TX/RX).
- Medir razao de engrenagem e bitola, tensao dos servos (7,4 V ou 12 V) e a fonte.
- Working tree tem edicoes de bancada nao commitadas (ombro PWM em `shoulder.*`, `params.h`, `pins.h`, bringup, sensors): nao misturar com esta troca.
