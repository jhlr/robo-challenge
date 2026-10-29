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

## Update (2026-10-29, bench): official library adopted, servo not answering yet
- Rule 3.3 (do not reinvent the wheel): `hal/sts.{h,cpp}` removed; the firmware now uses the official SCServo (`SMS_STS`), vendored in `firmware/lib/SCServo` (MIT, see `VENDORED.txt`; unused `SCSCL` removed because it warns). `hal::beginStsBus` (Serial2, 1 Mbaud, pins 16/17) is the single owner of the bus setup; `StsDrive`, `shoulder.*`, `bringup` and `follow` call `SMS_STS` directly. ID change = the official ProgramEprom sequence (`unLockEprom`, `writeByte(SMS_STS_ID)`, `LockEprom`).
- Shoulder begin follows Waveshare's own `setMode(servo)`: unlock, max angle limit (reg 11) = 4095, mode 0, lock. A servo left in wheel/motor mode has limit 0 and would not move.
- Bench result: with the official library, ping of ids 0-253 at 1M/500k/250k/115200/57600/38400 baud and both wirings of GPIO16/17 got ZERO replies. So it is not the driver, the id or the baud: power (9-12.6 V on the adapter), jumper A, common GND, servo cable, or the adapter's logic level.
- The ESP32 also showed a serial flood of garbage (repeating 16-byte pattern) with the adapter wired: same symptom as the 2026-10-28 handoff (signal wire on the board corrupts UART0 / boot loop). Suspect: adapter signal lines idling at 5 V into 3.3 V GPIOs, or a power back-feed. To check with a multimeter before wiring again.

## Conflict found in `~/Downloads/HANDOFF.md` (line follower from another machine, STS3215 + same adapter, validated on a robot)
- That handoff says: jumper on **B** = UART control, A = USB. The Waveshare wiki text I fetched says the opposite (A = UART). The handoff is empirical (Ping, WheelMode, WriteSpe worked), so try **B** first.
- Same handoff wires it crossed: GPIO17 (TX2) -> adapter RXD, GPIO16 (RX2) <- adapter TXD, common GND, servos on an external 6-12 V supply (never from the ESP32). That is the wiring originally tried here, so the wiki's "RX-RX, TX-TX" advice was a detour.
- It also confirms: SCServo unmodified, `Serial2.begin(1000000, SERIAL_8N1, 16, 17)`, wheel mode (MODE=1) written with unlock/lock and checked with `ReadMode`, no `SyncWriteSpe` in the library (two `WriteSpe` calls), mode 3 (step) unreliable on this hardware.

## Adapter schematic read (Waveshare Bus Servo Adapter (A)-Sch.pdf, 2026-10-29) - follows the wiki, overrides the other-machine handoff
- Host UART header H2 = GND / RXD / TXD, 3.3 V logic (SN74LVC1G125/126 buffers on the 3V3 rail): safe for the ESP32, no level shifter needed.
- H4 jumper: A joins H2 RXD/TXD to U1RXD/U1TXD (UART control); B joins U1RXD/U1TXD to the CH343P (USB). So jumper A = UART, as the wiki says. The handoff's "B = UART" is wrong for this board.
- Wiring (wiki "RX-RX, TX-TX" is literal): ESP32 TX (GPIO17) -> adapter TXD, ESP32 RX (GPIO16) <- adapter RXD, GND-GND. The adapter's "TXD" is an INPUT (host transmit); it is not crossed. The handoff's crossed wiring is the wrong one.
- Direction is automatic: Q1 turns on when the host TX line goes low, which enables the bus driver (U3); with TX idle high the bus is routed to RXD (U4). The ESP32 TX must idle high (Serial2.begin does that); no echo of the host's own bytes returns.
- Servo ports H3/H5: pin 1 = DC_IN0 (through D2), pin 2 = DATA, pin 3 = GND. The servo cable order must match.
- Bench: with jumper A and TX->TXD, RX<-RXD, the official-library scan (ids 0-253, 6 bauds) still got no reply, and the servo LED blinks (alarm). Remaining suspects: servo cable order/seating at H3/H5, a servo in alarm/fault.

## RESOLVED on the bench (2026-10-29): servos answer
- Cause: a bad cable (replaced) plus the wiring. Working setup = wiki + schematic: jumper A, ESP32 GPIO17 (TX) -> adapter TXD, GPIO16 (RX) <- adapter RXD, GND-GND, official SCServo at 1 Mbaud on Serial2. The handoff's crossed wiring and jumper B were wrong for this board.
- Scan (ids 0-253, 6 bauds): ids 1 and 2 at 1 Mbaud. Ping/readback: id 1 pos ~1700, id 2 pos ~3630, both mode 1 (wheel), 12.0 V. Id 3 (shoulder) not on the bus yet.
- Next: connect the third STS3215 ALONE (factory id 1 would collide with the wheel servo 1), set its id to 3 (official ProgramEprom sequence), then the shoulder's first small delta (`d 5`), and the wheel direction signs.
