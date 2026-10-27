# Plano do firmware (2026-10-20)

Estado: so planejamento. Nao existe codigo de firmware; so o simulador em `sim/`.

## Decisoes

1. Toolchain: PlatformIO (build por CLI, libs fixadas em `platformio.ini`, testes nativos da logica sem placa).
2. Arquitetura: scheduler nao bloqueante + maquina de estados hierarquica. Sensores lidos em taxas fixas, um tick por loop, comportamentos como estados. Espelha `sim/policies.py` (`sensor_first`).
3. Placa: **ESP32 recomendada** (decisao final pendente de confirmar o modelo exato).
   - Pro: 3 UARTs de hardware (barramento STS a 1 Mbps sem disputa), clock alto, depuracao por Bluetooth/WiFi (telemetria e ajuste de ganhos sem cabo).
   - Contra: logica 3,3 V. HC-SR04 precisa de divisor no ECHO (5 V -> 3,3 V). Servos SG90 aceitam sinal 3,3 V, alimentacao separada. TCS34725, VL53L0X/L1X e IMU funcionam em 3,3 V.
   - Mega 2560 fica de reserva (os docs antigos citam Mega no chassi antigo); o codigo isola o hardware em `hal/` para trocar.
4. Regra do desafio: robo totalmente autonomo, sem interferencia externa. BT/WiFi so para depuracao e **desligado por flag de compilacao** (`-DCOMPETITION`) na competicao.

## Estrutura proposta

```
firmware/
  platformio.ini
  src/main.cpp            # setup + tick do scheduler
  src/hal/                # um driver por periferico (STS bus, SG90, HC-SR04, IR array, TCS34725, IMU, ToF, buzzer/LED)
  src/core/               # scheduler, estado, belief map, politica sensor_first (sem Arduino.h: testavel no PC)
  src/behaviors/          # search, approach, pick, carry, place, signal
  src/debug/              # telemetria BT, so fora de COMPETITION
  test/                   # testes nativos (core/)
  config/params.h         # gerado de braco/params.json por script (nunca copiar a mao)
```

## Marcos

1. M1 bring-up: um sketch de teste por sensor/atuador, confirma pinos, IDs STS (1 a 3), leituras sanas.
2. M2 locomocao: 2 STS em modo roda (um por lado), seguidor de linha (array IR + PID), classificacao de cor do chao.
3. M3 braco: ombro STS3215 com presets de `braco/cinematica.py` (pega -9,8 graus, solta +25,5 graus), SG90 da garra, sequencia pegar/soltar.
4. M4 estrategia: portar `sensor_first` e o mapa de crencas do `sim/` para `core/`, validar contra o simulador.

## Pendente

1. Confirmar modelo da placa (ESP32 DevKit vs S3) e adaptador do barramento STS.
2. Pinagem final depende do chassi novo (medidas ainda nao fechadas).
3. Valores do `params.json` (altura do convés) ainda aproximados.

## Estado (2026-10-20, fim da sessao)

M1 e M2 escritos em `firmware/`. Compilam sem warnings (`pio run -e bringup -e follow`) e `pio test -e native` passa (8 testes da logica em `src/core/`). **Nada foi testado em hardware.**

- `pio run -e bringup -t upload`: menu serial 115200 (`h` ajuda): ping STS, mover servo, rodas, claw, dump de sensores, calibracao do IR, buzzer.
- `pio run -e follow -t upload`: seguidor de linha; em cruzamento avanca `kAxleToSensorMm`, le a cor do chao e vira conforme `kRoute` (S/L/R/U ciclico).
- Premissas a conferir (todas em `firmware/include/pins.h` e `params.h`): pinos, IMU = MPU6050 em 0x68, IR array analogico com 5 sensores, TCS34725 em Wire1 (conflito de endereco 0x29 com o VL53L0X), adaptador do barramento STS sem eco, `kTrackMm`, `kGearRatio`, `kAxleToSensorMm`, sinais das rodas, limiares de cor.
- Decisao de projeto: wheel mode nos 2 STS com odometria pelo encoder (posicao 12 bits, desembrulhada em `core/odo.h`); curvas de 90/180 graus por contagem de encoder, sem IMU.
- Falta: `-DCOMPETITION` ainda nao existe (so entra quando houver telemetria BT).

## Sensores (2026-10-21, decisoes)

1. **QRE8D** (8 saidas digitais 0/1): 8 GPIOs em `pins::kLine` (36, 39, 35, 32, 33, 19, 23, 4); `params::kLineIsHigh` define a polaridade. Alimentar em 3,3 V (saida nao pode passar de 3,3 V). Sem calibracao; `core/line.h` aceita ate 8 canais, posicao fica em passos de 2/7.
2. **2 TCS34725:** frente (junto ao ultrassom, acha cubo no chao a poucos cm) em `Wire` com o IMU; chao em `Wire1`. Mesmo endereco 0x29, por isso um por barramento. O sensor do anteparo da pinca sai do projeto (o recesso no `cr_frame` fica sem uso).
3. **VL53L0X removido do firmware:** tambem usa 0x29 e nao ha barramento livre. Se os ToF laterais voltarem, usar TCA9548A. O HC-SR04 cobre o sensor de distancia exigido.
4. SG90 da garra via LEDC direto (sem biblioteca); firmware sem warnings.
5. Correcao do estado acima: nada de IR analogico nem de calibracao `c` no bringup; build `bringup` e `follow` ok, testes nativos 8/8.

## Ombro com SG90 (2026-10-21)

O ombro agora e um SG90/MG90S com reducao 4:1 (ver `docs/2026-10-20_braco-opcao2.md`), nao STS3215. No firmware: `pins::kShoulder` (GPIO5, pino de strapping: pode dar um pulso no boot), `params::kShoulderRatio/Sign/CenterDeg`, comandos `o <deg>` e `a <deg do braco>` no bringup, classe `hal::Sg90` (garra e ombro). So as 2 rodas ficam no barramento STS (IDs 1 e 2). Build ok, sem warnings; nada testado em hardware.

## Dois carrinhos (2026-10-21)

Comparacao: A = motores amarelos TT (sem encoder); B = 2 STS3215 na armacao lateral engrenada ja construida (1 STS por lateral, centro -> 2 rodas). O app e o mesmo; muda a implementacao de `hal::Drive` (`src/hal/drive.h`):
1. `StsDrive`: modo roda, odometria pelo contador de 12 bits (`params::kGearRatio`, `kTrackMm`, sinais ja incluem a inversao das engrenagens); avanco de cruzamento e curvas por encoder.
2. `DcDrive`: 4 PWM (GPIO 16, 17, 2, 15) para uma ponte H de 2 entradas por canal (DRV8833, L9110S ou L298N com os jumpers EN ligados), 2 motores de cada lado em paralelo; zona morta `kDcMinDuty`; sem encoder: avanco por tempo (`kDcFullSpeedMmS`), curva por integracao do giroscopio do IMU.
3. Builds: `pio run -e follow` (STS) e `-e follow_dc` (amarelo); idem `bringup` / `bringup_dc`. Os 4 compilam sem warnings. No bringup, `v <l> <r>` agora vai de -1 a 1 e `e` imprime a odometria no STS (serve para calibrar `kGearRatio` e `kTrackMm`: andar um comprimento conhecido e girar 360 graus).
4. A medir: razao de engrenagens centro->roda (parece 1:1), bitola, velocidade do amarelo a 100%, e se o amarelo precisa de ponte H com mais corrente.
