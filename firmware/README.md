# Firmware (ESP32 DevKit, PlatformIO)

Camadas: `src/core` (logica pura, testada no PC), `src/hal` (dono do hardware: rodas STS, ombro STS, sensores), `src/<app>` (apps finos). Pinos em `include/pins.h`, numeros em `include/params.h`.

## Apps (`src/`)
| env | o que faz |
|---|---|
| `robo` | missao completa no carro STS: a logica de `SeguidorRC/src/seguidor/robo.h` (a mesma do simulador) sobre `src/robo/carro_real.h`. O carro nao anda ao ligar: `g [s]` anda, `x` para, `s` estado |
| `seguidor`, `rapido` | seguidor de linha (BLE + OTA, ver `SeguidorRC/`) |
| `follow` | seguidor com juncoes (carro STS ou DC) |

`pio run -e <env> -t upload`. Variantes `*_dc` e `*_manual` existem para o carro DC (motores TT) e para placas sem reset automatico.

## Ferramentas de bancada (`test/bench/`)
Medem ou testam o hardware, nao fazem parte da missao.
| env | para que |
|---|---|
| `bringup` | menu serial: `p` ping STS, `v` rodas, `d <graus>` ombro por delta, `l u\|m\|p` ombro nas alturas topo / plataforma / pegar, `g <graus>` garra, `s` sensores |
| `scan` | ping nos ids 0..253: acha id duplicado ou barramento instavel |
| `tensao` | configura UM STS novo sozinho no barramento: `v` janela de tensao, `i` id 3 |
| `bitola` | mede a bitola efetiva das rodas |
| `sensores` | sonar + 2 TCS + MPU em tempo real |

## Testes
`pio test -e native` roda os testes de `src/core` no PC (`test/test_core`).

## Ombro e fluxo de pegar cubo
- `hal/shoulder`: STS3215 ligado direto (4096 ticks = 360 graus). Sempre em passos de 64 ticks a partir da posicao atual, checando posicao e corrente. Alturas em `params.h` (`kShoulderTicksUp/Pick/Place`).
- `core/pickplace.h`: maquina de estados do fluxo pegar-e-colocar, testada com um IO falso. Falta o IO real da garra: `docs/2026-11-09_fluxo-pegar-cubo.md`.
