# Carro no mapa: calibracao, gatilho do verde e ferramentas BLE (2026-11-09)

Primeira vez que a missao completa (`firmware/src/robo`, a mesma logica do simulador) rodou no carro real, sobre a arena. Este documento junta o que foi medido, o que mudou e o que falta.

## 1. Medidas feitas no carro

**TCS34725 do chao** (leitura `chao rgbc R G B C`, comando `s` pelo BLE), sobre cada cor da arena:

| Cor | R | G | B | C |
|---|---|---|---|---|
| Branco | 112 | 75 | 51 | 231 |
| Preto (linha) | 28 | 18 | 13 | 57 |
| Verde | 25 | 21 | 13 | 57 (outro ponto: 24 20 12 54) |
| Azul | 25 | 18 | 15 | 56 |
| Vermelho | 58 | 30 | 19 | 102 |
| Amarelo | 98 | 57 | 29 | 177 |

- Preto, verde e azul tem brilho parecido (~55) e tons proximos: a tabela antiga (outro sensor) nao separava o verde do preto, por isso o gatilho do verde nunca disparava.
- O branco varia de **180 a 231** de brilho conforme sombra, inclinacao do carro e posicao. Com o brilho pesando muito, um branco sombreado virava AMARELO ou VERMELHO (e o vermelho manda o carro dar meia-volta).
- Tabela em `firmware/src/core/color_table.h`. Classificacao **por proximidade e relativa**: tom (r, g, b como fracao da soma) pesa 4x; brilho entra como razao logaritmica do canal clear, peso 0,05. Esse peso foi escolhido testando 20 leituras reais (todas certas; com peso 0,5 quatro erravam). Sem rejeicao por ambiguidade; o filtro de duas leituras seguidas (`filtrarCor`) e a protecao contra ruido.
- O mesmo vale para o TCS da frente (cubos), mas a tabela foi medida no chao: conferir com um cubo de cada cor.

**Barra de linha QRE-8D** (comando `b`, tempo cru em us): branco ~500-570, preto passa de 2500 (satura). O limiar de 1000 us esta certo. **O verde da arena e escuro no infravermelho**: sobre ele os 8 sensores leem "preto". No modelo da logica (e do simulador) o verde e chao claro sem linha, entao `CarroReal::ler` mascara a barra como "sem linha" quando o TCS do chao aceita VERDE.

**Rodas** (comando `t`): as duas andaram 78 mm em 1,5 s a 30%, iguais e para frente. Sinais `kLeftSign/kRightSign` corretos. A bitola de 185 mm ainda e a medida com regua, nao a efetiva (env `bitola`).

## 2. Comportamento do carro

- **Inicio:** a partida comeca quando o chao le VERDE por 5 leituras seguidas (~0,5 s), limite de 12 minutos (regras); `g [s]` inicia a mao. `x` e o fim do tempo reiniciam o ESP **desarmado** (sobrevive em memoria RTC), para nao reiniciar sobre o verde: so `g` ou desligar e ligar arma o gatilho (`core/starttrigger.h`).
- **Ao ligar** o ombro vai para o topo (1981 ticks) e avisa se nao chegou.
- **Guarda da mao** (`core/handguard.h`): sonar a menos de 30 mm por 3 leituras faz o carro dar re de 5 cm; se o obstaculo veio junto, e uma mao segurando o carro no ar: para e pausa ate soltar (o tempo parado nao conta no limite). So vale com a garra no topo (ombro a ate 100 ticks de 1981), porque abaixada a garra fica na frente do sonar. A cor nao entra: sob a mao o TCS da frente leu clear 130-330, igual a parede branca.
- **TCS da frente** tem leve mau contato: so e lido quando o sonar pede, tenta ate 3 vezes e se reinicializa sozinho depois de falhar (`hal::ColorSensor::read`).
- **Pegar e largar** (`core/pickplace.h`, ver `2026-11-09_fluxo-pegar-cubo.md`): `coletar` e `soltar` rodam a maquina sobre o ombro real; a garra e STUB (`hal/claw.*`, `pins::kClaw = -1`; o GPIO 22 que estava no `pins.h` e o SCL do TCS do chao).

## 3. Resultado da primeira corrida na arena

Carro na faixa verde, olhando para o tabuleiro, `g 8`: saiu do verde, andou reto ~200 mm e **atravessou o portao azul** (`PORTAO azul atravessado em (0,0)`). Em seguida o chao branco oscilou para AMARELO/VERMELHO por causa do peso do brilho (corrigido depois, secao 1) e o carro girou procurando a linha. Corridas anteriores fora da zona verde nao servem: a logica assume que comeca no verde.

## 4. Ferramentas de observacao e controle

- **Console BLE:** o `robo` anuncia `RoboChallenge` (Nordic UART). Comandos: `g [s]`, `x`, `s` (estado, ombro, leitura do chao), `t` (teste de reta), `b` (tempo cru da barra), `h`. Durante a corrida imprime a cada 500 ms: barra (`#` = linha, ja com o verde mascarado), odometria de cada roda, rumo, comando das rodas, cruzamentos, plano atual; e cada troca da cor aceita do chao com a leitura crua (`[chao] A -> B rgbc ...`).
- **OTA por BLE** (carro parado, sem console conectado), segredo `robo-challenge-2026` em `src/robo/main.cpp`. O ESP aceita um aparelho por vez.
- **Ponte** `firmware/tools/robo_bridge.py`: fica conectada ao carro, guarda o log em `~/.cache/robo/ble.log` e aceita comandos por HTTP em `localhost:8765` (`/status`, `/log?desde=N&espera=S`, `POST /send`, `POST /ota` que grava `.pio/build/robo/firmware.bin`). Roda no terminal do app (a permissao de Bluetooth e dele; processos filhos do agente sao derrubados pelo macOS). **Servidor MCP** `firmware/tools/robo_mcp.py` (`robo_enviar`, `robo_ler`) sobre a ponte, registrado com `claude mcp add robo`.

## 5. Pendente

1. Repetir a corrida na zona verde com a tabela nova e ver se o carro segue a linha e conta cruzamentos; ler a telemetria.
2. Medir a bitola efetiva (env `bitola`, USB) e conferir os giros.
3. Conferir o TCS da frente com um cubo de cada cor; se o mau contato persistir, trocar o cabo.
4. Garra real: pino, angulos, pegada (hoje `holding()` e sempre true); `reposition()` ainda e so um registro.
5. CAD: `cad/braco/params.json` ainda tem `shoulder_ratio = 4`.
