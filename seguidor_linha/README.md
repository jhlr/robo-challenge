# Seguidor de linha preta (ESP32-S3 + TB6612FNG + 8 sensores RC)

Sem PID: decisao por grupos de sensores. Placa na Arduino IDE: "ESP32S3 Dev Module". Monitor Serial a 115200.
O primeiro commit e o codigo original do colega; o segundo e esta versao corrigida.

## O que estava fazendo o robo rodopiar
1. **Linha perdida = giro no lugar para sempre.** Sem nenhum sensor na linha, o robo girava a PWM 100 ate achar algo, sem limite de tempo. Agora: segue o ultimo comando por 120 ms (falha na fita), procura no ultimo lado a PWM 70 por ate 1,5 s e depois **para** (volta quando a linha reaparece).
2. **Ponta (S1/S8) dava giro no lugar com uma unica leitura.** Qualquer ruido na ponta virava um giro. Agora a ponta precisa de 2 leituras seguidas.
3. **Calibracao ruim passava batida.** Sensor que nao reagia era so ignorado e o robo andava assim (se o robo ficava parado na calibracao, todos eram ignorados e ele girava procurando a linha). Agora, com menos de 6 sensores validos a calibracao e recusada e repetida.
4. Com `ultimoLado == 0` e linha perdida o robo andava reto sem parar: agora procura/para igual aos outros casos.

## Otimizacoes
- Os 8 pinos sao lidos de uma vez no registrador `GPIO_IN_REG` (antes: `digitalRead` em laco duplo) e a medida para quando todos descarregam.
- Fora do `MODO_TESTE` a leitura espera so ate o maior limiar da calibracao (~metade do tempo de antes), entao o laco roda mais vezes por segundo.
- Sem `String` no laco (alocacao de heap a cada ciclo); `constexpr` nas constantes; PWM da ponte H a 20 kHz (core 3.x).

## Nao testado em hardware
Compila para `esp32-s3-devkitc-1` (core 3.3.12). Os tempos (lacuna 120 ms, busca 1,5 s, 2 confirmacoes) e as velocidades sao chutes razoaveis: ajustar na pista (`LACUNA_MS`, `BUSCA_MS`, `VEL_BUSCA`, `CONFIRMA_PONTA`). Em `MODO_TESTE = true` os motores ficam desligados e so o Monitor Serial mostra a decisao.
