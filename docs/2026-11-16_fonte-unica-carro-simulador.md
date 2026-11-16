# Carro e simulador na mesma fonte (2026-11-16)

O simulador rodava a logica do carro com numeros proprios, diferentes dos do carro real. Isso escondia por que o carro se perdia. Agora os dois leem `firmware/include/params.h`
(via `seguidor/perfil_carro.h`: `ajustesDoCarro()` e `cruzamentoDoCarro()`), e o sensor morto usa uma funcao so (`preencherMortos`, em `decisao.h`).

## O que divergia (simulador antigo x carro real)
| Numero | Simulador antigo | Carro real |
|---|---|---|
| Barra a frente do eixo | 30 mm | 15 mm (`kAxleToSensorMm`, medido com regua em 2026-11-16; o chute inicial no params.h era 60) |
| Velocidade de reta (PWM 110) | ~254 mm/s | ~160 mm/s (`kStsFullSpeedMmS / kPwmFullSts`) |
| Giro da busca (sem MPU) | chute 110 graus/s | ~66 graus/s (`w = 2 v / bitola`) |
| Bitola | 140 supoe, 150 real | 185 efetiva |
| Zona morta | 65 | 0 (STS em malha fechada) |
| S7/S8 | lidos de verdade | mortos, copia do vizinho |
| Lacuna na fita | 120 ms (~30 mm) | agora 60 mm (distancia, nao tempo) |

## Causas de "se perde e nao volta" achadas no simulador
1. A busca mede o giro por TEMPO com `grausPorSegundoBusca`; o chute (110) era 1,7x o giro real, entao a "volta inteira" girava ~215 graus e a linha atras do carro nunca era vista.
   Sem MPU (o carro nao tem), a busca achava a linha em 4 de 12 casos no simulador antigo; com o valor da fisica e a barra a 60 mm (suposicao de entao) acha em 11 de 12.
   Com a barra medida (15 mm) a varredura de 360 graus alcanca so ~52 mm do eixo e a busca sem MPU acha 8 de 12 (falha nos de 60-100 mm de distancia), em 7 a 17 s.
2. A lacuna de 120 ms cobria ~19 mm no carro real: uma emenda de 60 mm virava busca, o carro girava e voltava pela linha que acabara de deixar, em laco (lacuna de 60 mm: NAO chegou em 600 s; agora chega).
3. `AjustesCruzamento::velMmS` (250) tambem era chute do carro antigo.

## Em aberto
- Busca longe da linha (60-100 mm) sem MPU: 8 de 12 no simulador, lenta (media ~8 s). Variar velocidade/trecho reto nao resolve (`velbusca`, `avancobusca`, `maxfator` testados): e o desenho da busca.
- Cenarios do simulador que ainda falham com os numeros reais: `reta off-25 ang+20`, `perdido de lado na linha` (com e sem MPU), `tabuleiro inteiro`; mais `rumo ancorado`/`assimetria` (dependem de MPU, que o carro nao tem).
- `teste_alvo.cpp` falha desde antes (`ruaDoEco(330, 0)`), sem relacao com isto.
- S7/S8 (GPIO35/34, so de entrada) seguem mortos: a barra RC precisa de pino com saida para carregar o capacitor. Mover para GPIO13/GPIO4 e zerar `kLineDeadMask`.

## Como rodar
- Simulador: `g++ -std=c++17 -O2 -Isrc -I../src -I../include -pthread test/simulador.cpp src/seguidor/decisao.cpp -o /tmp/sim && /tmp/sim [parametros]`;
  `SIM_TRACE=1` imprime uma linha a cada 50 ms (pose, barra, acao). `barra=30` muda a barra fisica E a que o firmware supoe; `mortos=0` barra saudavel; `encoder=0` TT amarelos.
- Teste do perfil: `g++ -std=c++17 -Wall -Isrc -I../include test/teste_perfil_carro.cpp src/seguidor/decisao.cpp -o /tmp/t && /tmp/t` (a parte da busca fica vermelha com `Ajustes{}` puro).
