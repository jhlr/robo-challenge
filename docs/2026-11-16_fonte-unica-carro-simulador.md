# Carro e simulador na mesma fonte (2026-11-16)

O simulador rodava a logica do carro com numeros proprios, diferentes dos do carro real. Isso escondia por que o carro se perdia. Agora os dois leem `firmware/include/params.h`
(via `seguidor/perfil_carro.h`: `ajustesDoCarro()` e `cruzamentoDoCarro()`), e o sensor morto usa uma funcao so (`preencherMortos`, em `decisao.h`).

## O que divergia (simulador antigo x carro real)
| Numero | Simulador antigo | Carro real |
|---|---|---|
| Barra a frente do eixo | 30 mm | 15 mm (`kAxleToSensorMm`, medido com regua em 2026-11-16; o chute inicial no params.h era 60) |
| Velocidade de reta (PWM 110) | ~254 mm/s | ~160 mm/s (`kStsFullSpeedMmS / kPwmFullSts`) |
| Giro da busca | chute 110 graus/s | ~66 graus/s (`w = 2 v / bitola`) |
| Bitola | 140 supoe, 150 real | 185 efetiva |
| Zona morta | 65 | 0 (STS em malha fechada) |
| S7/S8 | lidos de verdade | mortos, copia do vizinho |
| Lacuna na fita | 120 ms (~30 mm) | agora 60 mm (distancia, nao tempo) |

## Causas de "se perde e nao volta" achadas no simulador
1. A busca mede o giro por TEMPO com `grausPorSegundoBusca`; o chute (110) era 1,7x o giro real, entao a "volta inteira" girava ~215 graus e a linha atras do carro nunca era vista.
   A busca (o rumo vem dos encoders) achava a linha em 4 de 12 casos no simulador antigo; com o valor da fisica e a barra a 60 mm (suposicao de entao) acha em 11 de 12.
   Com a barra medida (15 mm) a varredura de 360 graus alcanca so ~52 mm do eixo e a busca acha 8 de 12 (falha nos de 60-100 mm de distancia), em 7 a 17 s.
2. A lacuna de 120 ms cobria ~19 mm no carro real: uma emenda de 60 mm virava busca, o carro girava e voltava pela linha que acabara de deixar, em laco (lacuna de 60 mm: NAO chegou em 600 s; agora chega).
3. `AjustesCruzamento::velMmS` (250) tambem era chute do carro antigo.

## Busca (rumo pelos encoders)
Conjunto de 40 casos (`--busca-lote`: 5 distancias da linha x 8 rumos). Antes: 19 de 40 achados. Agora: 40 de 40, ~5 s em media, pior ~13 s.
1. Linha so na ponta da barra (S1 ou S8 sozinho): o eixo esta mais longe da linha do que o giro alcanca (os sensores da ponta ficam a ~40 mm do eixo com a barra a 15 mm), entao girar no lugar so roçava a linha e o carro a perdia de novo. Agora avanca metade do ciclo (`fracaoGiroPonta` 0,5; era 0,95 de giro).
2. O quadrado em espiral crescia 1, 2, 3 x e depois repetia o mesmo quadrado; agora os lados crescem de dois em dois (1, 1, 2, 2, 3, 3), sem deixar faixa sem varrer.
3. A volta inteira de 360 graus (~6 s) virou 120 graus; roda de busca a 110 PWM e lado do quadrado de 600 ms (em `ajustesDoCarro`).

## Corrigido depois (barra a 15 mm: o braco do giro e curto, ~16 mm)
- O carro atravessava o portao, virava no 1o cruzamento e voltava pelo portao (portao contado 2x). Duas causas em `robo.h`: (1) `virarParaRua` parava na hora em que as pontas apagavam e ainda andava ~8 mm na freada (tau da roda x velocidade), depois somava o avanco ate o centro: o eixo girava 7 mm fora do centro. Agora mede a freada pelo odometro e anda (ou da re) so o que falta (`andarMm` aceita distancia negativa; `esperarParar` e a unica espera de freada). (2) `girarNaLinha` fechava na rua de ONDE VEIO (a barra a 15 mm a cobre ate ~45 graus): o limite para ignorar a rua de origem passou de 0,5 para 0,7 do giro.
- `reta off-25 ang+20`: a linha comeca fora do alcance da barra, entao o inicio e uma busca (nao seguimento), e o erro final assenta a meio passo de sensor (S4/S5 sozinho e a zona reta). O cenario so exige `ticksBuscando == 0` quando a barra ve a linha no inicio, e o erro aceito (`kErroFinalMm`, 8 mm) vem da quantizacao da barra (passo ~10,7 mm), nao de um numero ajustado ate passar.
- `teste_alvo.cpp` estava desatualizado desde o passo de rua de 40 cm (`kPassoMm` 400): os ecos agora saem de `kPassoMm`.
- O cenario do tabuleiro agora exige 1 portao, 3 cruzamentos, sem falso alarme e sem voltar (antes parava no 1o cruzamento).

## Em aberto
- S7/S8 (GPIO35/34, so de entrada) seguem mortos: a barra RC precisa de pino com saida para carregar o capacitor. Mover para GPIO13/GPIO4 e zerar `kLineDeadMask`.
- Nada disto foi gravado no carro (sem acesso ao carro; OTA/USB pendentes).

## Como rodar
- Simulador: `g++ -std=c++17 -O2 -Isrc -I../src -I../include -pthread test/simulador.cpp src/seguidor/decisao.cpp -o /tmp/sim && /tmp/sim [parametros]`;
  `SIM_TRACE=1` imprime uma linha a cada 50 ms (pose, barra, acao). `barra=30` muda a barra fisica E a que o firmware supoe; `mortos=0` barra saudavel; `encoder=0` TT amarelos.
- Teste do perfil: `g++ -std=c++17 -Wall -Isrc -I../include test/teste_perfil_carro.cpp src/seguidor/decisao.cpp -o /tmp/t && /tmp/t` (a parte da busca fica vermelha com `Ajustes{}` puro).

## MPU removido (2026-11-16)
O carro nao tem MPU: saiu todo o codigo dele (`inercial.h` com levantado/colisao/tracao/ancora na grade, `assimetria.h`, `calibracao.h`, a leitura do giroscopio em `robo.h`, o modelo de MPU do simulador e os cenarios so de MPU).
O rumo vem sempre dos encoders; `segurarRumoNaLacuna` ficou em `segurar_rumo.h`. `Robo::ligarImu` sumiu: o rumo se inicia no primeiro `tick`. Antes ela tambem esticava `lacunaMs` para 450; agora vale o do `ajustesDoCarro()`.
