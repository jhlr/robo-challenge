# Sonar com periodo de 30 ms no simulador: varredura de novo

Data: 2026-11-17. O simulador respondia o sonar a cada tick (5 ms). O carro real (`carro_real.h`, `kPeriodoSonarMs`) devolve leitura nova a cada 30 ms. Medido antes: em 110 a passagem por um cubo tinha ~28 leituras no sim contra ~5 no carro. Mudanca: `Carro::us()` do simulador passa a cachear por 30 ms (igual ao carro).

Velocidade de varredura (`velbusca`), 20 sementes, 18 cubos, 6 entregas:

| velbusca | antes (5 ms) completas / media | com 30 ms completas / media |
|---|---|---|
| 70 | 9/20, 541 s | 12/20, 568 s |
| 110 | 17/20, 452 s | 15/20, 451 s |
| 150 | 13/20, 431 s | 15/20, 448 s |

Leitura: com o sonar real o ótimo de 110 deixa de ser claro. 110 e 150 empatam em completas (diferenca de 2 sementes, dentro do ruido de 20). 70 continua o pior. Com 5 ms a velocidade mais alta falhava mais; com 30 ms essa falha some. A sensibilidade do resultado ao periodo do sonar e o achado: a simulacao anterior superestimava a percepcao.

Nao medido: taxa de acerto da decisao por numero de leituras (ainda falta cruzar cada passagem com a decisao do firmware).
