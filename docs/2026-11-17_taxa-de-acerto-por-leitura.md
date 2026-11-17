# Taxa de acerto por numero de leituras (sonar a 30 ms, velbusca 110, 10 sementes)

Metodo (simulador, copia descartavel do HEAD, nada do repo alterado): cada passagem de um cubo pelo cone do sonar (leituras reais consecutivas, <= 60 ms entre elas) conta N leituras. Reconhecida = a crenca da rua do cubo passou de < 0,5 para >= 0,5 entre o inicio da passagem e 2 s depois. Passagens com crenca >= 0,5 antes sao ignoradas (ja sabia).

| leituras na passagem | passagens | reconhecidas |
|---|---|---|
| 1 | 244 | 4% |
| 2 | 190 | 8% |
| 3 | 182 | 12% |
| 4-5 | 644 | 3% |
| 6-9 | 362 | 2% |
| 10+ | 135 | 24% |

(Passagens com 678 casos "ja sabia" ficaram fora.)

Leitura: a taxa quase nao sobe com o numero de leituras. Na maioria das passagens multi-leitura a crenca fica no prior (0,4) do comeco ao fim: o firmware nao consome essas leituras. Causas no codigo:
- Driving: `observarRaio` com o eco so roda a cada 300 ms (`ultimaObsRaioMs`, robo.h ~1426). Com leitura a 30 ms, ~10 leituras viram 1 observacao.
- Cruzamento: `observarRaio` usa uma leitura por direcao no momento da decisao (robo.h ~264).
Ou seja: o gargalo esta em quantas leituras a logica transforma em evidencia, nao so no sensor.

Limites da medida: a janela de 2 s e a associacao passagem -> rua sao aproximadas; 10 sementes.

## Teste: crenca consome cada leitura (cadencia de observacao 30 ms em vez de 300 ms)

Velbusca 110, 20 sementes, sonar a 30 ms:
- Antes (300 ms): 15/20 completas, media 451 s. Falhas: 1, 4, 5, 11, 17.
- Depois (30 ms): 14/20 completas, media 454 s. Falhas: 1, 4, 5, 6, 11, 17.

Resultado: nao melhora. A semente 6 passa de completa para falha; o resto empata. Consumir cada leitura nao aumenta o acerto na prática com a logica atual. Revertido (repo em d88e51f). A causa das leituras nao usadas pode estar em como a observacao e feita (uma direcao por vez, eco so na rua de origem), nao so na cadencia.

## Teste: leituras ambiguas valem para as duas ruas candidatas

Diagnostico: 45% das leituras de cubo caem em `ruaDoEco` = -1 (resto fora de [72, 230] mm da rua de 300 mm), entao sao descartadas (medido com andado = 0; nas passagens de andar o resto pode mudar).

Variante: leitura ambigua faz `observeStreet(..., true)` nas duas ruas candidatas, com a mesma evidencia.

Velbusca 110, 20 sementes:
- Base: 15/20, media 451 s. Falhas: 1, 4, 5, 11, 17.
- Variante: 15/20, media 456 s. Falhas: 1, 4, 5, 9, 17.

Resultado: empata. Usar as leituras ambiguas nao muda o desfecho. O gargalo parece nao ser o descarte. Revertido (repo em e873534 + esta doc).
