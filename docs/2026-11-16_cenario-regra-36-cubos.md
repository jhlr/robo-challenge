# Cenario de coleta: distribuicao da regra (18 cubos)

## O que estava errado
O `--coleta N` semeava so N cubos em 4 cores (azul, vermelho, amarelo, preto). A regra tem 36 cubos (5 de cada uma das 6 cores + 6 brancos, com marrons como obstaculo), e o tempo de referencia e pegar e entregar os PRIMEIROS N.

Consequencias das medidas anteriores (19/20 com 6 cubos):
- A arena tinha so 6 cubos, quase todos com plataforma. Isso escondia o problema de cubo sem plataforma.
- Seed 8 "nao alcancavel em 6/6" estava errado como conclusao: era consequencia do cenario, nao da regra.
- O gate de coleta (11/20) foi medido no cenario errado; precisa ser refeito.

## Correcao
`coletaDeN` (test/simulador.cpp) agora chama `semearRegras(18)`: 18 cubos sorteados da distribuicao da regra (5 de cada cor + 6 brancos). Foi pedido para nao colocar a arena toda (36), que nao cabe no board. A metrica segue: entregar N (6) cubos, em ate 12 min.

## Resultado (20 sementes, 18 cubos, metrica de 6 entregas)
| Firmware | Completas | Media nas completas |
|---|---|---|
| HEAD (antes da limpeza) | 17 / 20 | 458 s |
| Atual (com a limpeza do sweep) | 17 / 20 | 458 s |

Com 36 cubos o resultado era 10/20 (491 s); com 18 e 17/20. As duas versoes saem identicas, o que confirma que o sweep nao mudou comportamento.

## Proximo
- Refazer o teste do gate de coleta (so pegar cor com plataforma conhecida) neste cenario.
- Investigar as sementes que falham (ver saida `--coleta 6 20`), agora com 18 cubos.
