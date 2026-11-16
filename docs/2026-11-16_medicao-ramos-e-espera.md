# Medicao por ramo e limite de espera sem meta (2026-11-16)

Baseline HEAD d4e22a2: 115/120 entregas em 20 sementes (6 cubos). Testes ok.

## Ramos do cruzamento (fase 1)
Contadores `Robo::contRamo` (`SIM_FASES=1` imprime a linha `ramos:`), 1960 decisoes em 20 sementes:

| ramo | total | share |
|---|---|---|
| pose-segura (gesto ou sem giro) | 1037 | 53% |
| varre-completa | 468 | 24% |
| nao-esperado | 420 | 21% |
| ancorar | 35 | 2% |
| vizinhas-medidas / so-um-lado | 0 | 0% |

Achados:
- Os ramos `vizinhas-medidas` e `so-um-lado` nunca rodam com a configuracao padrao: o ramo `pose-segura` (gesto > 0 com encoder) vem antes e pega todo cruzamento esperado. Codigo morto na pratica; decidir na fase 2 se remove ou se o gesto deve ser o que e medido.
- `nao-esperado` (21%) e a fonte das varreduras completas: o cruzamento nao bate com o odometro (tolerancia de 40 mm) e o carro gira 360 graus. Esse e o alvo da fase 2.

## Limite de espera sem meta (fase 4)
`maxSemMetaMs` (padrao 150 s, parametro `semmeta=` em segundos):

| semmeta | entregas (de 120) | sementes com pedido de ajuda | pedidos totais |
|---|---|---|---|
| 150 | 115 | 4 | 4 |
| 90 | 106 | 17 | 40 |
| 60 | 105 | 19 | 68 |
| 30 | 41 | 20 | 333 |

Conclusao: o limite de 150 s nao e gargalo. Reduzi-lo gera pedidos de ajuda falsos (o carro so esta lento, nao perdido) e custa entregas. Mantem-se 150 s. Nas sementes 2, 5, 7 e 14 o limite nao muda a falha (ajudas 4 a 5 contra 4 a 5), entao a perda e de localizacao, nao de espera.

Penalidade de reinicio (-15) continua fora da simulacao; com 150 s e 4 pedidos em 20 sementes o custo esperado e baixo.
