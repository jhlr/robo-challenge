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

## Por que o cruzamento nao-esperado varre (fase 2)
Divisao do `nao-esperado` (420 casos em 20 sementes): 74% `ne-insegura` (a coluna ou a linha da crenca com massa < 0,8), 25% `ne-sem-trecho`, 1% `ne-odo-mais`, 0% `ne-odo-menos`. O odometro nao e a causa.

Causa (`core/localizer.h`): `onCrossing(loc)` sem distancia faz cada cruzamento ter 5% de chance de ter "pulado" dois quarteiroes, e essa massa nunca some dentro da cidade (nao ha evidencia interior que a distinga). A coluna fica em ~0,76 a 0,79, logo abaixo do limite de 0,8 de `kMinCoordBelief`, e o carro varre quase todo cruzamento por uma diferenca de 2%.

Teste: passar a distancia andada (`onCrossing(loc, mm)`, o modelo ja existia) tirou a varredura (insegura 22 -> 8 na semente 0), mas a semente 2 caiu de 5 para 1 entrega: o carro entrou num laco de meia-volta entre dois cruzamentos com crenca confiante e errada (girou 180 graus a cada cruzamento, de t=30 s em diante). Revertido. Proximo passo: fase 2b, o laco.

Evento de fim de beco: `core::onLineEnd` era chamado so nos testes; agora e chamado quando `classificarPerda` decide FimDeBeco. Efeito medido: nenhum (115/120 igual ao baseline, mesma entrega por semente). Mantido por ser a evidencia correta e nao mudar o resultado.

## Sementes 2, 5, 7, 14 (nao completam 6 cubos): causa e testes
Causa medida nos logs (`SIM_DEBUG`): a massa da coluna da crenca decai a cada cruzamento (modelo sem distancia), cai para ~78%, abaixo de `kMinCoordBelief` (0,8), e o planejador troca para "ancorar a linha": o carro vai a borda azul, a crenca volta a ~99%, mas o caminho de volta a faz decair de novo. Cada volta custa 60 a 150 s; o fim do ciclo e o pedido de ajuda (153 s sem meta). Em 4 de 4 sementes ha de 6 a 25 trocas para ancorar.

Testes (20 sementes, 6 cubos; baseline 115/120):
- Distancia andada no localizador (`onCrossing(loc, mm)`): 106/120. Menos varredura (insegura 312 -> 101) mas mais poses erradas (seed 0: 2 -> 10) e mais meias-voltas (20 -> 44). A odometria entre cruzamentos nao e confiavel o bastante para o localizador usar sem erro. Revertido.
- Limite de coluna 0,7 em vez de 0,8: 110/120 (sementes 0, 5, 8, 9, 11, 12, 18 pioram; so a 14 melhora). Revertido.

Conclusao: nenhuma das duas alavancas melhora. A causa que sobra e a odometria entre cruzamentos (erro de distancia por rua) e o planejador ancorar com a crenca na faixa 0,75-0,80. Proximo passo: medir o erro de distancia entre cruzamentos no simulador (rumo e odometria) antes de mexer no localizador.

## Erro de distancia entre cruzamentos (medido) e reanalise
Medido no simulador (`SIM_DEBUG`, PARADA): distancia andada pela odometria entre dois cruzamentos seguidos em linha reta, contra a distancia real entre as posicoes verdadeiras.
- seed 0: 93 pares, media +43 mm; seed 2: 81 pares, +47 mm; seed 5: 94 pares, +24 mm. Nenhum par encurtado acima de 20%; 4, 3 e 1 pares alongados acima de 25%.
- Ou seja: a odometria superestima ~10% (uma rua de 300 mm mede ~330). Dentro da banda de 12% que o localizador ja assume: o erro de escala NAO explica a crenca decaindo nem a piora com a distancia.

Reanalise: o problema observado (coluna em 78%, ancorar, borda) e uma consequencia. O defeito que aparece com a crenca CONFIANTE e o planejador trocando de alvo a cada cruzamento:
- seed 2, com distancia (confianca 100%/97%): entre (0,3) e (1,3) o carro faz 'coletar, seguir U' por ~60 s, sem nenhum evento de cubo. O alvo guloso (`decideAtCrossing`, `kKnownCube`) e refeito a cada cruzamento e aponta para o lado oposto ao que acabou de tomar.
- seed 7, sem distancia: em row 2, colunas 2 a 5, o mesmo padrao 'coletar U / ancorar' por 150 s.
Hipotese de trabalho (nao verificada ainda): o alvo nao tem compromisso (nao e mantido ate ser alcancado ou confirmado vazio) e o planejador nao verifica se o cubo mapeado ainda esta la quando passa pela rua. Proximo passo: identificar o cubo-alvo em cada volta (SIM_TRACE do alvo) e confirmar se ele existe na simulacao.

## Etapa 0 do plano de alvo: o alvo do cruzamento e fantasma (semente 7)
Instrumentacao (so SIM_DEBUG): `ALVO-CUBO` (rua que o planejador quer, crenca), `CUBO-INI` (disposicao real: ruas e cubos marrons).
Semente 7 (`--coleta 6`), 61 alvos escolhidos:
- 41 de 61 (67%) apontam para uma rua SEM cubo real. Ex.: (2,0)-(3,0) em 14 alvos, sem cubo nessa rua; o cubo real vizinho esta em (2,0)-(2,1).
- Cubos marrons (obstaculos, no centro dos quarteiroes): (0,0), (1,1), (0,2). O alvo (0,2)-(0,3) e vizinho de um marrom; o cubo marrom fica a 150 mm do eixo da rua, dentro do alcance lateral do sonar.
- Conclusao: a crenca de cubos mantem ruas ocupadas que nao tem cubo. A fonte mais provavel e a atribuicao de eco lateral (varredura/`observarRaio` com `deLado`) ou o eco de marrom, nao o eixo da rua.
- Ainda NAO separado: quanto do fantasma vem de marrom (cor 7 no TCS, eco lateral) e quanto de atribuicao de rua errada. Proximo passo do diagnostico: contar, para cada alvo fantasma, a origem da crenca (eco de cubo, eco de marrom, eco lateral).
Consequencia para o plano: a etapa 2 (limpar crenca fantasma) passa a ser prioridade, e tem que tratar o marrom como caso explicito (o TCS le cor 7, nao cubo).

## Etapa 2: crenca fantasma (semente 7)
Escritores da crenca de cubo (`cubos`):
- `observarRaio` com eco longe (`ECO-CUBO`): 90 eventos; quase todos em ruas com cubo real. Nao e a fonte dos fantasmas.
- `aoBloquearTemporario` (`setStreet 1.0`): ERA a fonte de um caminho errado. Chamado tambem quando um eco repetido NAO era cubo (4 leituras sem cor de cubo: marrom, parede, outro robo). Marcava a rua como cubo sem confirmacao.

Correcao (sem mudar o caso de cubo confirmado): `aoBloquearTemporario(bool cuboConfirmado)`. O eco sem cubo bloqueia a ROTA, mas nao marca cubo. Os tres sitios de chamada passaram o flag: eco sem cubo = false; cubo no caminho e falha ao coletar = true.

Resultado: 114/120 contra 115 (semente 2: 5 -> 4; demais iguais). Neutro nos numeros, mas removeu um escritor de cubo falso.

Fantasma que sobra (semente 7): rua (2,0)-(3,0) com 14 alvos e rua (0,0)-(1,0) com 6. Nao vem de eco longe nem de eco sem cubo. A hipotese: cubo confirmado pela cor no cruzamento (2,0) atribuido a rua por onde o carro passava, quando o cubo real esta na rua perpendicular (2,0)-(2,1). Proximo passo (etapa 2b): registrar, para cada escritor com cubo confirmado, a rua atribuida contra a rua real do cubo mais proximo.
