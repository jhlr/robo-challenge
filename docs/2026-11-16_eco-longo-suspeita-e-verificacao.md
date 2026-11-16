# Eco longo vira suspeita de cubo; o carro verifica a cor ao chegar

Data: 2026-11-16. Decisao do usuario: "vai ate o cubo quando achar; so da para checar a cor la".

## Mudanca
- `observarRaio`: eco longo (alem da rua seguinte) tambem marca a rua do eco como cubo SUSPEITO (`observeStreet(..., true)`), como antes do limiar de 427 mm. O planejador passa a ir ate la.
- Ao chegar (eco <= distancia de parar), se a cor na frente nao e cubo com plataforma (`idx < 0` ou `Nenhuma`), a rua do eco volta a vazia (`observeStreet(..., false)`). A cor continua sendo quem decide.
- `kEcoCuboAteMm` removido (sem uso).

## Medicao (20 sementes, 18 cubos, mesmo binario de recompilacao)
| versao | completas | media completas |
|---|---|---|
| limiar 427 mm (86ec09c) | 17/20 | 491 s |
| suspeita + verificacao (esta) | 17/20 | 460 s |

Mesmas sementes falham (1, 4, 11). Sementes mais rapidas na maioria; 0, 3, 5, 9 e 19 caem bastante; 13 sobe 449 -> 500 s.
Logs: `scratchpad` da sessao (`base/out.txt`, `exp/out.txt`), volateis.

## Leitura
Nao reverte a regra fisica: o eco longo nunca e prova de cubo, so pista. O custo de ir verificar sai menor do que o do limiar.
Nao testado: falsos cubos (marrom, outro robo) agora custam uma viagem cada; o simulador modela marrom como obstaculo, mas nao o outro robo nesta coleta.
