# Raiz: atribuicao de eco de cubo a rua (medido nas 20 sementes, 18 cubos)

## Medicao
- Cada `ECO-CUBO` do firmware foi comparado com os cubos reais criados pelo simulador (`CUBO-INI`), com a rua normalizada (menor no primeiro).
- 1684 de 1762 leituras (95,6%) caem em rua com cubo real.
- 78 (4,4%) caem em rua sem cubo. Deslocamento na linha: +1 (26), +2 (8), -1 (7), -2 (4), sem cubo na linha (33).
- Os erros tem `andado` mediana 13 mm (contra 88 nas leituras certas) e `usMm` mediana 476 mm (contra 341).

## Leitura
- Erro concentrado em eco LONGE (~500 mm) logo depois de sair de um cruzamento.
- A atribuicao (`ruaDoEco`) usa `andado + usMm + 87` mod 300. Com eco longo, qualquer erro de `andado` (odometria) ou de origem (`ruaDe`) muda a rua.
- Nao verificado: se a origem `ruaDe` esta certa nesses casos (nao medi a pose real no instante do eco).

## Correcao anterior nao vale
- Trocar os bloqueios para a geometria do eco piorou para 14/20 (ver `2026-11-16_stall-150s-carregando.md`). Ela nao corrige este erro de 4,4%; so mudou a rua do bloqueio.
