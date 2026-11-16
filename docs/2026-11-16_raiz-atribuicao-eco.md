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

## Raiz (medida com pose real, 20 sementes)
Metodo: cada `ECO-CUBO` foi pareado com o `RAIO-POSE` da MESMA chamada (pose real e origem `ruaDe` no instante do eco). Primeiro pareamento estava contaminado por chamadas da varredura; corrigido.
- 1323 ecos pareados: 1305 corretos, 18 errados (1,4%).
- Odometria NAO e a causa: `andado` do firmware menos a distancia real desde a origem tem mediana 1 mm nos erros e 16 mm nos acertos.
- Os 18 erros sao todos ECO LONGO (usMm mediano 646 mm; acertos 301 mm).
- 9 dos 18 tem um cubo MARROM na rua do eco (obstaculo fisico, nao cubo).
- Os outros 9 nao tem objeto na lista: provavelmente o outro robo.

Causa: `observarRaio` marca qualquer eco longo como CUBO (`observeStreet(..., true)`), sem confirmacao de cor. A cor so e lida a ~5 cm (TCS da frente), entao o eco longo nunca e confirmado. Marrom e o outro robo entram como cubo.

Experimento (eco longo sem cor NAO vira cubo): 15/20 contra 17/20. Revertido. O eco longo e a principal fonte de descoberta de cubos, entao o problema nao se resolve removendo-o.

Proximo passo (nao testado): distinguir marrom e outro robo do cubo no eco longo. Isso precisa de dado de sensor, nao de regra de simulacao.
