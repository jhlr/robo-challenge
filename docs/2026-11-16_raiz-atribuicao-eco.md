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

## Limiar de eco longo (aplicado)
- A simulacao de coleta NAO tem adversario (`configurarMissao(..., oponente=false)`). Os ecos que eu atribuia ao outro robo eram marrons ou cubos; a minha chave de rua tambem ignorava a orientacao do marrom. Correcao da leitura anterior.
- Nosso sensor nao le cor a distancia (TCS so a ~5 cm). Por isso eco longo nao pode ser evidencia de cubo.
- `kEcoCuboAteMm = kPassoMm + kBecoMm` (427 mm): eco de cubo so conta ate a rua seguinte. Eco alem dele nao marca cubo nem vazio na rua do eco; as ruas antes dele continuam vazias (VAZIA-ECO-LONGE).

Resultado (20 sementes, 18 cubos):
- Completas: 17/20, igual a linha de base.
- Media nas completas: 491 s (antes 460 s). A maioria das sementes ficou mais lenta (ex.: seed 7 669 -> 683 s, seed 9 610 -> 682 s, seed 19 419 -> 498 s).
- Erro de atribuicao (ECO-CUBO sem cubo real): 2,9% (34 de 1175), medido sem pose.

Leitura: o limiar remove a crenca falsa de cubo, mas o carro perde a evidencia de cubos a distancia que ajudava a explorar. O ganho esperado em completas nao apareceu; o custo em tempo apareceu. Mantido porque e a regra correta para o sensor real; o proximo passo e a exploracao usar a ausencia de eco longo como informacao (hoje ela nao e).
