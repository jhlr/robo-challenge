# Sweep dos outros arquivos (fora de robo.h)

## Feito nesta rodada (sem mudar comportamento)
- Removido codigo morto confirmado por grep: `kMaxSensors` (`core/line.h`), `velMinimaMmS` (`seguidor/alvo.h`), `#include "params.h"` duplicado (`robo/carro_real.h`).
- Removido `assert(!c.estranho || true)` (sempre verdadeiro, nao testava nada) em `teste_decisao.cpp`.
- Assert fraco em `teste_alvo.cpp` (aceitava -1 ou 0) virou `== 0`, que e o valor real.
- Verificacao: 8 testes `teste_*` passam; 20 sementes identicas ao HEAD.

## Decidir (gambiarra real ou duplicacao: nao mexi)
1. **Stubs que mentem para a logica**: `claw.holding()` sempre `true` (a pinca nunca falha no carro); `carro_real.reposition()` sempre `true` ("PENDENTE"); `missao()`/`usAtivo()` sempre `true`; `consumirToques()` sempre 0. O codigo de decisao roda como se as leituras existissem. Precisa implementar ou tirar o caminho.
2. **Classificadores de cor duplicados**: `core/color.h` e `core/color_table.h` usam enums e limiares diferentes para o mesmo sensor (o proprio `color.h` admite a divergencia). O `follow/` usa um, o `robo/` outro.
3. **Tabelas duplicadas**: a tabela de 4 direcoes aparece 3 vezes (`map.h`, `navigator.h`, `localizer.h`); 2 enums de altura de ombro (`core::Level`, `hal::ShoulderLevel`) mapeados a mao; `countsForSpin` e `spinDeg` com a mesma regra.
4. **Constantes de geometria fora de `params.h`**: rua de 400 mm em `localizer.h:40`; faixa 0.2..0.6 em `explore.h:52` (deveria vir do prior); `kAlcanceCorFrenteMm = 110` usada para 3 coisas diferentes (`carro_real.h`).
5. **Parametros de A/B ainda vivos**: `rapido/main.cpp` (`signLeft`, `signRight`, `mirror`) muda por comando BLE; `map.h` `emptyPenalty` sem nenhum chamador de producao; `kLineDeadMask = 0` (mecanismo nunca ativo no carro); `coleta.capacidade` fixo em 1 no carro e o simulador varre 1..2.
6. **Codigo nao usado por producao, so por teste**: `decideDelivery()` (`navigator.h`), `countsForDistance`/`countsForSpin` (`odo.h`), `refineWheelScale` (`localizer.h`), `ruaTemCubo` (`alvo.h`, so teste), `shoulderRotate` (so bancada). Remover exige mexer nos testes que os cobrem.
7. **Ponte H antiga**: `seguidor/ponte_h.*` nao tem chamador, mas `SeguidorRC.h` ainda inclui. Remover o arquivo e a inclusao e uma delecao de arquivo rastreado: confirme antes.
8. **Prints de debug sem macro de compilacao** em `follow/`, `rapido/`, `robo/main.cpp`, `carro_real.h` (telemetria a cada 500 ms enquanto dirige; `rapido` imprime a cada 250 ms em producao).
9. **Dois configs paralelos**: `rapido/main.cpp` tem velocidades e ganhos proprios (baseSpeed 0.30, kp 1.0) que diferem de `params.h` (kBaseSpeed 0.45, kKp 0.9), embora `params.h` diga ser a fonte unica. O `.ino` de exemplo sobrescreve `zonaMortaPwm` por fora de `configDoCarro()`.
10. **Sonar**: timeout 9000 us no `.ino`, 8000 us no `carro_real.h`.

## Fora de escopo desta rodada
- `firmware/lib/SCServo` (terceiros).
- `firmware/test/test_core/test_core.cpp`: usa Unity, nao compilado aqui. Ainda hard-codes de altura do ombro que duplicam `params.h`.
