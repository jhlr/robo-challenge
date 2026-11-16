# Sweep completo: estado de cada item

Verificacao final: `test_core` (Unity, 85 testes) passa, 8 `teste_*` passam, 20 sementes identicas ao HEAD (19/20).

## Feito
- **Cor**: um classificador so. `core/color_table.h` (medido no carro) e o dono. `Floor`, `classify` e os limiares de `core/color.h` foram removidos; `color.h` ficou com o HSV. `ColorSample` nao carrega mais `floor`. `follow/` e os dois bancos de bancada usam `classificarCor`.
- **Tabela de direcoes**: uma so, em `core/map.h` (`kDirs`). Antes eram 3 copias (`map.h`, `navigator.h`, `localizer.h`).
- **Codigo so de teste** removido: `decideDelivery` (o carro planeja a entrega em `seguidor/robo.h`), `countsForSpin` e `countsForDistance` (`odo.h`), `ruaTemCubo` (`alvo.h`). Os testes que os cobriam saem junto.
- **Ponte H antiga**: `seguidor/ponte_h.*` removidos do git (`git rm`; estao no historico) e o include em `SeguidorRC.h` saiu.
- **Dead code**: `kMaxSensors`, `velMinimaMmS`, include duplicado em `carro_real.h`.
- **Testes**: assert tautologico removido; `ultimaRuaVistaVazia` com assert exato; teste de classes de cor antigo removido (coberto por `test_color_table_*`).
- **Telemetria** do laco (500 ms, dirigindo) atras de `SEG_TELEMETRIA` (`include/params.h`), padrao 1 = comportamento igual. Pode ser desligada com `-DSEG_TELEMETRIA=0`.

## Garra e reposicionamento (resolvidos como suposicao explicita)
1. **Garra sem retorno**: `hal::Claw::holding()` agora devolve a constante nomeada `kPegadaAssumida` (true), com comentario dizendo que e suposicao, nao leitura. O log de cada coleta diz `PEGADA NAO VERIFICADA (garra sem retorno)`. Escolhi NAO falhar fechado: sem sensor, isso faria o carro nunca entregar. Quando houver sensor de garra, troca-se a constante pela leitura.
2. **`reposition()`**: agora retorna falso com o log `NAO IMPLEMENTADO`. Nao muda o comportamento atual: a etapa so roda depois de uma pegada falha, e a pegada assumida nunca falha. Implementar o reapontamento pelo sonar continua pendente e so da para validar no carro.

## Mantidos como estao
- `missao()`, `usAtivo()`, `consumirToques()` no carro: constantes que refletem o que o carro sabe (o juiz nao sinaliza toques).

## Nao mexi (decisao sua ou fora do carro)
- `rapido/` tem ganhos proprios (baseSpeed 0.30, kp 1.0) diferentes de `params.h` (kBaseSpeed 0.45, kKp 0.9). Mudar muda a bancada desse sketch.
- `SeguidorBasico.ino` (exemplo) sobrescreve `zonaMortaPwm` por fora de `configDoCarro()` e usa timeout de sonar 9000 us contra 8000 us no carro. Nao compila aqui (Arduino).
- `emptyPenalty` (`map.h`): valor 0, nao muda nada. Tirar o parametro muda a assinatura de todos os chamadores.
- `kLineDeadMask = 0` e `coleta.capacidade = 1`: configuracao de hardware/regra, nao gambiarra.
- `shoulderRotate` (so bancada) e `refineWheelScale` (usado pelo exemplo `.ino`): mantidos.
- `explore.h`: faixa 0.2..0.6 literal (= prior 0.4 +- 0.2). Trocar por derivacao exige incluir `belief.h`; valor igual.
- Prints de `follow/`, `rapido/` e `carro_real.h` (eventos) sem macro: sao eventos, nao telemetria de laco.
- Codigo do carro (`carro_real.h`, `robo/main.cpp`, `.ino`) nao foi compilado: falta o toolchain Arduino aqui. Mudancas ai sao so as de remocao de include e a macro de telemetria.
