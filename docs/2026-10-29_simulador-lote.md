# Simulador em lote (multiplas sementes) — 2026-10-29

Uma partida so do simulador e caotica: uma mudanca boa pode piorar a partida que estava sendo olhada. Para medir, `--lote N` roda a missao so com o nosso robo em N sorteios
(cubos e ruido mudam com a semente) e `--lote-ruido N` o mesmo com o ruido aleatorio ligado.

```
cd firmware/SeguidorRC
g++ -std=c++17 -O2 -Isrc -I../src -pthread test/simulador.cpp src/seguidor/decisao.cpp -o /tmp/sim
/tmp/sim --lote 12            # sem ruido; SIM_SO=4 roda so a semente 4 (use com SIM_DEBUG=1 para ver os eventos)
/tmp/sim --lote-ruido 12
```

Cada linha: se chegou a meta (2 entregas), entregas, pontos, toques em cubos (regras: -5 cada) e tempo. O resumo traz a media.

## Medidas (12 sementes, sem ruido, `--lote 12`)
| momento | chegam a meta | toques em cubos / partida | tempo medio |
|---|---|---|---|
| antes de corrigir a entrada na cidade depois da entrega | 2 de 12 | 1,6 | 529 s |
| compensando a partida em deslocamentos curtos | 6 de 12 | 1,9 | 415 s |
| cubo perto do cruzamento: depois da meia-volta replaneja no cruzamento | 7 de 12 | 0,7 | 387 s |
| atravessa a faixa azul em frente ate achar a rua | 8 de 12 | 1,0 | 332 s |
| olhar adiante so para o lado da linha (nao para o quarteirao) | 8 de 12 | 0,8 | 321 s |

Com ruido aleatorio (`--lote-ruido 12`, ruido propositalmente alto): 0 de 12.

## O que ainda falha (sem ruido)
- 4 de 12 sementes terminam com 1 entrega em 601 s (presas depois da 1a entrega ou ancorando).
- Ancorar com a linha desconhecida assume a linha 0; a correcao do nucleo (`core::planAtCrossing` vai a outra moldura quando a rua esta bloqueada) ainda nao e usada pela
  logica propria de ANCORAR do simulador (`Navegador::aoCruzamento`). Virar o sentido ali simplesmente piorou, entao nao foi aplicado.
