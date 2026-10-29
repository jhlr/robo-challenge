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

## Medida de 2026-10-29 (12 sementes, sem ruido)
- 6 de 12 chegam a meta; 1,4 entregas e 1,9 toques em cubos por partida; 415 s em media.
- Antes de corrigir a entrada na cidade depois da entrega (compensar a partida em deslocamentos curtos): 2 de 12.
- Com ruido aleatorio: 0 de 12 (ruido propositalmente alto, para ter ideia de mundo real).

## O que ainda falha (sem ruido)
- ANCORAR com a linha desconhecida: a pose assume a linha 0 e, com um cubo na rua para a moldura de cima, ele volta e volta a ela (toca no cubo varias vezes). O nucleo
  (`core::planAtCrossing`) ja manda para a outra moldura quando a rua esta bloqueada (teste `test_plan_anchors_the_other_way_when_the_street_to_anchor_is_blocked`), mas o
  simulador tem a propria logica de ANCORAR (Navegador::aoCruzamento) que ainda nao usa isso; ao testar la, um virar-de-sentido simples piorou (40 toques numa semente).
- Depois da 1a entrega, algumas sementes ficam presas (1 entrega em 601 s).
