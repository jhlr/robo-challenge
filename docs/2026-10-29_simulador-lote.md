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

## Atualizacao 2026-10-29 (tarde): mapa de rua de 40 cm e varredura parada das plataformas

- Mapa: passo 400 mm, becos 168 mm, ecoponto 291 mm, verde 371 mm (becos/ecoponto/verde: medidas da imagem na escala 400/302; so o passo veio de medida do usuario). Lote de 12 sementes sem ruido: 12 de 12 na meta, 259 s.
- Plataformas: o carro vai ao meio do mapa, chega a ~170 mm da face e varre -36..+36 graus com o ultrassom (Pitagoras: face = alcance x cos; centro de cada trecho com eco = distancia x tan do meio do trecho). A 250 mm o cone (+-10) fechava o vao de 100 mm. So a primeira entrega sobe ao topo (unica medida de velocidade em reta com distancia conhecida).
- Cubos: memoria do eco (um eco perto que some por causa do balanco do carro vale como "eco menos o que andou"; se o carro esta torto, realinha com a rua e le de novo). Toques em cubos: 2,0 -> 0,2 por partida.
- Varredura nos cruzamentos: com pose segura e cruzamento esperado, so gira para o lado cuja rua ainda nao foi medida (as duas medidas: nao gira; so uma: gira uma vez). 30 sementes: 25/30 na meta nos dois casos, 283 s contra 288 s.
- Falhas que restam (~5 em 30): o carro carregando fica preso num falso alarme no mesmo ponto (ex. semente 11, (1474, 230)); nao investigado.

## Rodada de otimizacao (30 sementes sem ruido)

| Mudanca | Na meta | Tempo medio |
|---|---|---|
| Antes (varredura so da rua nao medida) | 25/30 | 283 s |
| Rua bloqueada = a que o carro vai usar de fato (antes marcava sempre a do oeste); rota pelas ruas livres ate a coluna 0 | 26/30 | 277 s |
| ANCORAR com cubos nos dois sentidos da coluna: anda uma coluna ao leste | 27/30 | 266 s |
| Volta ao portao com a linha conhecida: `core::onOwnBlueAtRow` (pose segura logo na entrada, sem sweeps nem ANCORAR) | 27/30 | **207 s** |

Tentado e DESFEITO: (a) marcar toda plataforma lida como olhada (`yOlhados`): 23/30; (b) corrigir o rumo do giroscopio pelo plano da face das plataformas (ajuste do delta por minimos quadrados, fases 1 e 2): o delta sai certo (-22 contra -27 verdadeiro) mas o erro volta a crescer depois (vies do giroscopio + curvas fora das linhas), nao fechou o lote com ruido (0/12) e piorou o lote limpo (26/30). Com ruido o problema e ancorar o rumo ENQUANTO ha so parede (ecoponto e verde), sem linha para corrigir.
Falhas que restam (3/30): carro perde a linha ao reentrar pelo portao 40 mm abaixo da rua (deslocamento por tempo) e fica na faixa azul.
