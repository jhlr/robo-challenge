# SLAM simples aplicado ao seguidor de linha (2026-10-30)

## Conclusao

Nao precisa de SLAM completo. O mapa da arena e conhecido (`core/map.h`), entao o problema e **localizacao + um mapa de cubos incerto**, e o codigo ja tem os dois:

| Peca de SLAM | Ja existe | Onde |
|---|---|---|
| Localizacao (filtro de histograma / Markov, 6x5x4 estados) | sim | `core/localizer.h` |
| Mapa de ocupacao probabilistico (Bayes binario por rua + esquecimento) | sim | `core/belief.h` |
| Exploracao guiada pelo mapa | sim | `core/explore.h`, `navigator.h` |
| Odometria + giroscopio | helpers prontos, **nao ligados** | `core/odo.h`, `core/yaw.h` |
| Estimador continuo da posicao na linha | `readLine()` existe, o seguidor usa regras discretas S1..S8 | `core/line.h` x `seguidor/decisao.cpp` |

Descartados (peso demais, nenhum ganho com mapa conhecido): EKF-SLAM, FastSLAM / filtro de particulas, GMapping, Hector, scan matching/ICP, pose graph. O histograma de 120 estados e exato, sem amostragem, e cabe no ESP32.

## O que otimiza o codigo (em ordem de retorno)

1. **Odometria entre cruzamentos como medida no localizador.** Com os STS3215 (encoder absoluto 12 bit, `core/odo.h`) cada rua mede ~400 mm. `onCrossing()` hoje so tem `pMissCrossing = 0.05`; com a distancia percorrida o filtro separa "cruzamento perdido" (andou ~800 mm) de "fantasma" (andou ~100 mm) em vez de chutar. Tambem rejeita leitura de cruzamento falsa na hora.
2. **Cruzamento como marco que zera a deriva.** Em cada cruzamento confirmado: distancia ao longo da rua = 0, rumo = multiplo de 90 graus. Reseta o erro acumulado do giroscopio e da odometria (mesma ideia do "recalibrar a posicao ao cruzar uma linha" de robos VEX). O custo de deriva fica limitado a uma rua.
3. **Auto-calibracao do diametro da roda e da bitola** com o mesmo marco: distancia conhecida (passo 400 mm) / contagens entre dois cruzamentos. Ja foi o truque do angulo do cruzamento (`asin(10/37,5)`); aqui e o mesmo.
4. **Fusao de rumo: giroscopio (predicao) + diferenca das rodas (correcao), filtro complementar.** Erro de theta e o que mais acumula na odometria (relatorio CMU abaixo). Com velocidade em malha fechada nos STS, `assimetria.h` (compensar motor fraco com giroscopio) deixa de ser necessaria: remover em vez de portar.
5. **Estado continuo na linha: (desvio lateral e, erro de rumo psi).** Centroide da barra QRE-8D = medida de `e`; giroscopio = taxa de `psi`. Um Kalman de 2 estados (ou so PD sobre o centroide) substitui as faixas S3/S6, S2/S7 e o ciclo giro+avanco de 80 ms, que existem por causa dos TT sem encoder. Manter as regras de cruzamento (S1+S8) do usuario; mudar so o seguimento reto.
6. **Mapa conhecido como feedforward.** Sabendo a distancia ate o proximo cruzamento (odometria + localizador), perfil de velocidade: rapido no meio da rua, desacelera ~60 mm antes do cruzamento, em vez de `fatorLargura` reativo. Maior ganho de tempo de volta.
7. **Cubos em log-odds** (soma em vez de Bayes com divisao, sem saturar em 0/1) e escolha da proxima rua por ganho de informacao: `explore.h` ja ranqueia; trocar custo por (P(cubo) * valor) / (tempo da rota). Ganho pequeno, baixo risco.

## Ordem sugerida
(5)+(4) primeiro, porque dependem do porte para STS que ja esta na fila; depois (1)-(3) e (6). O simulador (`firmware/SeguidorRC/test/simulador.cpp`, `--lote N`) mede cada uma: 28/30 e 187 s sem ruido e o baseline.

## Estado (2026-10-30)
- Feito: `onCrossing(pose, mm)` + `refineWheelScale` em `core/localizer.h` (69 testes) e o sketch `SeguidorBasico` portado para os STS: `hal::StsDrive` no lugar da ponte H, odometria entre cruzamentos alimentando o localizador, assimetria removida do sketch. Builda limpo: `cd firmware && pio run -e seguidor`.
- Nao testado no robo. Faltam: reconciliar pinos dos sensores (13/14/18/19/21/22/23/25 do HANDOFF vs `pins.h`), sinais de roda (kLeftSign/kRightSign), `onTurn` do navegador (nao ligado), medir `kGearRatio`/`kTrackMm`.
- Peso morto a decidir: `ponte_h.*` e `assimetria.*` na biblioteca (o simulador ainda usa `assimetria.h`).

## Pontas soltas
- Artigo/ferramenta: "localizacao em grafo com odometria barata + marcos de cruzamento" e publicavel como relato de competicao; o diferencial e medir o ganho em lote no simulador.
- Nada aqui foi implementado nem medido.

## Fontes
- [Tese Debrecen: seguidor com IR + encoder + IMU, PID em dois lacos](https://dea.lib.unideb.hu/items/b9ca2691-f1ea-4c79-ae70-9a46e37a8609)
- [CMU MRSD: localizacao, EKF em cascata, odometria + IMU](https://mrsdprojects.ri.cmu.edu/2022teami/implementation/localization)
- [Encoder + giroscopio + acelerometro com Kalman (4 rodas)](https://www.thaiscience.info/journals/Article/CMUJ/10325060.pdf)
- [VEX Forum: recalibrar posicao ao cruzar linha](https://www.vexforum.com/t/position-tracking-2-encoder-or-3-encoder/62721/2)
- [Topological SLAM: survey](https://www.iit.comillas.edu/documentacion/revista/IIT-13-113A_preview/Topological%20simultaneous%20localization%20and%20mapping:%20a%20survey.pdf)
