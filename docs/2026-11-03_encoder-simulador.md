# Encoder no simulador (STS3215) — 2026-11-03

O seguidor foi desenhado sem encoder (distancia = tempo x velocidade, zona morta e assimetria calibradas). Os STS3215 tem encoder e velocidade em malha fechada.
`simulador.cpp` ganhou o modo `encoder=1` (padrao desligado, entao o lote antigo nao muda).

## O que o modo faz
- `Carro::odoMm()`: percurso medio das rodas lido do eixo (nao ve a patinagem no chao); `escalaodo=` simula erro de escala.
- Sem zona morta nem assimetria fixa entre lados (o servo regula), sem calibracao de assimetria, sem medir velocidade entre cruzamentos nem subir ate a moldura vermelha.
- `andarMm` anda pela distancia lida e comeca a frear a tempo (v x tau), esperando parar; `andadoDesdeSaida` e `andadoRuaMm` vem do odometro.
- `yPeloGiro`: parado de frente para a plataforma, gira -36..+36 graus, o meio do trecho com eco e o centro e `y = centro - distancia x tan(meio)` (Pitagoras). Erro medido: ~6 mm.

## Medidas (sem ruido, `--lote`)
| modo | sementes | na meta | tempo medio |
|---|---|---|---|
| padrao (sem encoder) | 30 / 60 | 28/30, 55/60 | 187 s / 210 s |
| `zona=0` (STS nao tem zona morta) | 30 | 29/30 | 183 s |
| `encoder=1` | 30 / 60 | **28/30, 56/60** | 212 s / 222 s |
| com ruido (`--lote-ruido 30`): padrao | 30 | 19/30 | 331 s |
| com ruido: `encoder=1` | 30 | 9/30 | 483 s |

Sem ruido o modo encoder empata com o padrao (e toca menos cubos: 0,3 contra 0,5 por partida), mas e ~15% mais lento (a varredura de y a cada entrega) e COM RUIDO ainda perde (9/30 contra 19/30): nao portar antes de entender o ruido.

## O que fechou o buraco (13/30 -> 28/30)
1. A reentrada pelo portao: o y vinha de "vi a plataforma" (+-25 mm). `yPeloGiro` acha o centro pelas bordas do eco (cone do HC-SR04 sai da ponta, 100 mm a frente do eixo: `borda()` corrige isso) e o erro de y cai para <= 11 mm.
2. `minMovMm()`: com encoder corrige deslocamentos de 4 mm, nao so de 20 mm (o limiar era da partida por tempo).
3. `girarAte`: a roda leva ~tau para parar, o giro que sobra (omega x tau, ~4 graus) deixava o rumo real errado; agora freia a tempo e espera parar. O vies do giroscopio NAO era o problema (estimado 1,199 contra real 1,200 graus/s).

## Proximos passos
1. Entender por que o ruido derruba o modo encoder (9/30 contra 19/30).
2. Saida pela rua do meio e varredura parada (ideia do usuario): tirar a travessia do verde e a subida a moldura.
3. Velocidade adaptativa: rapido quando a pose esta segura e o trecho livre, freando pela distancia restante ate o cruzamento.
4. Giro por angulo: odometria diferencial para estimar o vies do giroscopio em reta.
