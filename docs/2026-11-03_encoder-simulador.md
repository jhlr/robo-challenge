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

## Cruzamento com rodadinha (`gesto=`) e varredura continua do y (2026-11-03, noite)
`gesto=45` (padrao no modo encoder): com pose segura e o odometro conferindo o cruzamento (400 mm +-40), em vez da varredura de 90 graus da uma rodadinha de 45 graus para o lado menos explorado e volta reto; `gesto=0` volta a varredura antiga; `gesto=-1` passa direto sem girar.
`yPeloGiro` agora varre CONTINUO (gira de -36 a +36 lendo o ultrassom a cada ~30 ms, rotulado pelo giroscopio) em vez de 19 giros com parada: erro de y <= 8 mm e ~10 s a menos por entrega.

| modo (encoder=1, sem ruido, 60 sementes) | na meta | tempo medio |
|---|---|---|
| `gesto=0` (varredura antiga) | 57/60 | 212 s |
| `gesto=-1` (passa direto) | 56/60 | 216 s |
| `gesto=45` | 55/60 | 244 s (antes da varredura continua) |
| padrao (sem encoder) | 55/60 | 210 s |

Conclusao: o tempo parado nos cruzamentos NAO e o gargalo (o total quase nao muda entre as tres). O perfil de uma partida: ~30 s por entrega (16 s procurando a plataforma, 14 s ate o portao), mais ~45 s na primeira ida (20 s contando cruzamentos ate a linha vermelha). Com ruido (`--lote-ruido 30`) o modo encoder continua em 10 a 11/30 contra 19/30 do padrao, e nenhuma fonte de ruido sozinha explica (desligar uma por vez da 9 a 13/30): os fracassos sao o carro perdido (pose (0,4) com o carro em x=1374 junto da borda, ou preso na borda leste x=3003) sem recuperar.

## Proximos passos
1. Entender por que o ruido derruba o modo encoder (9/30 contra 19/30).
2. Saida pela rua do meio e varredura parada (ideia do usuario): tirar a travessia do verde e a subida a moldura.
3. Velocidade adaptativa: rapido quando a pose esta segura e o trecho livre, freando pela distancia restante ate o cruzamento.
4. Giro por angulo: odometria diferencial para estimar o vies do giroscopio em reta.

## Rumo derivado na reentrada (2026-11-03, noite)
Causa de uma das falhas com ruido (semente 0): o rumo estimado derivou ~22 graus (solavanco + vies do giroscopio), o carro chegou torto na plataforma e `yPeloGiro` (que assume a face perpendicular) errou o y em 54 mm; reentrou fora da rua e ficou ~180 s perdido. O perfil de alcance da varredura (116 a 172 mm) entregava a inclinacao: `r(phi) = d / cos(phi - delta)`.
`yPeloGiro` agora ajusta `delta` e `d` na propria varredura; se `|delta|` passa de 10 graus (zona morta: o ajuste tem vies de ate ~8 graus com o rumo certo, por causa do cone e dos 3 raios do `us()`), reancora o rumo na face (multiplo de 90), gira para ela e mede o y no referencial da face.
Resultado: sem ruido `--lote 30` 28/30 -> 29/30 (199 s contra 212 s); com ruido `--lote-ruido 30` ficou em 10/30 (sem ganho: o gargalo com ruido e outro, o carro perdido por outras causas). Um modelo do cone dentro do ajuste foi tentado e piorou (7/30): era sobreajuste aos 3 raios do simulador.

## Sem MPU: rumo pelos encoders e giro pela linha (2026-11-03, noite)
O MPU foi retirado do carro. O simulador ganhou `rumoenc=1` (o rumo vem de `(dD - dE) / bitola`, dos encoders das rodas, e a bitola que o firmware supoe e `bitolaaj=`) e o giro de rua fechado na linha (`girarNaLinha`, so com `rumoenc=1`).
O giro na linha: o eixo avanca 17,5 mm (pivota no centro do cruzamento; as pontas apagam com o eixo ~17,5 mm antes dele), gira direto pela faixa em que os sensores do centro veem a rua nova (+-24 graus em volta da direcao dela), marca o angulo contado na entrada e na saida, volta ao MEIO e o rumo vira o multiplo de 90 certo. Erro de giro real medido: 1 a 4 graus mesmo com a bitola errada.

| rumo | `--lote 30` sem ruido |
|---|---|
| giroscopio | 30/30, 178 s |
| encoders, giro so por angulo, bitola efetiva exata (187,5) | 28/30, 219 s |
| encoders + giro na linha, bitola 187,5 | 28/30, 200 s |
| encoders + giro na linha, bitola 180 ou 195 (+-4%) | 0 e 1 de 30 |
| encoders + giro na linha, bitola 140 ou 150 | 0/30 |

A bitola efetiva e a real dividida pela derrapagem (150 / 0,8 = 187,5), nao a medida com a trena. Ainda sensivel a +-4%: o que sobra e o resto que usa o angulo contado (giros no verde, varredura de y na plataforma, varredura das ruas), nao os giros de rua.
`autocal=1` corrige a bitola a cada giro (a razao contado/pedido), mas o meio da faixa tem ~5% de vies e a corrige para cima: com a bitola ja certa cai de 28/30 para 15/30; com ela errada sobe de 0 para 5 a 7 de 30. Fica desligada.
Proximo passo: tornar o resto do codigo menos dependente do angulo (varredura de ruas por faixa, nao por angulo) e medir a bitola efetiva no carro de verdade (girar 360 graus pelo encoder e ver o quanto o carro girou).
