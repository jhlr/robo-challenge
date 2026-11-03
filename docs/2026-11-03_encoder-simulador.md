# Encoder no simulador (STS3215) — 2026-11-03

O seguidor foi desenhado sem encoder (distancia = tempo x velocidade, zona morta e assimetria calibradas). Os STS3215 tem encoder e velocidade em malha fechada.
`simulador.cpp` ganhou o modo `encoder=1` (padrao desligado, entao o lote antigo nao muda).

## O que o modo faz
- `Carro::odoMm()`: percurso medio das rodas lido do eixo (nao ve a patinagem no chao); `escalaodo=` simula erro de escala.
- Sem zona morta nem assimetria fixa entre lados (o servo regula), sem calibracao de assimetria, sem medir velocidade entre cruzamentos nem subir ate a moldura vermelha.
- `andarMm` anda pela distancia lida e comeca a frear a tempo (v x tau), esperando parar; `andadoDesdeSaida` e `andadoRuaMm` vem do odometro.
- `yPeloGiro`: parado de frente para a plataforma, gira -36..+36 graus, o meio do trecho com eco e o centro e `y = centro - distancia x tan(meio)` (Pitagoras). Erro medido: ~6 mm.

## Medidas (30 sementes, sem ruido)
| modo | na meta | tempo medio |
|---|---|---|
| padrao (sem encoder) | 28/30 | 187 s |
| `zona=0` (STS nao tem zona morta) | 29/30 | 183 s |
| `encoder=1` | 11 a 18/30 | 360 a 450 s |

`encoder=1` ainda PIORA: a logica a jusante foi afinada contra erros de tempo. Causa achada ate agora: reentrada pelo portao ~28 mm fora da rua (o y vem de "vi a plataforma", +-25 mm; o giro no lugar tambem desloca ~7 mm), e o carro segue a faixa azul do portao como rua. Nao aplicar no firmware antes de fechar isso.

## Proximos passos
1. Fechar a reentrada (medir o y DEPOIS do ultimo giro; ou entrar pela rua do meio, linha 2, sem andar no verde).
2. Saida pela rua do meio e varredura parada (ideia do usuario): tirar a travessia do verde e a subida a moldura.
3. Velocidade adaptativa: rapido quando a pose esta segura e o trecho livre, freando pela distancia restante ate o cruzamento.
4. Giro por angulo: odometria diferencial para estimar o vies do giroscopio em reta.
