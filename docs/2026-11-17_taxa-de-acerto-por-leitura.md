# Taxa de acerto por numero de leituras (sonar a 30 ms, velbusca 110, 10 sementes)

Metodo (simulador, copia descartavel do HEAD, nada do repo alterado): cada passagem de um cubo pelo cone do sonar (leituras reais consecutivas, <= 60 ms entre elas) conta N leituras. Reconhecida = a crenca da rua do cubo passou de < 0,5 para >= 0,5 entre o inicio da passagem e 2 s depois. Passagens com crenca >= 0,5 antes sao ignoradas (ja sabia).

| leituras na passagem | passagens | reconhecidas |
|---|---|---|
| 1 | 244 | 4% |
| 2 | 190 | 8% |
| 3 | 182 | 12% |
| 4-5 | 644 | 3% |
| 6-9 | 362 | 2% |
| 10+ | 135 | 24% |

(Passagens com 678 casos "ja sabia" ficaram fora.)

Leitura: a taxa quase nao sobe com o numero de leituras. Na maioria das passagens multi-leitura a crenca fica no prior (0,4) do comeco ao fim: o firmware nao consome essas leituras. Causas no codigo:
- Driving: `observarRaio` com o eco so roda a cada 300 ms (`ultimaObsRaioMs`, robo.h ~1426). Com leitura a 30 ms, ~10 leituras viram 1 observacao.
- Cruzamento: `observarRaio` usa uma leitura por direcao no momento da decisao (robo.h ~264).
Ou seja: o gargalo esta em quantas leituras a logica transforma em evidencia, nao so no sensor.

Limites da medida: a janela de 2 s e a associacao passagem -> rua sao aproximadas; 10 sementes.
