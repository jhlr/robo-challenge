# SeguidorRC

Seguidor de linha preta **sem PID** para ESP32-S3 + ponte H TB6612FNG + barra de 8 sensores RC. Placa na Arduino IDE: "ESP32S3 Dev Module". Monitor Serial a 115200.

## Tres pecas independentes
| Arquivo | Faz | Usa pinos? |
|---|---|---|
| `sensores_rc.h` | le os 8 sensores de uma vez e calibra (`iniciar`, `calibrar`, `ler`) | sim |
| `decisao.h` | escolhe a velocidade das rodas (`decidir`) | **nao**, nem `millis()`: testa no PC |
| `ponte_h.h` | manda a velocidade para a ponte (`iniciar`, `mover`, `parar`, `aplicar`) | sim |

Sem classes: so funcoes e `struct` de dados (`BarraRC`, `PinosPonteH`, `Ajustes`, `Memoria`, `Comando`). O sketch liga uma peca na outra; veja `examples/SeguidorBasico`.
Voce pode usar so a decisao com outros sensores, ou so a ponte H em outro robo.

```cpp
ler(barra);
Comando c = decidir(barra.preto, millis(), ajustes, memoria);
aplicar(ponte, c);
```

## Usar
- Arduino IDE: copie (ou faca link de) a pasta `SeguidorRC` para `~/Documents/Arduino/libraries/`.
- arduino-cli: `arduino-cli compile --fqbn esp32:esp32:esp32s3 --library SeguidorRC SeguidorRC/examples/SeguidorBasico`
- Teste da decisao no PC: `g++ -std=c++17 -Isrc test/teste_decisao.cpp src/seguidor/decisao.cpp -o /tmp/teste && /tmp/teste`

## Ajustar na pista
Tudo em `Ajustes` (valores de partida em `decisao.h`): `velReto`, `velCorrecaoLeve`, `velCorrecaoForte`, `lacunaMs`, `buscaGiroGraus`, `grausPorSegundoBusca`, `buscaEspiralMs`, `velBuscaExterna`, `bitolaMm`, `raioMaxEspiralMm`. Os tempos e velocidades sao chutes razoaveis, **nao testados em hardware** (so compila para ESP32-S3, core 3.3.12). Com `kModoTeste = true` no exemplo os motores ficam desligados e so o Monitor Serial mostra a decisao.

**Toda curva move as rodas em sentidos opostos** (gira no eixo), nunca uma roda parada ou mais lenta arrastando. So girar nao sai do lugar (gira, passa do ponto, gira de volta e trava), entao cada correcao **intercala giro e avanco reto** dentro de `periodoCorrecaoMs` (`fracaoGiroLeve` / `fracaoGiroForte` dizem a parte do ciclo girando). Os TT amarelos nao andam abaixo de ~65 de PWM, entao `velCorrecaoLeve` e `velBuscaExterna` ficam acima disso.

## O que cada sensor significa (S1 = ponta esquerda ... S8 = ponta direita)
| Acionados | Faz |
|---|---|
| S4 e/ou S5, sem os outros | em cima da linha: **reto** |
| S3 ou S6 tambem | correcao **leve**: gira no eixo para esse lado (`velCorrecaoLeve`) |
| S2 ou S7 | correcao **forte**: gira no eixo, mais rapido (`velCorrecaoForte`) |
| S1 e S8 **juntos** | **cruzamento** (`cruzamento.h`: 2 leituras seguidas) |
| S1 **ou** S8 sozinho, **com** sensores do meio | **estranho**: nao vira a favor dele; `Comando.estranho = true` e o debug avisa |
| S1 **ou** S8 sozinho, **sem** o meio | a linha esta na ponta: correcao **forte** para esse lado, com avanco intercalado (so girar no lugar nao leva a linha ao centro: o eixo fica quase embaixo da barra) |

## O que mais a leitura parcial dos sensores diz
- **Posicao proporcional:** media dos acesos de S2 a S7 (S2 = -2,5 ... S7 = +2,5). Ate `posReto` reto, ate `posLeve` giro leve, de `posLeve` a `posForte` a velocidade sobe em rampa ate o giro forte.
- **Padrao largo:** 4 ou mais acesos sem ser cruzamento (`larguraAviso`): chegando num cruzamento, ou torto sobre a linha. Anda a `fatorLargura` da velocidade.
- **Oscilacao:** correcoes que trocam de lado `trocasOscilacao` vezes nas ultimas `janelaOscilacao`: ganho alto. Anda a `fatorOscilacao`. Nunca abaixo de `velMinima` (zona morta dos TT).
- **Chegou torto no cruzamento:** a diferenca de tempo entre S1 e S8 (`inclinacaoGraus`, com `velMmS` e `larguraBarraMm` do `AjustesCruzamento`, que sao chutes) diz o quanto; o sketch alinha antes de girar os 90 graus.
- **Fim de beco** (`classificarPerda`): a linha some ~`becoMm` (127 mm) depois do ultimo cruzamento: borda do mapa. Falha curta e lacuna; sumir em outro ponto e perdida.
- **A rua segue adiante?** (`ruaSegueAdiante`): depois de andar um pouco no cruzamento, S4 ou S5 acesos = ha rua.
- **Sensor travado** (`saude.h`): preto por mais de 30 s seguidos e sujeira ou defeito; o sensor e marcado `ok = false` e ignorado.

## Achar a linha de novo sem ficar de lado
Girando no eixo para procurar, a barra acende **inteira** ao ficar *paralela* a linha: o robo esta de lado, nao em cima dela. Durante a busca (`estaBuscando`), um padrao largo (`larguraAviso` ou mais acesos) **nao** e aceito como linha: ele continua girando ate a linha cruzar a barra (padrao estreito) e so entao segue. O sketch tambem **nao** detecta cruzamento enquanto busca.

## Linha perdida
Segue o ultimo comando por `lacunaMs` (falha na fita); depois **sempre** gira no proprio eixo uma volta inteira (`buscaGiroGraus`, para o ultimo lado) antes de espiralar, e so entao abre em espiral **limitada** (o raio da curva nunca passa de `raioMaxEspiralMm`, 125 mm, a folga ate os cubos dos quarteiroes) e volta a girar no eixo (roda de fora constante, roda de dentro acelerando) e **repete ate achar a linha**. Se nunca viu a linha, comeca pela direita.

## O que estava fazendo o robo rodopiar (codigo original do colega)
1. Linha perdida = giro no lugar para sempre, a PWM 100, sem limite. Agora ha lacuna, giro e espiral controlados.
2. Ponta (S1/S8) dava giro com uma unica leitura: agora as pontas nao guiam, so avisam quando uma acende sozinha.
3. Calibracao ruim passava batida: com menos de 6 sensores validos ela e recusada e repetida.
4. Os 8 pinos sao lidos de uma vez no registrador `GPIO_IN_REG` e a leitura espera so ate o maior limiar da calibracao.

## Borda do mapa (linha vermelha ou azul)

As ruas terminam numa linha vermelha (cima e baixo) ou azul (laterais) que acende S1 e S8 como um cruzamento, mas e a borda. `borda.h` decide pela COR do chao (TCS34725; a tabela de cores e `firmware/src/core/color_table.h`, uma so): vermelho ou azul sem cubo = meia-volta e volta para dentro do mapa; com cubo = para e deixa para o navegador. A cor vista vale por 400 ms (o TCS pode estar a frente ou atras da barra). No sketch fica desligada (`USA_COR 0`) ate ligar o sensor. Teste: `test/teste_borda.cpp`; cenarios no `test/simulador.cpp`.

## Calibracao das velocidades

`Ajustes::zonaMortaPwm` (PWM abaixo do qual a roda nao anda) e a unica calibracao de hardware: `normalizar(ajustes)` levanta toda velocidade de giro, correcao e busca para fora dela. No simulador, com a zona real em 80 e a decisao achando 65, 52 cenarios falham (o robo nao gira); com a zona calibrada, 2. Meca a zona antes de ajustar o resto. Os valores de correcao (`velCorrecaoLeve` 90, `fracaoGiroLeve` 0,7, `fracaoGiroForte` 0,95) vieram de uma varredura no simulador (`python` + `test/simulador.cpp -q chave=valor`), nao da pista.

## MPU6050, calibracao e o mapa completo

`inercial.h` e `calibracao.h` (puros) + `USA_IMU` no sketch: rumo ancorado na grade (multiplo de 90 numa reta, corrige tambem o vies do giroscopio), levantado -> motores parados -> espera estabilizar 1,5 s -> reposicionado (rumo 0, SEMPRE de frente; pose volta para "no verde", mapa de cubos fica), travado -> reforca o PWM e recua, colisao forte (oponente; cubo e leve e fica abaixo de 1,2 g), lacuna segurando o rumo, e calibracao automatica da zona morta e dos graus por segundo girando no eixo. Todos os limiares de `AjustesInercial` sao chutes: grave os picos reais (`colisao.picoG`) e corrija.

O simulador (`test/simulador.cpp`, `--html=arquivo` gera uma pagina que anima cada cenario) inclui o TABULEIRO INTEIRO (6 x 5 cruzamentos a 302 mm, becos de 127 mm, azul nas laterais, vermelho em cima e embaixo, verde e amarelo nas pontas). A primeira azul lida e o nosso portao (`AcaoCruzamento::Portao`); a do outro lado e borda.

**Busca da linha:** gira uma volta no eixo, anda reto (trecho cresce a cada volta, `buscaAvancoMs` ate x`buscaAvancoMaxFator`), vira 90 graus e repete; se a barra acender inteira durante o trecho reto, bateu de frente numa linha: gira no eixo ate a linha cruzar a barra. Depende de `grausPorSegundoBusca` certo: com o chute de 110 (real ~103) erra o rumo e acha 4 de 12 posicoes de partida; calibrado pelo giroscopio acha 9 de 12.
**Limite conhecido (virar para uma rua escolhida):** a barra para na borda da rua, com o eixo ~40 mm antes do centro. Antes de girar 90 graus para outra rua, avance ate as duas pontas apagarem (nao depende da largura da linha); o simulador faz isso em `virarParaRua`. O navegador real ainda nao esta ligado ao sketch.
