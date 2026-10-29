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

## Regra das duas pontas acesas (acima de todas as outras)

S1 e S8 no preto juntos, em QUALQUER situacao (seguindo, perdido, buscando), tem prioridade sobre a decisao, a busca e o resto (so levantado/motores parados vem antes). Para e roda `passoVarredura` (`giro_cruzamento.h`, maquina de estados pura; o sketch e o simulador so alimentam leituras e motores): vira 90 graus comecando pelo lado MENOS EXPLORADO (`exploracao.h`: rumo do MPU + cruzamentos ja visitados; sem MPU, o lado da ultima correcao) e decide por historico + padrao:
- vinha seguindo a linha: **cruzamento** se ha rua dos dois lados; senao falso alarme.
- estava perdido/buscando: se depois de virar o padrao fica **estreito** (1 a 3 acesos), era ele de lado em cima da linha: **endireitou** e segue por ela; senao vira para o outro lado; senao falso alarme.
O padrao estreito nao exige o centro: com a barra sobre a linha o eixo fica ~30 mm ao lado dela (o simulador mostrou).

## Simulador ao vivo (sem fim)

```bash
cd firmware/SeguidorRC && g++ -std=c++17 -O2 -Isrc -I../src test/simulador.cpp src/seguidor/decisao.cpp -o /tmp/simulador && /tmp/simulador --servir
# abra http://localhost:8765
```
O servidor C++ simula so o que a pagina pede, SEM duracao gravada nem tempo maximo (a pagina nao tem slider): botoes de avancar/voltar 0,1 s e 1 s, velocidade ate 8x, clique no mapa para soltar o robo ali (apontando para o lado escolhido) e ver a busca acontecendo, levantar/soltar de frente e pancada forte. `/tmp/simulador` sem argumentos roda os 54 cenarios de teste (com `-q` so o placar; `chave=valor` muda fisica e ajustes).

## Mundo do simulador: cubos, oponente, sensores novos e ruido

`test/simulador.cpp` agora modela (tudo CONFIGURAVEL em `Mundo::cfg`, ligado no simulador antes de existir no robo):
- **Ultrassom** (alcance 900 mm, cone 20 graus, ruido 3 mm) e **TCS da frente** (cor do cubo a ~9 cm). `alvo.h` (puro) diz o que o eco e: cubo numa rua (`ruaTemCubo`), **oponente = eco que se mexe** (`ecoMovel`: o cubo e parado, entao andando v mm/s para ele a distancia cai v*dt; outra variacao e alvo que se move) e se a cor do cubo se pega (`podeColetar`, configuravel: branco e cor sem plataforma bloqueiam a rua). No sketch: `-DUSA_ULTRASSOM=1 -DUS_TRIG=.. -DUS_ECHO=..` (desligado).
- **TCS do chao** com cadencia de 60 ms, verde/amarelo, leitura parado com 3 leituras que concordam, e **reposicao pelo verde** so depois de varias leituras seguidas (`reposicionadoPeloVerde`).
- **Cubos** nas ruas (nunca no cruzamento), **oponente** que anda pelo mapa, leva cubos e bate no robo (o MPU sente a pancada), **ruido aleatorio SINALIZADO** (mancha escura que parece linha, patinagem, atraso dos sensores, leitura errada do TCS: cada evento entra no registro e a mancha e desenhada; liga/desliga na pagina).
- **Navegador** (no simulador; usa o `core/`): localizador Bayes (`localizer.h`), memoria dos cubos que esquece com o tempo (`belief.h`), **exploracao por regioes menos visitadas** (`explore.h`: sem cubo provavel vai para o cruzamento menos visitado, nao para a primeira rua), ruas **bloqueadas** (`Blocked`), ancora a coluna na azul do outro lado e a linha na vermelha, coleta (para a ~6 cm, le a cor, pega ou bloqueia), entrega no verde e volta pelo portao. A pagina mostra o que o carrinho lembra (rosa = chance de cubo por rua, X = rua bloqueada, numero = vezes que passou).

**Limites conhecidos (a missao ainda e informativa nos testes):** a localizacao deriva depois de pancadas do oponente e de curvas que nao sao verificadas (depois de virar nao se confere se a rua existe), e entao ele fica "ancorando" em vez de coletar; o navegador ainda nao esta ligado ao sketch.

## Regras do desafio no simulador (docs: `Regras_Challenge_2026_Portugues-V1.0.pdf`)

- **Cubos**: 5 de cada cor (azul, vermelho, verde, amarelo, preto, marrom) e 6 brancos; so se pega a cor que tem plataforma no nosso ecoponto (o juiz poe de 1 a 5, cores distintas; configuravel em `AjustesColeta` e `Mundo::porPlataformas`). **Branco ou cor sem plataforma**: identificar, SINALIZAR (som/luz: +50, uma vez por cubo), nao mexer e tratar a rua como **bloqueada** (desviar). Cubos nos quadrados marrons do centro so sao obstaculo.
- **Capacidade 2** por viagem; depois de coletar tem de ENTREGAR na plataforma da cor (+100). Plataforma errada ou solto fora = cubo perdido (-10). Reinicio -15. Sair da arena / passar da vermelha ou da azul do outro lado = robo perdido, -5, volta ao verde.
- **Colisao entre robos (7.1)**: quem avancava contra o outro leva -10 e **volta ao ponto de partida (verde)**; o atingido e recolocado onde estava.
- **O oponente e outro robo IGUAL ao nosso** (mesmo firmware, outra thread, mesma arena espelhada, cubos compartilhados). Os dois veem um ao outro como eco do ultrassom (e como eco que se mexe).
- **A garra ainda nao e simulada**: o carregar e a trajetoria sim (cubos sobre o carro, destino tracejado, plataforma achada com o TCS da frente); o cubo "e solto" quando o carro para diante da plataforma.
- **Velocidade calibrada pela distancia entre cruzamentos**: de um cruzamento ao proximo em linha reta sao 302 mm; o tempo andando vira velocidade (media movel) e alimenta tempos de perda de linha, fim de beco e deslocamentos fora da cidade.
- **Ruido** (sempre sinalizado, ligado/desligado na pagina): sujeira que parece linha, desgaste branco na fita, patinagem, bateria fraca, um motor mais fraco, sensor RC preso, sensores atrasados, eco falso ou perda do ultrassom, leitura errada do TCS, viés do giroscopio andando e solavanco.

Descobertas do simulador: o ultrassom na ponta ve cubos da rua SEGUINTE se o limite de eco passar de ~240 mm (`AjustesAlvo::ruaMm = 230`); o debounce de 300 ms depois de virar engolia o cruzamento do beco e o carro passava da linha vermelha; a rua percorrida inteira sem cubo tem de virar "vazia" na memoria (a de tras da esquina nunca aparece na varredura); a ancora so da a coluna na azul do outro lado (`planAtCrossing` agora vai para o leste quando so a coluna e incerta); medir velocidade crescente de partida a parada subestima a velocidade reta (media 213 contra 254 mm/s), entao deslocamentos longos sem linha precisam de varredura em vez de contar so tempo.

## O que o ultrassom alcanca, a calibracao pos-reset e o giro no cruzamento

- **Alcance:** com ~900 mm ele ve, em linha reta, ate 3 ruas a frente (cone estreito: so cubos no eixo da rua). `alvo.h::ruaDoEco` diz em qual rua da linha esta o cubo (0 = a primeira); sem eco, as ruas inteiras ao alcance estao vazias. O eco vira INFORMACAO (memoria) e OBJETIVO (o navegador vai ate la), na varredura e tambem a cada 300 ms enquanto anda. Eco numa rua que ja era vazia de certeza = o outro robo.
- **Rua vazia e caminho normal**, so nao e alvo (nao insiste nela). `kEmptyStreetPenalty = 0` (parametro, caso queira preferir ruas nao vistas).
- **Depois de cada reset** (posicao aleatoria do verde, olhando para a cidade, rumo do MPU = 0): sobe ate a moldura vermelha para saber o y, alinha com uma rua (a linha de entrada fica conhecida) e **mede + varre os 5 primeiros cruzamentos** (velocidade, ruas, memoria).
- **Girar 90 graus no cruzamento VARRE o corpo do carro** (~130 mm de raio) por cima das ruas do lado: um cubo a 112-140 mm do centro e atingido. Com a pose segura e o cruzamento esperado nao gira: S1+S8 e a cor bastam. Varre so perdido, em duvida e na calibracao.

## Correcoes do mundo real e calibracao por contagem (decididas com o usuario)

- **Cubos no MEIO da rua** (os quadrados cinza ficam a ~151 mm do cruzamento): no robo real o giro de 90 graus no cruzamento NAO esbarrava nos cubos; o simulador colocava cubos a 112-190 mm do cruzamento e o corpo do carro (130 mm de raio) os atingia. Agora varre sempre (`pularVarreduraComPoseSegura = false`, opcional).
- **Calibracao depois de um reset**: nao varre 5 cruzamentos. Vai ate a linha VERMELHA de um dos lados CONTANDO os cruzamentos que viu (o localizador fixa a linha com a contagem + o vermelho) e mede a velocidade em cada trecho de 302 mm; o ultrassom olha longe (ate 3 ruas) no caminho, entao ja acha cubos distantes.
- **Pendente**: depois de calibrado nao precisaria mais subir ate a moldura vermelha no verde para saber o y (`usarAlinhamento = false` entra reto e a primeira rua vertical, de lado, o alinha: endireita 90 graus e conta ate o vermelho). No simulador essa variante ainda perde o portao em alguns casos (sai da cidade pela azul depois de desviar de um cubo): por isso o padrao continua alinhando.
