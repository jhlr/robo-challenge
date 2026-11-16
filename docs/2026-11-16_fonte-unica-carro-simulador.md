# Carro e simulador na mesma fonte (2026-11-16)

O simulador rodava a logica do carro com numeros proprios, diferentes dos do carro real. Isso escondia por que o carro se perdia. Agora os dois leem `firmware/include/params.h`
(via `seguidor/perfil_carro.h`: `ajustesDoCarro()` e `cruzamentoDoCarro()`), e o sensor morto usa uma funcao so (`preencherMortos`, em `decisao.h`).

## O que divergia (simulador antigo x carro real)
| Numero | Simulador antigo | Carro real |
|---|---|---|
| Barra a frente do eixo | 30 mm | 15 mm (`kAxleToSensorMm`, medido com regua em 2026-11-16; o chute inicial no params.h era 60) |
| Velocidade de reta (PWM 110) | ~254 mm/s | ~160 mm/s (`kStsFullSpeedMmS / kPwmFullSts`) |
| Giro da busca | chute 110 graus/s | ~66 graus/s (`w = 2 v / bitola`) |
| Bitola | 140 supoe, 150 real | 185 efetiva |
| Zona morta | 65 | 0 (STS em malha fechada) |
| S7/S8 | lidos de verdade | mortos, copia do vizinho |
| Lacuna na fita | 120 ms (~30 mm) | agora 60 mm (distancia, nao tempo) |

## Causas de "se perde e nao volta" achadas no simulador
1. A busca mede o giro por TEMPO com `grausPorSegundoBusca`; o chute (110) era 1,7x o giro real, entao a "volta inteira" girava ~215 graus e a linha atras do carro nunca era vista.
   A busca (o rumo vem dos encoders) achava a linha em 4 de 12 casos no simulador antigo; com o valor da fisica e a barra a 60 mm (suposicao de entao) acha em 11 de 12.
   Com a barra medida (15 mm) a varredura de 360 graus alcanca so ~52 mm do eixo e a busca acha 8 de 12 (falha nos de 60-100 mm de distancia), em 7 a 17 s.
2. A lacuna de 120 ms cobria ~19 mm no carro real: uma emenda de 60 mm virava busca, o carro girava e voltava pela linha que acabara de deixar, em laco (lacuna de 60 mm: NAO chegou em 600 s; agora chega).
3. `AjustesCruzamento::velMmS` (250) tambem era chute do carro antigo.

## Busca (rumo pelos encoders)
Conjunto de 40 casos (`--busca-lote`: 5 distancias da linha x 8 rumos). Antes: 19 de 40 achados. Agora: 40 de 40, ~5 s em media, pior ~13 s.
1. Linha so na ponta da barra (S1 ou S8 sozinho): o eixo esta mais longe da linha do que o giro alcanca (os sensores da ponta ficam a ~40 mm do eixo com a barra a 15 mm), entao girar no lugar so roçava a linha e o carro a perdia de novo. Agora avanca metade do ciclo (`fracaoGiroPonta` 0,5; era 0,95 de giro).
2. O quadrado em espiral crescia 1, 2, 3 x e depois repetia o mesmo quadrado; agora os lados crescem de dois em dois (1, 1, 2, 2, 3, 3), sem deixar faixa sem varrer.
3. A volta inteira de 360 graus (~6 s) virou 120 graus; roda de busca a 110 PWM e lado do quadrado de 600 ms (em `ajustesDoCarro`).

## Corrigido depois (barra a 15 mm: o braco do giro e curto, ~16 mm)
- O carro atravessava o portao, virava no 1o cruzamento e voltava pelo portao (portao contado 2x). Duas causas em `robo.h`: (1) `virarParaRua` parava na hora em que as pontas apagavam e ainda andava ~8 mm na freada (tau da roda x velocidade), depois somava o avanco ate o centro: o eixo girava 7 mm fora do centro. Agora mede a freada pelo odometro e anda (ou da re) so o que falta (`andarMm` aceita distancia negativa; `esperarParar` e a unica espera de freada). (2) `girarNaLinha` fechava na rua de ONDE VEIO (a barra a 15 mm a cobre ate ~45 graus): o limite para ignorar a rua de origem passou de 0,5 para 0,7 do giro.
- `reta off-25 ang+20`: a linha comeca fora do alcance da barra, entao o inicio e uma busca (nao seguimento), e o erro final assenta a meio passo de sensor (S4/S5 sozinho e a zona reta). O cenario so exige `ticksBuscando == 0` quando a barra ve a linha no inicio, e o erro aceito (`kErroFinalMm`, 8 mm) vem da quantizacao da barra (passo ~10,7 mm), nao de um numero ajustado ate passar.
- `teste_alvo.cpp` estava desatualizado desde o passo de rua de 40 cm (`kPassoMm` 400): os ecos agora saem de `kPassoMm`.
- O cenario do tabuleiro agora exige 1 portao, 3 cruzamentos, sem falso alarme e sem voltar (antes parava no 1o cruzamento).

## Em aberto
- S7/S8 (GPIO35/34, so de entrada) seguem mortos: a barra RC precisa de pino com saida para carregar o capacitor. Mover para GPIO13/GPIO4 e zerar `kLineDeadMask`.
- Nada disto foi gravado no carro (sem acesso ao carro; OTA/USB pendentes).

## Como rodar
- Simulador: `g++ -std=c++17 -O2 -Isrc -I../src -I../include -pthread test/simulador.cpp src/seguidor/decisao.cpp -o /tmp/sim && /tmp/sim [parametros]`;
  `SIM_TRACE=1` imprime uma linha a cada 50 ms (pose, barra, acao). `barra=30` muda a barra fisica E a que o firmware supoe; `mortos=0` barra saudavel; `encoder=0` TT amarelos.
- Teste do perfil: `g++ -std=c++17 -Wall -Isrc -I../include test/teste_perfil_carro.cpp src/seguidor/decisao.cpp -o /tmp/t && /tmp/t` (a parte da busca fica vermelha com `Ajustes{}` puro).

## MPU removido (2026-11-16)
O carro nao tem MPU: saiu todo o codigo dele (`inercial.h` com levantado/colisao/tracao/ancora na grade, `assimetria.h`, `calibracao.h`, a leitura do giroscopio em `robo.h`, o modelo de MPU do simulador e os cenarios so de MPU).
O rumo vem sempre dos encoders; `segurarRumoNaLacuna` ficou em `segurar_rumo.h`. `Robo::ligarImu` sumiu: o rumo se inicia no primeiro `tick`. Antes ela tambem esticava `lacunaMs` para 450; agora vale o do `ajustesDoCarro()`.

## Revisao de tempo e gargalos (2026-11-16, noite)
Medido com o cronometro novo (`Robo::resumoFases`, `SIM_FASES=1 ... --coleta 6 3`; no carro, a linha `tempo:` da telemetria a cada 500 ms) e com `SIM_TRACE`/`SIM_TRACE2=ini-fim`.

**1. A missao no simulador com a fisica do carro quase nao entrega.** `--coleta 6 5`: 0 a 1 cubo de 6 em 12 min (3 entregas em 5 sementes). No simulador antigo (254 mm/s, barra a 30 mm) o mesmo codigo entregava os 6 em ~550 s: parte da "velocidade" era do simulador. A 160 mm/s a rua de 400 mm custa 2,5 s e o piso do deslocamento fica ~12 s por 5 ruas; o resto e overhead.

**2. Onde o tempo vai (por fase, sementes 0-2, antes dos ultimos ajustes):** borda 44-50% (laco de meia-volta no vermelho depois de errar a pose), cruzamento 51% numa semente, reto 23-40%, plataforma 4-11%. Cada cruzamento custa ~4 s (parar + TCS 3x + varredura de +-90 graus + virar) contra 2,5 s para andar a rua.

**3. Causas achadas (todas no simulador, com o numero):**
- Barra a 15 mm: o giro "fecha na linha" (`girarNaLinha`) nao funciona, a barra ainda cobre a rua de onde veio a 90 graus. `virarParaRua` agora gira pelos encoders (a grade e de multiplos de 90; o seguimento corrige o resto). Se a patinagem real for grande, a rua nova pode cair fora da barra: conferir na pista.
- `virarParaRua` decidia "rua cruzada saiu da barra" por S1/S8 apagados; com S8 morto (copia o S6) a linha por onde veio mantinha S8 aceso e o carro andava 88 mm antes de girar. Agora usa padrao estreito (<= 3 acesos).
- Depois de errar a pose (cubo perdido, `VERDE inesperado`, portao do outro lado) o carro anda ate sair da arena: 15 km em 8 min a 160 mm/s com a decisao dizendo "gira direita". Falta um limite global (odometro desde o ultimo cruzamento/borda vista > ~1,2 rua = perdido: parar e buscar).
- A partida nao observa as plataformas antes de coletar: coleta um cubo de cor sem plataforma, gasta ~85 s procurando e o perde (-10). `entrarNaCidade` poderia chamar `mapearDeLonge` uma vez no verde (~20-30 s) antes do primeiro cubo.

**4. Gargalos de tempo no laco do carro real (por leitura do codigo; agora medidos pela telemetria `tempo:`):**
- `CarroReal::avancar`: por ciclo leitura RC da barra (ate ~2 ms com limiar adaptativo), 2 leituras de encoder STS e 2 escritas de velocidade STS (cada uma e uma transacao serial), mais `vigiar()` (BLE + OTA + comandos). Se a media passar de 5 ms o `dtMs` que a logica supoe nao e o real.
- `Ultrasonic::readMm` usa `pulseIn` com timeout de 8 ms: bloqueia o ciclo ate 8 ms sem eco. Na varredura do cruzamento ele e chamado a CADA ciclo; o HC-SR04 nao da leitura util a menos de ~60 ms. Ler a 30-60 ms e interpolar.
- `guardaDaMao` le o sonar a cada 60 ms quando a garra esta levantada (outros 8 ms bloqueados).
- Parada no cruzamento: freada (~50 ms de tau) + 3 leituras do TCS paradas (90 ms) + varredura de 360 graus de giro a 99 graus/s (3,6 s). Sugestao: varrer so quando a pose nao e segura (ja e assim com `varrerSoQuandoPrecisa`) e usar o gesto de 45 graus (0,9 s) ou nenhum giro quando as duas ruas ja foram medidas.
- Velocidade: reta a 110 PWM = 160 mm/s de um maximo de 174 (`kStsFullSpeedMmS`): so ha 8% de folga; ganhar tempo em reta so com roda maior ou relacao de engrenagem.

## Depois da revisao: placa S3, observacao de plataformas, limites (2026-11-16, noite)
- **Placa ESP32-S3.** Barra S1..S8 = GPIO 1, 2, 4, 5, 6, 7, 8, 9 (todos leem; `kLineDeadMask = 0`). Mapa sugerido para o resto (pins.h, `platformio.ini` agora `esp32-s3-devkitc-1`): STS RX 18 / TX 17, I2C do chao SDA 11 / SCL 12, I2C da frente SDA 13 / SCL 14, sonar TRIG 15 / ECHO 16 (divisor 5 V -> 3,3 V), buzzer 21, garra 10 (quando ligar), LED -1 (o da placa e NeoPixel no 48). Evitar: 0, 3, 45, 46 (strapping), 19/20 (USB), 26-37 (flash/PSRAM), 43/44 (console). Os 4 envs (robo, follow, rapido, seguidor) compilam para a S3.
- **Observar as plataformas antes do primeiro cubo.** Quando a pose fica segura (1a vez), o navegador faz uma viagem ao ecoponto com a carga vazia (`nav.observando`, reusa o fluxo de entrega): le a cor de todas as plataformas, fixa o que e coletavel e re-mede o y por uma plataforma conhecida (`yPeloGiro`) antes de reentrar. Cenario `observa as plataformas antes do primeiro cubo` (observou em ~120 s, 3 cores coletaveis, 0 cubos coletados antes). Custo: ~85 s de varredura.
- **Limite no verde.** No verde sem ver linha o carro anda no maximo 700 mm (`kMaxVerdeSemLinhaMm`); antes seguia reto para sempre se a pose estivesse errada (15 km em 8 min no simulador).
- **Sonar a cada 30 ms no carro** (`CarroReal::us`): a varredura chamava a cada ciclo de 5 ms e cada leitura sem eco bloqueava ate 8 ms.
- **Medicao.** `--coleta 6 5` ainda entrega 2 de 6 cubos em 5 sementes (antes: 0 a 1). O que sobra e de localizacao: ao reentrar na cidade num y que nao e o de uma rua, o 1o cruzamento chega "cedo" (181 mm de 360) e e lido como borda do tabuleiro, a pose inverte (linha 0 <-> 4) e o carro sai pelo portao errado. O cenario de partida do simulador sempre comeca em cima de uma rua, entao o caso de y aleatorio (a partida real) nao e exercitado: proximo passo = sortear o y de partida no simulador e corrigir a leitura do 1o cruzamento (`chegou CEDO`) para so valer com pose segura.

## Humano pode recolocar o carro: partida aleatoria e pedido de ajuda (2026-11-16, noite)
Regra: pegar o carro e poe-lo de volta no verde e permitido (custa o reinicio). Entao o caso que importa e largar bem de uma posicao ALEATORIA do verde (a partida e cada recolocacao), e nao se recuperar sozinho de qualquer situacao.
- `--coleta` agora parte de uma posicao aleatoria do verde (determinista pela semente), como a partida real. Antes partia sempre em cima de uma rua e escondia os defeitos abaixo: 0 de 6 cubos em 5 sementes.
- **Pedir ajuda** (`kMaxSemMetaMs`, 150 s): sem nenhuma meta (coletar, entregar, observar plataformas, passar o portao) o carro PARA e apita (`Carro::sinal`) ate o verde aparecer sob o sensor; o simulador faz o papel do humano (8 s depois, ponto aleatorio do verde). Antes ele vagava (busca 58-78% do tempo, ou 15 km para fora da arena).
- **Rumo ao ser recolocado:** `VERDE inesperado` agora zera `rumo` (sem MPU o rumo so vem dos encoders; o texto dizia que reiniciava mas nao reiniciava).
- **Pose segue o rumo:** a busca gira o carro sem passar por `aposVirar`, e a pose continuava olhando para onde ele entrou; agora `aoCruzamento` alinha o sentido da pose ao rumo dos encoders (a linha contada de cima virava a de baixo).
- **'Chegou cedo' so com distancia conhecida:** partindo de um cruzamento nao ancorado (`nav.ancorando`) nao ha distancia esperada ate o proximo; com 360 mm supostos o 1o cruzamento de verdade era lido como borda do tabuleiro.
- Resultado: ate o 1o cubo o fluxo funciona de ponta a ponta (ancora, observa 5 plataformas, coleta). `--coleta 6 5`: 2 entregas em 5 sementes (3 sementes sem entrega, 1 com 2 entregas). O gargalo agora e a entrega: na semente 4 o carro vai ao y da plataforma azul vista na observacao, nao a encontra (`plataforma ERRADA (vi cor 0)`) e gasta ~100 s; a conta de y por odometro desde o portao erra o suficiente para perder a marca.
- Tempo por fase (sementes 0-4, 720 s): giro no cruzamento 10-36%, plataforma 13-28%, reto 18-24%. A varredura de +-90 graus a cada cruzamento e o maior custo recorrente.

## Entrega e conferencia com as regras (2026-11-16, noite)
**Entrega (`--coleta 6 5`, partida aleatoria no verde): 0 entregas -> 10 entregas em 5 sementes (uma semente completou os 6 cubos em 715 s).** Causas e correcoes:
1. **Rumo dos encoders 20 graus torto no portao** (`rumo 520` contra th real 540): o rumo deriva a cada giro da linha e da busca, e `entregarNaPlataforma` virava para o oeste a partir dele, descendo torto e perdendo a marca. Agora, andando reto sobre a linha por 150 mm (`kRetoParaReancorarMm`), o rumo volta ao multiplo de 90 mais perto (as ruas sao eixos).
2. **`achou()` voltava 90 mm fixos** depois de cada tentativa falha e andava ate 260 mm: derivava ~170 mm para o oeste por tentativa e saia da arena (x = -1445). Agora volta exatamente o que andou e a moldura vermelha tambem para a aproximacao.
3. **Candidatos de plataforma a cada 130 mm (9 posicoes)**: as plataformas ficam nas 5 marcas, a 260 mm (160 de comprimento + 100 de vao). Agora so as 5 marcas, e o encaixe do y usa 260.
4. **Viagem de observacao das plataformas no inicio: DESLIGADA por padrao** (`Navegador::observarNoInicio`, `observar=1` no simulador). Custa ~85 s e a reentrada depois dela falhou (o carro seguia a linha azul do portao): 4 entregas em 5 sementes com ela, 10 sem. O cenario `observa as plataformas...` continua testando-a.

**Cubos e arena contra as regras (PDF V1.0, `Regras_Challenge_2026_Portugues-V1.0.pdf`):**
- OK: 36 cubos = 5 de cada uma das 6 cores + 6 brancos (`semearRegras`); cubos de rua no MEIO do segmento entre cruzamentos (quadrados cinza da figura 5); cubos nos quadrados marrons so como obstaculo; 1 a 5 plataformas de cores distintas nas 5 marcas, 160 x 50 mm em pe (160 em y), 100 mm entre elas, pitch 260; cor sem plataforma = tratada como branco (identificar, nao mexer, rua bloqueada); partida em posicao aleatoria do verde; entrega so conta sobre a plataforma da cor; pontos 100 / 50 / -10 / -15 reinicio / -5 saida da arena.
- **Divergencia: passo das ruas.** O PDF da arena 2840 x 1500 com verde 280 + ecoponto 220 por lado deixa 1840 x 1460 mm de cidade; 6 x 5 cruzamentos nessa area dao passo de ~300-316 mm (a imagem mede ~302). O simulador e o firmware usam `kPasso = 400` (medida do usuario em 2026-10-29; becos de 168 e ecoponto de 291 foram escalados de 302 para 400). Com 400 mm a cidade teria 2000 mm de largura, mais que os 1840 do PDF. Se o 400 e de uma arena de teste propria, ok; se a arena da competicao e a do PDF, `kPasso`, `kBeco`, `kEcoponto*`, os limiares de `alvo.h` (kCuboMinMm/MaxMm) e o `trechoRestante0` precisam voltar a ~302.
- **Capacidade:** as regras permitem 2 cubos por viagem; o carro usa 1 (`coleta.capacidade = 1`). Com a entrega custando 60-100 s, 2 por viagem e o maior ganho de pontos ainda aberto.
- **Robo inativo (-10):** parar e apitar esperando o humano pode ser lido como inativo; o reinicio (-15) sai de qualquer forma.

## Otimizacao de tempo a 1 cubo por viagem (medido)

Capacidade fica em 1 cubo/viagem. Cronometro por fase agora separa `varre`, `gesto` e `vira`
(antes um `giro` so). Participacao do tempo (sementes 3/7/11): reta ~25-28 %, plataforma 19-27 %,
vira 14-17 %, varre 13-18 %, gesto 9-11 % (cruzamento ~40 % junto). Por cruzamento: ~2,5 s andando
e ~4 s de overhead.

Varredura de 60 sementes (6 cubos, 12 min), entregas totais:

| config | entregas |
|---|---|
| olhar adiante em toda virada (antes) | 132 |
| olhar so se a rua nova nao foi medida (novo padrao `olharSoSeNaoMedida`) | 136 |
| idem + gesto=20 | 138 |
| antes + gesto=20 | 135 |

Diferenca entre configs (+-5) esta no ruido: ajustar parametros nao move o resultado. O ganho
estrutural restante esta em (1) busca de plataforma na 1a entrega (~84 s, e o 1o cubo de cor sem
plataforma conhecida se perde) e (2) partida ANCORAR (~30 s). Observacao inicial segue desligada.

### Busca de plataforma: duas ideias testadas e descartadas (5 sementes, 6 cubos)

Na semente 7 o eco parado dava plataformas em y=828 e 1108 mm, mas as reais estao nas marcas 708 e 1228
(compressao ~0,54 em torno do y do carro: o y do eco nao e confiavel). Entregas, baseline 9:
- encaixar o eco na marca mais proxima: 0 (ler a cor na marca "certa" falha, o resto da navegacao compensa o vies);
- ignorar ecos fora da grade de 260 mm como alvo: 4.
Revertido. Antes de mexer na ordem da busca, corrigir a causa: por que o y estimado pelo eco sai comprimido (angulo da varredura parada / D2).
