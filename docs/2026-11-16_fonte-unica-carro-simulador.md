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

### Y das plataformas: o eco estava certo, a grade supunha a fase errada

Investigacao (semente 1/7): o eco parado dava plataformas em y=859/1137 e a busca ia ler a cor em 968/708/1228 (marcas inteiras),
mas o cenario de coleta poe 4 plataformas ENTRE as marcas (slots 0,5..3,5 = 578, 838, 1098, 1358). Nao havia compressao: o firmware
supunha que as plataformas ficam sempre nas 5 marcas inteiras (448..1488), e alem de ler no lugar errado ainda encaixava o y do carro
nessa grade (erro de ate 130 mm, depois a reentrada no portao errava a rua e ele rodava a borda). Agora `Navegador::faseGrade`
(fase da grade de 260 mm) e aprendida pela primeira plataforma lida de perto (`aprenderGrade`) e o encaixe so vale depois (`naGrade`);
antes disso o eco e usado como esta. A fase real da arena (marcas ou entre elas) segue sendo uma duvida das regras: o codigo agora aceita as duas.

Metodo de teste: sementes RUINS (`ruins.sh` acha ate N com <=2 entregas de 6; `avalia.sh` mede so elas; boas sao descartadas).
Sementes ruins 0,1,2,4,9: 7 -> 18 entregas. Novo conjunto ruim: 1, 8, 14, 15, 16, 17.
Depuracao nova: `MAPEAR inicio/fim/eco` em SIM_DEBUG.

Limite de busca de plataforma 525 -> 560 mm do meio (uma marca extrema com erro de y de ~10 mm ficava de fora).
Falhas que restam nas sementes ruins (1, 8, 14, 15, 16, 17), ainda nao corrigidas:
1. Partida (semente 14): ANCORAR roda a coluna da borda esquerda por 150 s (meias-voltas na borda vermelha) ate pedir ajuda.
2. Pos-entrega (sementes 14, 16): depois de "VERDE inesperado" o carro fica indo e voltando sobre a faixa azul do portao ("atravessado" / "AZUL do outro lado" em laco),
   ou atravessa a arena ate o lado do oponente (x ~ 2900).
3. Cubo de cor sem plataforma na 1a viagem (semente 16): ~50 s de busca e -10; so a observacao inicial evita, e ela e pior no total.

### Laco no portao (semente 14, 16)

Causa: ao atravessar o portao azul, a travessia em frente terminava assim que o TCS deixava o azul E qualquer sensor via preto. Logo
depois da faixa so as pontas da barra (S1/S2) viam a sobra da faixa / a rua ~30 mm ao lado; o seguidor lia isso como curva fechada e
girava ~120 graus subindo pela faixa azul (laco de "PORTAO atravessado" / "AZUL do outro lado"). Agora a travessia so acaba com preto
nos 4 sensores do meio (`portaoSoMeio`), ou em 1,3 s. Sementes ruins 1,8,14,15,16,17: 12 -> 20 entregas; sementes 0-4: 18 -> 21; testes ok.

### Partida na borda (semente 14): diagnostico, sem correcao ainda

A semente 14 e recolocada no verde a y=1041 (a 97 mm da rua mais perto, fora da barra de 75 mm). Ao chegar na coluna 0 a barra le #######
por 3 leituras (x=147..158), o carro PARA uns 100 ms (parece a leitura de cor/eco), mas nao registra PARADA/cruzamento; segue 60 mm,
perde a linha, gira no eixo (120 graus, ~8 s de busca no lugar) e fica 50 mm fora da coluna. O rumo sai 30-60 graus errado
(-123, -149), o localizador perde a coluna (massa 1.00 -> 0.17) e a linha nunca fica segura: o carro roda a coluna 0 de ponta a ponta
(3 bordas, 150 s) ate pedir ajuda. Dos 5 outros starts ruins, so a 14 tem isso (chegar de lado numa rua no meio do quarteirao).
Proximo passo: descobrir por que a faixa preta de lado nao vira cruzamento (cruzamentoConfirmado / tratarCubo / parada de eco em x~150).
Depuracao: BELIEF agora mostra direcao e massa de linha/coluna.

Causa achada (semente 14): em ANCORAR `trechoRestante0 = -1` desligava a guarda "faixa do cruzamento de onde saiu", e o mesmo cruzamento era
contado duas vezes (PARADA com andado 5 mm), deslocando o localizador em uma linha. Agora, ancorando, um stop a menos de 60 mm do ultimo
cruzamento nao e chegada (`ancorarSemRepetir`). Com 108 mm (o minimo teorico antes da borda) pegava tambem o 1o cruzamento depois do portao
(~130 mm) e piorava as sementes 0-4 (21 -> 17); com 60 mm ficam 21. Sementes ruins (1,8,19,22,29,30): 10 -> 12; semente 14: 3 -> 4.

Semente 10: a guarda de 60 mm (acima) tambem engolia a faixa a ~30 mm DEPOIS de uma meia-volta por cubo na frente, que e o cruzamento a que ele
volta e quem consome `pularProximo`; a flag ficava pendente e pulava o proximo cruzamento de verdade (pose uma coluna atras, ~150 s perdido com
cubo, ate pedir ajuda). A guarda agora so vale com `!pularProximo`. Sementes ruins (2,8,10,12,18,19): 11 -> 15; sementes 0-4: 20 (era 21, ruido).

Semente 10, 2a falha (ainda aberta): na reentrada o carro cruza o portao a y=1346 (rua em 1368, 22 mm de erro de y); logo depois da faixa so
S2/S3 veem a rua, o seguidor sem historico (`vinhaSeguindo` falso) gira no eixo ("CURVA FECHADA esquerda") ate th=65 e a parada seguinte
e lida como "chegou CEDO" (borda azul), com a meia-volta voltando pela arena ate o lado do oponente (x~2300). `trechoRestante0` agora e zerado
ao cruzar o portao (valor da ultima saida, 360 mm, estava velho; correto, sem efeito medido). A causa a atacar e o giro no eixo com a rua so
22 mm ao lado logo depois do portao: devia corrigir de leve, nao girar.

### Giro no eixo logo depois do portao (semente 10)

Causa: a travessia do portao terminava com preto em S3..S6; com a entrada 22 mm fora da rua, so S2/S3 viam a rua (`.##.....`), o seguidor
lia "linha 2 sensores a esquerda" e girava no eixo (a barra, a 15 mm do eixo, nao leva a linha ao centro girando) ate 65 graus, depois
"chegou CEDO". Agora a travessia so acaba com preto nos 2 sensores do centro (S4/S5), ou 1,3 s, e segue reto ate a rua (ou o 1o cruzamento).
Sementes ruins (2,8,10,12,18,19): 15 -> 21; sementes 0-4: 20 -> 23; testes ok.

### Semente 8: plataformas existentes lidas como "vazia"

A semente 8 so entregava 2 de 6 mas nao estava parada: os 3 cubos seguintes eram de cores COM plataforma (azul/amarela/preta, slots entre as marcas), e a busca
dava "vazia" em y=1147/1391/611 (reais 1098/1358/578): o eco parado erra +20..+50 mm no y (bordas cortadas na varredura) e a conta de distancia soma ~20 mm, entao
o carro lia a 50-70 mm do centro e nao via a plataforma. Agora, se o eco ja tinha visto plataforma a menos de 80 mm da marca, antes de dar "vazia" tenta 55 mm para
cada lado (`tentarLados`). Sementes ruins (7,8,13,22,30,38): 10 -> 13 (semente 8: 2 -> 5); sementes 0-4: 23 -> 23; testes ok.

### Semente 7: portao lido como borda azul

Reentrada a y=1004 (36 mm fora da rua, rumo 5 graus torto): na faixa azul o branco aparece numa ponta da barra (`saiuDoVerde`) e o fim da travessia
(`algumPreto`) disparava com o carro AINDA em cima da faixa; a barra toda preta virava "cruzamento", a cor azul sem `portaoPendente` virava "BORDA azul do outro lado" e a
meia-volta levava o carro de volta ao verde, onde ele seguia a moldura vermelha do ecoponto (x=-640) por mais de 150 s. A travessia agora so termina com o TCS fora do azul
(`portaoAteAzulAcabar`; o teto de 1,3 s continua). Sementes ruins (7,13,22,30,38,53): 9 -> 20; sementes 0-4: 23 -> 22 (ruido); testes ok.
