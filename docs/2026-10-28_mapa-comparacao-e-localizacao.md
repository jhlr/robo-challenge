# Mapa da arena: comparacao com o grafo e localizacao (2026-10-28)

Imagem: `2026-10-28_mapa-arena.png` (1158 x 615 px para 2840 x 1500 mm, ~2,45 mm/px). Medi as linhas por pixel, nao a olho.

## 1. Grafo do firmware contra a imagem (ponto a ponto)
| Ponto | Imagem (medido) | `core/map.h` | Bate? |
|---|---|---|---|
| Linhas verticais | 6, em x = 273, 394, 517, 639, 761, 883 px | 6 colunas | sim |
| Linhas horizontais | 5, em y = 62, 184, 306, 429, 551 px | 5 linhas | sim |
| Espacamento | 122 px = ~299 mm nos dois eixos | 300 mm | sim |
| Cruzamentos | 6 x 5 = 30, **todos de 4 vias** | 30 nos | sim |
| Ruas entre cruzamentos | 25 horizontais + 24 verticais = 49 | `east` 25 + `north` 24 | sim |
| Largura da linha | 6 a 7 px = ~16 mm (regras: 20 mm) | nao modelada | informativo |
| **Becos nas pontas** | cada linha continua ~127 mm alem do ultimo cruzamento ate a borda | **nao existe** | **faltava** |
| Linha azul | x = 213-219 e 937-943 px (limites esquerdo e direito da cidade) | nao existe | faltava |
| Borda vermelha | y = 1-8 e 605-611 px (cima e baixo), x = 1-7 e 1149-1155 | nao existe | faltava |
| Cubos | so nas ruas (quadradinhos cinza); marrom no centro dos quadrados nao pega | so ruas | sim |

Becos (22 no total): 6 verticais em cima, 6 em baixo, 5 horizontais a esquerda e 5 a direita. Medi 52 px = ~127 mm em todos. O sim usava 150 mm.

**Consequencia importante:** os cruzamentos das pontas **nao sao cantos**. Tambem tem os 8 sensores no preto, porque cada um tem 4 vias (uma e um beco curto). Entao "8 no preto" nao diz se o cruzamento e de dentro ou da borda.

Geometria fora da cidade (esquerda para direita): ecoponto amarelo (90 px, 220 mm), area livre verde (114 px, 280 mm), linha azul, beco de 127 mm, primeiro cruzamento. A arena e simetrica por ponto: o outro robo entra pelo lado oposto.

## 2. Convencao de coordenadas (decidido: relativa ao nosso lado)
O robo pode comecar em **qualquer um dos lados** (varia por partida, decisao do usuario 2026-10-28). Como a arena e simetrica por ponto e os quadrados sao identificados de forma espelhada, o mapa **nao precisa saber se o robo esta a esquerda ou a direita da imagem**: as coordenadas sao sempre relativas ao nosso lado.
- `c = 0` e a coluna mais proxima da **linha azul por onde o robo entra** (a do lado do verde onde ele comecou); `c = 5` e a mais distante (e a do outro time).
- `r = 0` e a borda que fica a **direita** do robo ao entrar voltado para `+c`; `r = 4` e a da esquerda. A linha por onde ele entra (qual das 5) nao se sabe de inicio: a **linha** so se descobre ao chegar numa borda (beco sem saida ou vermelho).
- O robo descobre o lado ao achar **a linha azul do seu proprio verde**: a primeira azul que le e a nossa (ele comeca no verde, entre o amarelo e essa azul).

## 3. O que as regras mudam (lidas no PDF, secao 7)
1. **Posicao inicial:** os robos comecam em posicao **aleatoria na area livre (verde)**, colocados pelo arbitro. Nao ha pose inicial fixa: a localizacao parte do desconhecido, e o primeiro passo e achar e cruzar a **linha azul do nosso lado** para entrar na cidade.
2. **Vermelho = robo perdido:** o robo "deve responder de forma imediata sempre que qualquer sensor detectar a presenca da cor vermelha no chao". Os becos de cima e de baixo terminam na borda vermelha; entao **nao se usa o vermelho como ancora**: ele e proibido, nao referencia.
3. **Azul do outro lado = robo perdido:** passar da linha azul do lado adversario e proibido. Os becos desse lado terminam nela. So a **nossa** linha azul (o portao de entrada) e uma ancora segura.
4. **Cubos marrons** (X, Y, Z, no centro dos quadrados) sao so obstaculo para os sensores de distancia: nao tocar. O ultrassom apontado ao longo de uma rua pode ve-los de lado e confundir com cubo de rua.
5. **Cubos brancos, ou de cor sem plataforma:** identificar e **nao mover**; a rua fica **bloqueada** e o robo desvia por outra rota. Isso e uma rua com P(bloqueio) alta no mapa de crencas, nao um alvo.
6. **Onde ha cubo e aleatorio** (decisao do usuario, 2026-10-28): nao se sabe quais quadrados cinza recebem cubo. O mapa de crencas usa **o mesmo prior em todas as ruas** (`BeliefModel.prior`), sem zerar nenhuma. As regras identificam os quadrados cinza por letra (A e B de 1 a 5, C de 1 a 3, D a G de 1 a 3), mas isso so restringiria o prior se soubessemos a lista exata de ruas com quadrado; fica como melhoria futura.
7. **Dois cubos por viagem**, entregar antes de coletar de novo; partida de 12 minutos; tocar no robo conta como reinicio.

## 4. Localizacao: o robo nao pode andar cego
A pose hoje e so contagem de cruzamentos, e um cruzamento perdido a desloca sem ele perceber. Com as regras acima, as ancoras ficam:
1. **Nossa linha azul** (so ela): ao ler azul entrando na cidade, o robo sabe que esta no beco da coluna 0, voltado para dentro. Isso fixa coluna e direcao.
2. **Contagem de cruzamentos a partir dessa ancora**, com a confirmacao pelo giro de 90 graus (ja no sketch) para rejeitar falso cruzamento.
3. **Becos como pontos sem saida:** a linha some depois de ~127 mm. No topo/baixo/lado oposto o beco termina em vermelho/azul proibido: ao ver o fim de linha ou qualquer vermelho, **recuar na hora**, e usar o fim do beco como evidencia de borda (sem tocar a cor).
4. **Conferencia de contagem:** de coluna 0 ao beco do lado oposto sao 5 cruzamentos; se o beco (fim de linha) aparece antes ou depois, a contagem derivou: corrige a coluna e registra o erro.
5. **Giroscopio (MPU)** quando entrar, para manter a direcao entre curvas.

Seguranca: a pose tem confianca; com confianca baixa o robo **nao decide por um cubo distante**, volta a uma ancora (nosso azul) para se reancorar. Nunca avanca para um fim de linha na direcao do vermelho ou do azul adversario.

## 5. Feito (2026-10-28): `core/localizer.h`
Filtro de histograma (Bayes) sobre 30 cruzamentos x 4 direcoes, puro e testado no PC (54 testes no total). Eventos: `onOwnBlue`, `onCrossing` (com chance de cruzamento perdido), `onTurn`, `onLineEnd`, `onForbidden(Red | OtherBlue)`. `estimate()` da a melhor pose e a confianca; `columnMass` e `rowMass` dao a confianca de cada coordenada. Caminhar numa rua ensina a coluna mas nao a linha: a linha so aparece numa borda. `navigator.h::planAtCrossing` so usa o mapa de cubos se coluna e linha estiverem acima de 0,8; senao vai para a borda `+r` para se reancorar.

## 6. O que ainda falta no codigo (proposta)
1. `core/map.h`: acrescentar os becos (22 de 127 mm), marcar quais terminam em borda proibida (vermelho, ou azul do outro lado) e a convencao da secao 2.
   - `core/belief.h`: prior igual em todas as ruas (sem zerar nenhuma); "bloqueada" (cubo branco ou de cor sem plataforma) como estado proprio, que desvia a rota.
2. `core/localizer.h` (novo, puro): pose com confianca, `onCrossing`, `onOwnBlueLine`, `onLineEnd`; correcao por ancora e contagem do erro. Testado no PC com trajetos simulados, incluindo cruzamento perdido e a mais.
3. `navigator`: so decidir por cubo com confianca acima de um limiar; senao ir reancorar.
4. Sketch: acrescentar a leitura do TCS34725 do chao (cor da borda) e passar para o `localizer`.

## 7. Perguntas
1. ~~De que lado o robo comeca?~~ Resposta: no **verde**, em posicao aleatoria, em qualquer dos dois lados (varia). Resolvido pela convencao relativa da secao 2: a primeira azul que ele le e a nossa.
2. O TCS34725 do chao fica a frente das rodas de que distancia? Ele ve a linha azul antes do eixo chegar nela.
3. Pode confiar na cor azul e vermelha do chao no TCS (calibrar com a lona real, que ja e o que o colega faz)?
4. ~~Quais ruas tem quadrado cinza?~~ Aleatorio: prior uniforme em todas as ruas.
