# Stall de 150 s: carro com cubo sem rota ate o portao

## Sintoma
Nas sementes 1, 10 e 11 (18 cubos) o carro fica ~150 s sem meta e pede ajuda ("PEDINDO AJUDA: 151 s sem nenhuma meta"). O contador de meta conta coletas, entregas, portoes e plataformas observadas; durante o stall nenhum deles avanca.

## Mecanismo (seed 10, 178–329 s)
- O carro carrega 1 cubo (capacidade 1) e planeja "entregar".
- Cada rua que ele quer usar tem um cubo que nao pode pegar agora: "cubo ... no caminho de quem ja carrega: nao passa por cima, lembra dele e volta por outra rua". Isso chama `aoBloquearTemporario` (`robo.h:205/207`), que marca a rua como bloqueada.
- As ruas bloqueadas so voltam a valer depois da entrega (`liberarBloqueiosTemporarios`, `robo.h:213`). Enquanto carrega, a lista so cresce.
- No log, `bloqTemp` chega a 14 entradas. A lista empilha duplicatas (a mesma rua entra de novo a cada leitura), entao o numero de ruas distintas e menor, mas bloqueia o caminho ate o portao.
- O carro passa 150 s replanejando e vagando pela regiao oeste (poses (1,2), (2,2), (3,2), (3,1), (2,0)) ate o contador de meta estourar.

## Causa provavel (nao provada por busca de caminho)
Com cubos em ruas demais e capacidade 1, o bloqueio pode desconectar o portao do carro. Nao verifiquei a conectividade do grafo no momento do stall; a evidencia e o crescimento da lista e o padrao de replanejamento.

## Decisao pendente
A capacidade 1 foi escolha da equipe ("entrega logo"); a regra permite 2 cubos por viagem. Com 2, o carro pegaria o cubo do caminho em vez de contorna-lo, e esse bloqueio deixaria de existir. Isto muda o comportamento do carro real, entao e decisao sua.

## Correcao: bloqueio na rua do eco (nao na rua percorrida)
- `aoBloquearTemporario` tinha um fallback que bloqueava `ruaDe`→`ruaPara` (a rua que o carro percorre) sempre que o eco nao era cubo ou a geometria falhava. Um eco de outro robo ou parede bloqueava a rua onde o carro estava.
- Agora todo bloqueio usa a geometria do eco (`ruaDoCuboVisto`) e nao bloqueia nada se o eco for ambiguo. O caminho sem cubo passa a receber a distancia do eco.
- Diagnostico novo (SIM_DEBUG): `FANTASMA bloqueio rua ... sem cubo real`, comparando cada bloqueio com os cubos reais do simulador.

Resultado (20 sementes, 18 cubos):
- Seed 10: fantasmas 9 -> 0; 5/6 -> 6/6.
- Seed 4: 6/6 -> 5/6; ainda tem 2 fantasmas (0,4)-E a 47 s e (1,3)-N a 89 s. Causa nao verificada.
- Seeds 15 e 16: completam, mas mais lentas (545 s e 468 s, antes 410 s e 441 s).
- Liquido: 17/20 antes e depois; media 460 s (antes 458 s).

Proximo: achar a origem dos fantasmas restantes (seed 4).

## Seed 4: origem dos fantasmas (investigado, correcao testada e revertida)
- Fantasma (0,4)-E a 25 s: `aoBloquear()` (robo.h, bloqueio de cubo sem plataforma e do outro robo) marca `ruaDe→ruaPara`, a rua percorrida, como bloqueada e vazia. O cubo real nao esta ali.
- Testei trocar `aoBloquear` pela geometria do eco (como em `aoBloquearTemporario`): seed 4 6/6, mas 14/20 no total (antes 17/20). Sementes 2, 3, 6, 9 e 11 pioraram.
- Causa da piora: perto de um cruzamento a geometria do eco e ambigua (`ruaDoEco` devolve -1) e nada e bloqueado; o bloqueio pela rua percorrida, mesmo errado as vezes, era o que impedia o carro de voltar ao mesmo cubo. A regra "bloquear a rua do eco" sozinha nao vale.
- Revertido. Estado commitado: 17/20, media 460 s.
- Proximo: identificar o cubo sem plataforma pela cor na posicao certa (o TCS da frente pode ler um objeto que nao e cubo naquele momento), antes de decidir o bloqueio.
