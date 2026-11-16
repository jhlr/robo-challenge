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
