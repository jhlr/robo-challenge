# Plano: aoBloquear pela geometria (passo 3) e stall de 150 s (passo 4)

Data: 2026-11-16. Estado de partida: `19acdfb` (17/20 completas, media 460 s). Sementes que falham: 1, 4, 11.

## Passo 3: aoBloquear (fantasma da semente 4)

**Problema.** `aoBloquear()` (robo.h, ~linha 148) bloqueia `ruaDe -> ruaPara`, a rua que o carro PERCORRE, e zera a crenca de cubo nela. Quando o obstaculo esta em outra rua (cubo sem plataforma, ou o outro robo), a rua percorrida fica falsamente vazia e bloqueada.

**Tentativa anterior (bd7778d, revertida).** Trocar por `ruaDoCuboVisto` (geometria do eco) piorou para 14/20. Causa provavel: perto de cruzamento `ruaDoCuboVisto` devolve falso (ambiguo), e nada era bloqueado. O bloqueio na rua percorrida, mesmo errado as vezes, impedia o carro de voltar ao mesmo cubo.

**Passos.**
1. Medir antes de mudar. Na semente 4 (SIM_DEBUG), contar as chamadas de `aoBloquear` e separar: (a) geometria valida e rua igual a percorrida, (b) geometria valida e rua diferente, (c) geometria ambigua (-1). Quantas da (b) e quantas da (c).
2. Hipotese: so (b) e o fantasma. (c) precisa do bloqueio antigo para nao haver re-visita.
3. Mudanca minima: geometria quando valida; quando ambigua, manter o comportamento antigo (rua percorrida). Nao setar cubo na crenca dentro de `aoBloquear` (so bloqueio).
4. Medir 20 sementes, mesmo binario recompilado.

**Aceite.** Completas >= 17/20; semente 4 melhora (menos tempo ou completa); nenhuma semente que completa hoje passa de +60 s.

**Risco.** Se (c) for grande, a versao hibrida mantem o fantasma nesses casos. Nesse caso o proximo passo e desambiguar pela distancia do eco (qual das duas ruas vizinhas do cruzamento), nao mais regra.

## Passo 4: stall de 150 s (sementes 1, 10, 11)

**Problema.** `bloqueiosTemporarios` (robo.h, ~linha 187) so cresce enquanto carrega (chegou a 14, com duplicatas) e so libera depois da entrega. Com 1 cubo por viagem, ruas com cubo no caminho ficam bloqueadas por viagens inteiras e o carro gira.

**Passos.**
1. Medir nas sementes 1, 10, 11: tamanho maximo da lista, duplicatas por entrega, e quanto tempo o carro fica sem avancar com a lista cheia.
2. Passo puro (sem mudar resultado): nao inserir par ja presente (checar `streetSlot(bloq, ...)` antes do `push_back`). Esperado: mesma saida, lista menor. Se a saida mudar, a lista esta sendo usada como contador e o passo para.
3. Se ainda travar: quando a unica rota para o verde/plataforma passa por rua bloqueada temporariamente, liberar a MAIS ANTIGA (nao todas) e registrar. Medir.

**Aceite.** Sementes 1, 10, 11 completam ou ficam abaixo de 300 s; 20 sementes >= 17/20.

## Ordem e entrega
1. Medicao do passo 3 (sem mudanca de codigo).
2. Passo 4.2 (dedupe, sem mudanca de resultado) como commit separado.
3. Passo 3 (mudanca minima), 20 sementes.
4. Passo 4.3 so se o 4.2 nao bastar.
Cada passo: um commit, doc com numeros, medicao com binario recompilado (`g++ -std=c++17 -O2 -Isrc -I../src -I../include -pthread test/simulador.cpp src/seguidor/decisao.cpp`).

## Resultado do passo 3 (medido)

Verdade do simulador por bloqueio (semente 4, instrumentacao em copia descartavel, repo intocado):
- 18 bloqueios por cubo sem plataforma: rua percorrida VAZIA e geometria COM cubo em 12; ambas com cubo em 2; so a percorrida com cubo em 1; geometria vazia em 3 bloqueios "outro robo" (ambas vazias).

Experimentos (20 sementes, 18 cubos):
| variante | completas | media completas |
|---|---|---|
| base 19acdfb / 87b2d8e | 17/20 | 460 s |
| geometria em todo bloqueio | 15/20 | 544 s |
| geometria so no bloqueio por cubo (outro robo: percorrida) | 15/20 | 544 s (identico) |

Leitura: o acerto de 12 de 18 na semente 4 nao se traduz no conjunto. Sementes 2, 6 e 11 caem para 2-3 entregas. O resultado identico nas duas variantes mostra que o regresso vem do bloqueio por cubo na geometria. Hipotese nao testada: zerar a crenca da rua geometrica (cubo real) faz o planejador deixar de ir la, e o cubo nao e entregue; a rua percorrida zerada era vazia de qualquer forma.

Decisao: revertido (repo em 87b2d8e). Passo 3 com geometria NAO entra. Proximo: testar so a parte de bloqueio sem zerar a crenca (bloq = true, crenca intacta) antes de descartar a geometria.

## Passo 3, revisado: bloquear nao e esvaziar

Hipotese do usuario: a regra "rua sem cubo nao vale revisitar" ficou forte demais. Confirmado no codigo: `aoBloquear` zerava a crenca da rua. Crenca 0 cai em `kEmptyBelow` (0,1), e `kKnownEmpty` tira essa rua de qualquer destino (`navigator.h`), alem de pesar a rota (`routesFrom`, penalidade de rua vazia). Ou seja: um bloqueio temporario por cubo ou outro robo virava "vazia para sempre".

Mudanca: `aoBloquear` so marca o bloqueio (`bloq`), sem zerar a crenca. Rua bloqueada continua fora das rotas; so deixa de ser descartada como vazia.

| variante | completas | media completas |
|---|---|---|
| base 87b2d8e | 17/20 | 460 s |
| geometria no bloqueio (acccea7, revertida) | 15/20 | 544 s |
| so bloqueio, sem zerar crenca | 17/20 | 452 s |

Sementes 1, 4 e 11 continuam sem completar (seed 1: 3 entregas em 288 s, depois para). Ganho de tempo pequeno; mantido porque remove a regra errada e nao piora nenhuma semente completa em mais de ~20 s.
