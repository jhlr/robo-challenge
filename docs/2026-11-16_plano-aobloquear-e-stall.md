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
