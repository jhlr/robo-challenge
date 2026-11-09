# Fluxo de pegar e colocar cubo (definido pelo usuario, 2026-11-09)

## Alturas do ombro (STS3215 direto, 4096 ticks = 360 graus, aumentar ticks = descer a garra)

| Altura | Ticks | Observacao |
|---|---|---|
| Topo (cima) | 1981 | inicio e transito; ~10 graus acima ja e o limite mecanico |
| Pegar | 2927 | limite inferior, garra no cubo do chao |
| Plataforma | 2567 | cubo sobre a plataforma (~31 graus acima de pegar) |

Constantes em `firmware/include/params.h` (`kShoulderTicksUp/Pick/Place`). Movimentos fisicos sao sempre deltas da posicao atual, em passos pequenos com checagem de corrente (`firmware/src/ombro_delta`).

## Sequencia

1. Ombro comeca no topo.
2. Anda pelo mapa.
3. Percebe o cubo com o ultrassom.
4. Confere a cor com o sensor de cor.
5. Da re de ~9 cm.
6. Abre a mandibula.
7. Desce o ombro (pegar).
8. Fecha a mandibula.
9. Confere se a mandibula segura algo.
10. Levanta o ombro ate o topo.
11. Confere pelo ultrassom que nao ha cubo na frente.
12. Vai ate a plataforma.
13. Confere o posicionamento da plataforma com o ultrassom.
14. Confere a cor da plataforma.
15. Da 13 cm de re.
16. Desce o ombro para a altura de plataforma.
17. Solta a mandibula.
18. Levanta o ombro ate o topo.
19. Gira 180 graus.
20. Repete tudo; priorizar cubos de localizacao ja conhecida.

## Implementacao
- `firmware/src/core/pickplace.h`: maquina de estados pura (`PickPlace::step(IO&)`), testada com um IO falso (7 testes em `test_core`). Sem pegada: levanta o ombro, confere de novo (o cubo pode so aparecer como seguro ao levantar), reposiciona (sonar), desce e fecha outra vez, ate 3 fechamentos; qualquer falha para, sobe o ombro (exceto se o proprio ombro travou: ai nao manda segundo comando) e registra `PpFail`. Cubo na garra (`carrying`) fica registrado para a missao decidir.
- `firmware/src/hal/shoulder.*`: ombro direto em ticks, sempre em passos de 64 ticks (~5,6 graus) a partir da posicao atual, checando posicao e corrente; `shoulderGoTo(Up|Place|Pick)`. Bringup: `l u|m|p` e `d <graus>`.

## Cubo na garra: guardar ja
Carregando, a prioridade e entregar: a maquina nao tem desvios para coletar. A navegacao ja simulada (`robo.h`) segue direto para o oeste/portao e, se acha outro cubo no caminho, nao pega: marca a rua como bloqueada temporariamente, lembra dele para depois da entrega e volta por outra rua (`tratarCubo`, `aoBloquearTemporario`).

## Seguranca: mao segurando o carro (`core/handguard.h`)
Regra do usuario: sonar muito perto (< 30 mm, ou eco 0) por 3 leituras faz o carro dar re de 5 cm (`CarroReal::recuar`, por encoders). Se depois da re o obstaculo continua tao perto, ele veio junto: e a mao, o carro esta no ar. Para as rodas, pausa a missao (o tempo parado nao conta no limite) e so volta depois de 5 leituras livres. Parede ou cubo ficam para tras e a missao segue. Ligado em `src/robo/main.cpp` (`guardaDaMao`, chamada a cada ciclo, inclusive dentro de giros).
- A cor nao entra: com a mao a 2-4 cm o canal clear do TCS da frente ficou em 130..330 (parede branca ~250), nao escuro.
- A guarda so vale com a garra no topo (ombro a ate 100 ticks de 1981, lido do servo a cada 60 ms): abaixada, a garra fica na frente do sonar e nao pode contar como obstaculo. Posicao ilegivel conta como abaixada. Isso tambem cobre pegar e colocar, que acontecem com o ombro fora do topo.
- Limitacao: o HC-SR04 nao mede bem abaixo de ~2 cm; mao colada pode dar eco 0 ou sem eco (sem eco nao conta como perto).

## Pendente
- Falta o IO real da maquina (inclui `reposition()`: reapontar para o cubo pelo sonar e ajustar a distancia): garra SG90 (angulos aberta/fechada nao medidos, e como saber se esta segurando algo), `backUp(mm)` pelos encoders, cor/sonar ja existem em `robo/carro_real.h`. `CarroReal::coletar` continua stub.
- Ligar a maquina a `Robo` (SeguidorRC/seguidor/robo.h): `goToPlatform` e `turn180` vem da navegacao ja simulada; `coletar(i)` e `entregarNaPlataforma` devem chamar a maquina.
- CAD: `cad/braco/params.json` ainda tem `shoulder_ratio = 4`.
- A altura de pegar derivou 150 ticks uma vez; era o usuario forcando a descida (torque segura bem).
