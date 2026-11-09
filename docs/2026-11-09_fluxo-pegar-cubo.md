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
- `firmware/src/core/pickplace.h`: maquina de estados pura em duas fases, porque a navegacao (`robo.h`) cuida de andar o mapa, achar cubo e plataforma e girar. `PickPlace::pick()`: cor, re 9 cm, abre, desce, fecha, confere pegada, sobe, confere frente livre; sem pegada levanta, confere de novo, reposiciona, desce e tenta outra vez (ate 3 fechamentos). `PickPlace::place()`: posicao e cor da plataforma, re 13 cm, desce a altura de plataforma, abre, sobe. Qualquer falha para e sobe o ombro (exceto se o proprio ombro travou).
- Ligada ao carro: `CarroReal::coletar(i)` e `CarroReal::soltar(cor)` rodam a maquina sobre o ombro real, o recuo por encoders (`recuar`), o sonar e o TCS da frente. Em `robo.h`, `coletar` devolve bool (falhou = cubo continua la, vira rua bloqueada temporaria e o carro volta por outra) e `soltar` e chamado na plataforma (nao largou = cubo perdido). O simulador responde sempre true: o braco nao e simulado.
- `firmware/src/hal/shoulder.*`: ombro direto em ticks, sempre em passos de 64 ticks a partir da posicao atual, checando posicao e corrente. Bringup: `l u|m|p` e `d <graus>`.
- `firmware/src/hal/claw.*`: garra SG90 **STUB** (`pins::kClaw = -1`): abrir e fechar so registram, a pegada sempre confere. O SG90 nao da retorno de forca nem de posicao (o usuario fechou com forca e a leitura nao mudou). O pino 22 que estava no `pins.h` e o SCL do TCS do chao, por isso foi desatribuido.

## Cubo na garra: guardar ja
Carregando, a prioridade e entregar: a maquina nao tem desvios para coletar. A navegacao ja simulada (`robo.h`) segue direto para o oeste/portao e, se acha outro cubo no caminho, nao pega: marca a rua como bloqueada temporariamente, lembra dele para depois da entrega e volta por outra rua (`tratarCubo`, `aoBloquearTemporario`).

## Seguranca: mao segurando o carro (`core/handguard.h`)
Regra do usuario: sonar muito perto (< 30 mm, ou eco 0) por 3 leituras faz o carro dar re de 5 cm (`CarroReal::recuar`, por encoders). Se depois da re o obstaculo continua tao perto, ele veio junto: e a mao, o carro esta no ar. Para as rodas, pausa a missao (o tempo parado nao conta no limite) e so volta depois de 5 leituras livres. Parede ou cubo ficam para tras e a missao segue. Ligado em `src/robo/main.cpp` (`guardaDaMao`, chamada a cada ciclo, inclusive dentro de giros).
- A cor nao entra: com a mao a 2-4 cm o canal clear do TCS da frente ficou em 130..330 (parede branca ~250), nao escuro.
- A guarda so vale com a garra no topo (ombro a ate 100 ticks de 1981, lido do servo a cada 60 ms): abaixada, a garra fica na frente do sonar e nao pode contar como obstaculo. Posicao ilegivel conta como abaixada. Isso tambem cobre pegar e colocar, que acontecem com o ombro fora do topo.
- Limitacao: o HC-SR04 nao mede bem abaixo de ~2 cm; mao colada pode dar eco 0 ou sem eco (sem eco nao conta como perto).

## Pendente
- Garra real: pino (livre e sem conflito), angulos aberta e fechada (`kClawOpenDeg/ClosedDeg` sao chute, nao medidos), e uma pegada de verdade (sensor ou chave); hoje `holding()` e sempre true.
- `reposition()` e so um registro: falta reapontar para o cubo pelo sonar e acertar a distancia.
- A sequencia pegar/largar nunca rodou no carro; o ombro desce 83 graus a partir do topo. Testar com o carro parado e o ombro livre antes de `g`.
- CAD: `cad/braco/params.json` ainda tem `shoulder_ratio = 4`.
