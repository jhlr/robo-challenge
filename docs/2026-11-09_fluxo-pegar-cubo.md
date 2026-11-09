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

## Pendente
- Firmware do ombro ainda assume reducao 4:1 (`kShoulderRatio`, centro e angulos antigos): trocar por ticks diretos.
- A altura de pegar derivou 150 ticks sozinha uma vez (2777 -> 2927); conferir se o torque segura o braco.
