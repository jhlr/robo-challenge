# Cubo sem plataforma: politica de coleta (resultado negativo)

## Seed 8: a causa correta
- A seed 8 entrega 5 de 6 porque o 6o cubo (preto, cor 6) nao tem plataforma do nosso lado. Os 5 cubos entregues sao exatamente os que tem plataforma (cor 2 x2, cor 3 x2, cor 5).
- O preto foi coletado em 120 s e descartado em 179 s com "cubo sem plataforma da cor: perdido" (-10 pontos).
- A leitura anterior ("o cubo (1,0)-N ficou vazio por VAZIA-SEM-ECO") estava errada: esse cubo foi coletado as 278 s e entregue as 290 s.
- Causa do descarte: `coletavel` começa com todas as cores verdadeiras (`alvo.h:71`) e a observacao das plataformas do nosso lado (`procurarPlataforma`) para ao achar a cor procurada. Antes de ver todas, uma cor sem plataforma e coletada.

## Tentativa: so coletar cor com plataforma conhecida
Mudanca em `robo.h` (ramo `podeColetar`): so pega se a plataforma da cor ja foi vista (ou tudo foi olhado).

| Variante | Sementes completas (6 cubos, 20 sementes) |
|---|---|
| HEAD (sem mudanca) | 19 / 20 |
| Gate, sem observacao (`observar=0`) | 0 (nenhuma cor desconhecida e coletada: travou) |
| Gate + observacao (`observar=1`) | 11 / 20 (seeds 1 e 3 completas; seed 8 em 5/6) |

Decisao: revertida. A regra e correta em principio (cubo sem plataforma nao vale nada), mas a observacao de todas as plataformas no inicio custa tempo e piora a entrega em 8 sementes. Nao ha como saber a cor sem plataforma sem essa viagem.

## Seed 8 nao e alcancavel em 6/6
Com o preto sem plataforma, o maximo possivel nesta semente sao 5 cubos. O "5 de 6" nao e um defeito do planejador: e a meta impossivel pelas regras (cubo sem plataforma nao deve ser coletado). Proximo passo se quiser medir isso: a meta de entrega passar a ser "todos os cubos com plataforma", e nao "6".

## Proximo passo sugerido
- Custo de observar as plataformas no inicio (`observar=1`) sem o gate, medido nas 20 sementes, para separar o efeito da viagem do efeito do gate.
- Varredura de gambiarras em `robo.h` (remendos `kMaxEcosNaoCubo`, `deMemoria`, throttle de 300 ms) ainda nao feita.
