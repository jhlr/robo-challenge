# Simulador da arena (RCB Challenge 2026), v0

Compara estrategias do robô em um grafo topológico da arena. Não é física: mede tempo e pontos.

```bash
python3 sim/run.py --episodes 150 --csv sim/results/baseline.csv
python3 -m unittest sim/test_sim.py
```

## O que está modelado

- Grade 6x5 de interseções, células de ~300 mm (estimado da Figura 1, não está no texto das regras), 49 ruas internas.
- 36 cubos (5 por cor, 6 brancos) nos pontos médios das ruas; 1 a 5 plataformas no Ecoponto, capacidade 3 (sem empilhar).
- Cubo bloqueia a rua: o robô lê a cor de perto, pega se for desejado, senão volta (e ganha 50 se for branco ou sem plataforma, uma vez por cubo).
- Oponente: a cada ~25 s leva ou empurra um cubo (35% leva, 65% empurra para um ponto vizinho) e às vezes bloqueia a rua (5%, 6 s).
- Pontuação das regras: +100 entrega, +50 identificação, -10 cubo perdido; sem pontuação negativa final.
- Erro de localização: cada interseção tem 2% de cruzamento perdido, 20 s para se relocalizar; "gated" (odometria + IMU conferindo) divide isso por 10.

## Políticas

1. `RandomWalk`: reativo, sem memória.
2. `Nearest`: mapa de crenças, vai ao cubo desejado mais próximo e entrega após cada cubo.
3. `Planner`: mapa de crenças com decaimento (meia-vida `tau`), leva até 2 cubos, escolhe alvos por valor esperado sobre tempo.

Modo `sensor_first` (padrão): a memória só guia; ao seguir por uma rua com cubo lembrado, o ToF olha antes (0,5 s) e a crença é corrigida na hora.

## Resultado (150 episódios, semente pareada)

Tabela completa em `sim/results/sensor_first.csv`. Leitura honesta:

- Qualquer memória bate o passeio aleatório (cerca de 950 a 1040 contra 780 pontos).
- "Memória decide" proíbe ruas que a memória diz bloqueadas; "sensor decide" só penaliza (+40 s) e o ToF confere a rua antes de entrar. Sensor decide custa ~4% de pontos (Planner tau=60: 952 contra 991, dentro do ruído de ~45) e corta as aproximações desperdiçadas quase pela metade (37 contra 69). Decisão do usuário: manter o sensor como decisivo.
- O decaimento da crença importa por causa do oponente: Planner sem decaimento 763 (sensor decide) contra 952 com tau=60 s.
- Com 30% das crenças gravadas erradas, os dois modos caem ~15 a 20% (786 e 826). O modo sensor decide não foi mais robusto em pontos neste modelo; não vale afirmar o contrário.
- O Planner com 2 cubos por viagem não superou o Nearest simples. Não complicar antes de medir no robô.
- O erro de localização custa pouco com os parâmetros supostos; calibrar com a taxa real de cruzamento perdido.

## Limites

Parâmetros de tempo, oponente e erro não foram medidos. Os resultados valem para comparar estratégias entre si, não para prever a pontuação real.

## Varredura da taxa do oponente (2026-10-24, 100 episodios, semente pareada)

CSVs em `results/opp_rate_{0.0,0.02,0.04,0.08}.csv` (eventos/s do oponente; o padrao e ~0,04 = 1 a cada 25 s). Erro padrao tipico 40 a 60 pontos: diferencas menores que ~100 entre duas politicas nao sao distinguiveis.

- O passeio aleatorio fica entre 806 e 913 em todas as taxas; qualquer politica com memoria o supera em ~100 a 300 pontos, menos o Planner sem decaimento com oponente parado (629 a 671: a crenca nunca envelhece, mas nada muda para justificar).
- Com oponente fraco (0 a 0,02) o Nearest simples e o melhor (970 a 1046 com memoria decide); o Planner tau=60 chega a 805 a 928. Com oponente forte (0,08) o Planner tau=60 sobe para 1193 e passa o Nearest (1098), mas a diferenca (~95) e cerca de 1,5 erro padrao.
- "Sensor decide" custa pontos, e mais do que os ~4% citados acima (uma semente so): Nearest -87, -146, -33 e -45 pontos (taxas 0, 0,02, 0,04, 0,08; 3% a 14%); Planner tau=60 -3, +22, -109 e -121 (ate 10% com oponente forte; em 0,02 ele ate ganha, dentro do ruido). Continua sendo a decisao do usuario (corta aproximacoes desperdicadas); o custo e real com oponente ativo.
- 30% de crenca errada derruba o Planner tau=60 em 14% a 25% em todas as taxas.
- Nada disso foi medido no robo: oponente, tempos e erro de localizacao seguem sendo suposicoes.
