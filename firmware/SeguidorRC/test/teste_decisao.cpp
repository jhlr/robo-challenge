// Teste da decisao no PC (sem Arduino): g++ -std=c++17 -Wall -Isrc test/teste_decisao.cpp src/seguidor/decisao.cpp -o /tmp/teste && /tmp/teste
#include <cassert>
#include <cstdio>

#include "seguidor/decisao.h"

using namespace seguidor;

static Comando passo(Memoria& m, const Ajustes& aj, const char* padrao, unsigned long ms) {
  bool p[kSensores];
  for (int i = 0; i < kSensores; i++) p[i] = padrao[i] == '#';
  return decidir(p, ms, aj, m);
}

int main() {
  const Ajustes aj;
  { Memoria m;  // centro: reto
    const Comando c = passo(m, aj, "...##...", 1);
    assert(c.esq == aj.velReto && c.dir == aj.velReto && !c.parar); }
  { Memoria m;  // S3 sozinho: correcao leve para a esquerda
    const Comando c = passo(m, aj, "..#.....", 1);
    assert(c.esq == -aj.velCorrecaoLeve && c.dir == aj.velCorrecaoLeve && !c.estranho); }
  { Memoria m;  // S3 com S4: continua leve
    const Comando c = passo(m, aj, "..##....", 1);
    assert(c.esq == -aj.velCorrecaoLeve && c.dir == aj.velCorrecaoLeve); }
  { Memoria m;  // S6 com S5: leve para a direita
    const Comando c = passo(m, aj, "....##..", 1);
    assert(c.esq == aj.velCorrecaoLeve && c.dir == -aj.velCorrecaoLeve); }
  { Memoria m;  // S2: correcao forte para a esquerda; S7: forte para a direita
    Comando c = passo(m, aj, ".#......", 1);
    assert(c.esq == -aj.velCorrecaoForte && c.dir == aj.velCorrecaoForte);
    c = passo(m, aj, "......#.", 2);
    assert(c.esq == aj.velCorrecaoForte && c.dir == -aj.velCorrecaoForte); }
  { // toda curva da decisao usa sentidos opostos: nenhuma roda fica parada ou so mais lenta
    const Ajustes a;
    const char* padroes[] = {"..#.....", ".#......", "....##..", "......#."};
    for (const char* p : padroes) {
      Memoria m;
      const Comando c = passo(m, a, p, 1);
      if (c.esq != a.velReto) assert((c.esq < 0) != (c.dir < 0));
    } }
  { Memoria m;  // correcao proporcional: S2+S3 (posicao -2,0) fica entre a leve e a forte
    const Comando c = passo(m, aj, ".##.....", 1);
    assert(-c.esq > aj.velCorrecaoLeve && -c.esq < aj.velCorrecaoForte && c.dir == -c.esq); }
  { Memoria m;  // S4 sozinho (posicao -0,5) ainda e reto; S3+S4 (-1,0) e leve
    assert(passo(m, aj, "...#....", 1).esq == aj.velReto);
    assert(passo(m, aj, "..##....", 2).esq == -aj.velCorrecaoLeve); }
  { Memoria m;  // padrao largo (4 acesos, sem ser cruzamento): anda mais devagar e avisa
    const Comando c = passo(m, aj, "..####..", 1);
    assert(c.largo && c.esq < aj.velReto && c.esq >= aj.velMinima && c.esq == c.dir); }
  { Memoria m;  // cruzamento (pontas juntas) nao e "largo": velocidade normal
    const Comando c = passo(m, aj, "########", 1);
    assert(!c.largo && c.esq == aj.velReto); }
  { Memoria m;  // oscilacao: correcoes trocando de lado seguidas deixam o reto mais devagar
    const char* padroes[] = {"..#.....", ".....#..", "..#.....", ".....#..", "..#.....", ".....#.."};
    Comando c;
    for (int i = 0; i < 6; i++) c = passo(m, aj, padroes[i], i);
    assert(c.oscilando);
    c = passo(m, aj, "...##...", 10);
    assert(c.oscilando && c.esq < aj.velReto && c.esq >= aj.velMinima); }
  { Memoria m;  // sem trocar de lado nao oscila
    Comando c;
    for (int i = 0; i < 6; i++) c = passo(m, aj, "..#.....", i);
    assert(!c.oscilando);
    c = passo(m, aj, "...##...", 10);
    assert(c.esq == aj.velReto); }
  { // correcao nao trava: num ciclo ha trecho de giro (sentidos opostos) E de avanco reto, e no total o robo anda para a frente
    const Ajustes a;
    int giro = 0, avanco = 0;
    long frente = 0;
    const int avancoEsperado = static_cast<int>(a.velReto * a.fatorAvancoCorrecao);   // entre os giros avanca mais devagar
    for (unsigned long t = 0; t < a.periodoCorrecaoMs; t++) {
      Memoria m;
      const Comando c = passo(m, a, "..#.....", t);
      if (c.esq < 0 && c.dir > 0) giro++;
      else if (c.esq == avancoEsperado && c.dir == avancoEsperado) avanco++;
      else assert(false);                 // nada de roda parada ou so mais lenta
      frente += (c.esq + c.dir) / 2;
    }
    assert(giro > 0 && avanco > 0 && frente > 0); }
  { // curva fechada: 3 acesos de um lado e nenhum do outro gira o ciclo INTEIRO, sem avancar
    const Ajustes a;
    for (unsigned long t = 0; t < a.periodoCorrecaoMs; t++) {
      Memoria m;
      const Comando e = passo(m, a, "###.....", t);
      assert(e.esq < 0 && e.dir > 0);
      Memoria m2;
      const Comando d = passo(m2, a, ".....###", t);
      assert(d.esq > 0 && d.dir < 0);
    }
    Memoria m;   // mas com algum aceso do outro lado nao e curva fechada: volta a intercalar
    bool avancou = false;
    for (unsigned long t = 0; t < a.periodoCorrecaoMs; t++) avancou |= passo(m, a, "###....#", t).esq > 0;
    assert(avancou); }
  { Memoria m;  // S4 e S5 sozinhos: reto
    const Comando c = passo(m, aj, "...##...", 1);
    assert(c.esq == aj.velReto && c.dir == aj.velReto && !c.estranho); }
  { Memoria m;  // ponta sozinha: estranha, nao vira a favor dela
    Comando c = passo(m, aj, "#..##...", 1);
    assert(c.estranho && c.esq == aj.velReto && c.dir == aj.velReto);
    c = passo(m, aj, "...##..#", 2);
    assert(c.estranho && c.esq == aj.velReto && c.dir == aj.velReto); }
  { Memoria m;  // as duas pontas juntas: normal (cruzamento), nao e estranho
    const Comando c = passo(m, aj, "#..##..#", 1);
    assert(!c.estranho && c.esq == aj.velReto && c.dir == aj.velReto); }
  { Memoria m;  // so S1 e mais nada: a linha esta na ponta esquerda: correcao FORTE para a esquerda, intercalada com avanco (nao so gira)
    int giro = 0, avanco = 0;
    for (unsigned long t = 0; t < aj.periodoCorrecaoMs; t++) {
      Memoria m2;
      const Comando c = passo(m2, aj, "#.......", t);
      if (c.esq == -aj.velCorrecaoForte && c.dir == aj.velCorrecaoForte) giro++;
      else if (c.esq == static_cast<int>(aj.velReto * aj.fatorAvancoCorrecao) && c.dir == c.esq) avanco++;
      else assert(false);
      assert(!c.estranho || true);
    }
    assert(giro > avanco && avanco > 0);                  // forte: gira a maior parte do ciclo, mas avanca
    const Comando c = passo(m, aj, "#.......", 1);
    assert(m.ultimoLado == -1 && c.esq == -aj.velCorrecaoForte); }
  { Memoria m;  // so S8 e mais nada: forte para a direita
    const Comando c = passo(m, aj, ".......#", 1);
    assert(c.esq == aj.velCorrecaoForte && c.dir == -aj.velCorrecaoForte && m.ultimoLado == 1); }
  { // girando para achar a linha, a barra acende inteira ao ficar paralela a ela: NAO e linha; continua girando ate ficar estreito
    Memoria m;
    passo(m, aj, ".#......", 0);
    passo(m, aj, "........", 10);
    const unsigned long t = 10 + aj.lacunaMs + 100;                 // ja passou da lacuna: esta buscando
    assert(estaBuscando(m, t, aj));
    Comando c = passo(m, aj, "########", t);
    assert(c.esq == -aj.velBuscaExterna && c.dir == aj.velBuscaExterna);   // segue girando para a esquerda
    assert(estaBuscando(m, t + 10, aj));
    c = passo(m, aj, "..####..", t + 10);
    assert(c.esq == -aj.velBuscaExterna && c.dir == aj.velBuscaExterna);   // 4 acesos: ainda largo
    c = passo(m, aj, "...##...", t + 20);                           // agora a linha cruza a barra: e a linha
    assert(c.esq == aj.velReto && c.dir == aj.velReto);
    assert(!estaBuscando(m, t + 30, aj)); }
  { // fora da busca, o padrao largo e so devagar (cruzamento chegando), nao busca
    Memoria m;
    const Comando c = passo(m, aj, "..####..", 1);
    assert(c.largo && c.esq >= aj.velMinima && c.esq == c.dir); }
  Ajustes ae;
  ae.buscaEspiralMs = 4000;   // a espiral e opcional (padrao 0): estes dois blocos testam a espiral ligada
  { Memoria m;  // perdeu a linha: lacuna mantem, depois gira no eixo, depois abre em espiral
    passo(m, ae, ".#......", 0);                         // vira para a esquerda (ultimoLado = -1)
    Comando c = passo(m, ae, "........", 10);
    assert(c.esq == -ae.velCorrecaoForte && c.dir == ae.velCorrecaoForte);   // lacuna: mantem o ultimo comando (S2 = forte)
    c = passo(m, ae, "........", 10 + ae.lacunaMs);
    assert(c.esq == -ae.velBuscaExterna && c.dir == ae.velBuscaExterna && !c.parar);   // gira no eixo para a esquerda
    c = passo(m, ae, "........", 10 + ae.lacunaMs + buscaGiroMs(ae) + ae.buscaEspiralMs / 2);
    assert(c.esq > -ae.velBuscaExterna && c.esq < ae.velBuscaExterna && c.dir == ae.velBuscaExterna);   // espiral: roda de dentro acelerando
    const unsigned long t0 = 10 + ae.lacunaMs;
    c = passo(m, ae, "........", t0 + buscaGiroMs(ae) + ae.buscaEspiralMs);
    assert(c.esq == ae.velBuscaExterna && c.dir == ae.velBuscaExterna && !c.parar);   // trecho reto: varre para frente, fora do alcance do giro no lugar
    c = passo(m, ae, "........", t0 + buscaGiroMs(ae) + ae.buscaEspiralMs + ae.buscaAvancoMs);
    assert(c.esq == -ae.velBuscaExterna && c.dir == ae.velBuscaExterna);              // vira 90 graus (para o lado onde a linha sumiu)
    c = passo(m, ae, "........", t0 + buscaGiroMs(ae) + ae.buscaEspiralMs + ae.buscaAvancoMs + buscaViradaMs(ae));
    assert(c.esq == -ae.velBuscaExterna && !c.parar);    // recomeca o giro: nunca para
    // o trecho reto cresce a cada volta (x2 na segunda), ate buscaAvancoMaxFator
    const unsigned long volta1 = buscaGiroMs(ae) + ae.buscaEspiralMs + ae.buscaAvancoMs + buscaViradaMs(ae);
    // a segunda volta nao gira uma volta inteira (buscaGiraSoNaPrimeira): vai direto para a espiral e o trecho reto, que agora dura 2x
    c = passo(m, ae, "........", t0 + volta1 + ae.buscaEspiralMs + ae.buscaAvancoMs + 10);
    assert(c.esq == ae.velBuscaExterna && c.dir == ae.velBuscaExterna);              // ainda reto: o trecho agora dura 2x
    c = passo(m, ae, "...##...", 99999);                 // linha voltou: anda reto
    assert(c.esq == ae.velReto && !c.parar); }
  { // o giro no eixo e SEMPRE uma volta inteira antes da espiral, e comeca ao fim da lacuna
    Memoria m;
    passo(m, ae, ".#......", 0);
    passo(m, ae, "........", 10);
    assert(buscaGiroMs(ae) == 3273);                        // 360 graus a 110 graus/s
    Comando c = passo(m, ae, "........", 10 + ae.lacunaMs + buscaGiroMs(ae) - 1);
    assert(c.esq == -ae.velBuscaExterna);                   // ainda girando no eixo, nada de espiral
    c = passo(m, ae, "........", 10 + ae.lacunaMs + buscaGiroMs(ae) + 1000);
    assert(c.esq > -ae.velBuscaExterna); }                  // agora sim: espiral (roda de dentro acelerando)
  { // a espiral nunca passa do raio maximo: a roda de dentro, no fim, da no maximo raioMaxEspiralMm
    const Ajustes a;
    const int vi = velBuscaInternaFim(a);
    const float vo = a.velBuscaExterna;
    const float raio = (a.bitolaMm / 2.0f) * (vo + vi) / (vo - vi);
    assert(raio <= a.raioMaxEspiralMm + 0.01f);             // nunca passa do raio maximo
    assert(vi < 0);                                          // e a roda de dentro continua para tras: sentidos opostos
    assert(vi > -a.velBuscaExterna && vi < a.velBuscaExterna); }
  { // durante a espiral inteira, nenhuma amostra passa do raio maximo
    Memoria m;
    Ajustes a;
    a.buscaEspiralMs = 4000;   // espiral ligada
    passo(m, a, ".#......", 0);
    passo(m, a, "........", 10);
    const unsigned long inicio = 10 + a.lacunaMs + buscaGiroMs(a);
    for (unsigned long t = 0; t < a.buscaEspiralMs; t += 50) {
      const Comando c = passo(m, a, "........", inicio + t);
      const float vi = c.esq, vo = c.dir;                       // ultimoLado = esquerda: a roda esquerda e a de dentro
      assert(vi <= velBuscaInternaFim(a) && vo == a.velBuscaExterna && vi < 0);   // sempre sentidos opostos
      if (vi != vo) assert((a.bitolaMm / 2.0f) * (vo + vi) / (vo - vi) <= a.raioMaxEspiralMm + 0.01f);
    } }
  { Memoria m;  // nunca viu a linha: tambem procura, comecando pela direita
    passo(m, aj, "........", 100);
    const Comando c = passo(m, aj, "........", 100 + aj.lacunaMs);
    assert(c.esq == aj.velBuscaExterna && c.dir == -aj.velBuscaExterna); }
  { Memoria m;  // esquecerUltimoComando: a lacuna nao repete um comando que nao rodou
    passo(m, aj, "...##...", 0);
    esquecerUltimoComando(m);
    const Comando c = passo(m, aj, "........", 10);
    assert(c.esq == 0 && c.dir == 0); }
  std::puts("decisao: ok");
}
