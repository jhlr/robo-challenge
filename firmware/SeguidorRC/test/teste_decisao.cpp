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
  { Memoria m;  // curva leve para a esquerda: roda esquerda lenta
    const Comando c = passo(m, aj, "..#.....", 1);
    assert(c.esq == aj.velLenta && c.dir == aj.velRapida); }
  { Memoria m;  // ponta so vale com 2 leituras seguidas
    Comando c = passo(m, aj, "#.......", 1);
    assert(c.esq == aj.velReto && c.dir == aj.velReto);
    c = passo(m, aj, "#.......", 2);
    assert(c.esq == -aj.velGiro && c.dir == aj.velGiro); }
  { Memoria m;  // perdeu a linha: lacuna mantem, depois gira no eixo, depois abre em espiral
    passo(m, aj, "..#.....", 0);                         // vira para a esquerda (ultimoLado = -1)
    Comando c = passo(m, aj, "........", 10);
    assert(c.esq == aj.velLenta && c.dir == aj.velRapida);   // lacuna: mantem o ultimo comando
    c = passo(m, aj, "........", 10 + aj.lacunaMs);
    assert(c.esq == -aj.velBuscaExterna && c.dir == aj.velBuscaExterna && !c.parar);   // gira no eixo para a esquerda
    c = passo(m, aj, "........", 10 + aj.lacunaMs + aj.buscaGiroMs + aj.buscaEspiralMs / 2);
    assert(c.esq > -aj.velBuscaExterna && c.esq < aj.velBuscaExterna && c.dir == aj.velBuscaExterna);   // espiral: roda de dentro acelerando
    c = passo(m, aj, "........", 10 + aj.lacunaMs + aj.buscaGiroMs + aj.buscaEspiralMs);
    assert(c.esq == -aj.velBuscaExterna && !c.parar);    // recomeca o giro: nunca para
    c = passo(m, aj, "...##...", 99999);                 // linha voltou: anda reto
    assert(c.esq == aj.velReto && !c.parar); }
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
