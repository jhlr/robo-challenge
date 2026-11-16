// g++ -std=c++17 -Wall -Isrc test/teste_segurar_rumo.cpp src/seguidor/decisao.cpp -o /tmp/teste_segurar_rumo && /tmp/teste_segurar_rumo
#include <cassert>
#include <cstdio>

#include "seguidor/segurar_rumo.h"

using namespace seguidor;

int main() {
  const Ajustes aj;
  { // lacuna: fora da tolerancia gira no eixo (rodas em sentidos opostos) de volta; dentro dela anda reto
    Comando c; c.parar = false;
    segurarRumoNaLacuna(c, 10.0f, 0.0f, aj);   // virou para a esquerda -> gira para a direita
    assert(c.esq > 0 && c.dir == -c.esq);
    segurarRumoNaLacuna(c, -10.0f, 0.0f, aj);
    assert(c.esq < 0 && c.dir == -c.esq);
    segurarRumoNaLacuna(c, 90.0f, 0.0f, aj);
    assert(c.esq > 0 && c.dir == -c.esq);
    segurarRumoNaLacuna(c, 2.0f, 0.0f, aj);    // dentro da tolerancia
    assert(c.esq == aj.velReto && c.dir == aj.velReto); }
  std::puts("teste_segurar_rumo: ok");
  return 0;
}
