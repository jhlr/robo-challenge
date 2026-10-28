// g++ -std=c++17 -Wall -Isrc test/teste_exploracao.cpp -o /tmp/teste_exploracao && /tmp/teste_exploracao
#include <cassert>
#include <cstdio>

#include "seguidor/exploracao.h"

using namespace seguidor;

int main() {
  { Exploracao e;   // nada visitado: empate -> lado da ultima correcao; sem ele, esquerda
    assert(ladoMenosExplorado(e, 0.0f, true, -1) == -1);
    assert(ladoMenosExplorado(e, 0.0f, true, 0) == 1); }
  { Exploracao e;   // andou 3 cruzamentos para +x e depois 2 para +y (esquerda de quem anda para +x): os visitados ficam a ESQUERDA de quem aponta para +x
    registrarCruzamento(e, 0.0f); registrarCruzamento(e, 0.0f); registrarCruzamento(e, 0.0f);
    registrarCruzamento(e, 90.0f); registrarCruzamento(e, 90.0f);
    assert(e.x == 3 && e.y == 2);
    assert(visitadosDoLado(e, 0.0f, 1) == 0 && visitadosDoLado(e, 0.0f, -1) == 4);   // apontando para +x: a direita (y menor) tem 4 dos 5 visitados
    assert(ladoMenosExplorado(e, 0.0f, true, 1) == 1);                                // vira para a esquerda: menos explorado
    assert(ladoMenosExplorado(e, 180.0f, true, 1) == -1); }                           // apontando para -x a esquerda e y menor: invertido
  { Exploracao e;   // rumo desconhecido: cai no ultimo lado
    registrarCruzamento(e, 0.0f);
    assert(ladoMenosExplorado(e, 0.0f, false, -1) == -1); }
  { Exploracao e;   // rumo com erro pequeno ainda cai no multiplo de 90 mais proximo; as bordas do grid nao estouram
    for (int i = 0; i < 40; i++) registrarCruzamento(e, 12.0f);
    assert(e.x == kRaio && e.y == 0); }
  std::puts("teste_exploracao: ok");
  return 0;
}
