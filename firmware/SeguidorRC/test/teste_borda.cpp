// g++ -std=c++17 -Wall -Isrc test/teste_borda.cpp -o /tmp/teste_borda && /tmp/teste_borda
#include <cassert>
#include <cstdio>

#include "seguidor/borda.h"

using namespace seguidor;

int main() {
  assert(acaoNaPonta(CorLinha::Outra, false) == AcaoCruzamento::Cruzamento);
  assert(acaoNaPonta(CorLinha::Outra, true) == AcaoCruzamento::Cruzamento);   // cubo na mao nao muda um cruzamento normal
  assert(acaoNaPonta(CorLinha::Vermelho, false) == AcaoCruzamento::MeiaVolta);
  assert(acaoNaPonta(CorLinha::Azul, false) == AcaoCruzamento::MeiaVolta);
  assert(acaoNaPonta(CorLinha::Vermelho, true) == AcaoCruzamento::Navegador);
  assert(acaoNaPonta(CorLinha::Azul, true) == AcaoCruzamento::Navegador);

  const CorLinha igual[3] = {CorLinha::Vermelho, CorLinha::Vermelho, CorLinha::Vermelho};
  const CorLinha mistura[3] = {CorLinha::Vermelho, CorLinha::Outra, CorLinha::Vermelho};
  assert(corConfirmada(igual, 3) == CorLinha::Vermelho);
  assert(corConfirmada(mistura, 3) == CorLinha::Outra);   // leituras discordando: nao acredita
  assert(corConfirmada(igual, 0) == CorLinha::Outra);
  MemoriaCor m;
  assert(corRecente(m, 1000, 400) == CorLinha::Outra);
  anotarCor(m, CorLinha::Outra, 100);   // cor normal nao apaga nem grava nada
  assert(corRecente(m, 150, 400) == CorLinha::Outra);
  anotarCor(m, CorLinha::Azul, 1000);
  assert(corRecente(m, 1300, 400) == CorLinha::Azul);   // viu ha 300 ms: vale
  assert(corRecente(m, 1500, 400) == CorLinha::Outra);  // ha 500 ms: velho demais
  esquecerCor(m);
  assert(corRecente(m, 1000, 400) == CorLinha::Outra);
  std::puts("teste_borda: ok");
  return 0;
}
