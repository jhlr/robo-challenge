// g++ -std=c++17 -Wall -Isrc test/teste_borda.cpp -o /tmp/teste_borda && /tmp/teste_borda
#include <cassert>
#include <cstdio>

#include "seguidor/borda.h"

using namespace seguidor;

int main() {
  assert(acaoNaPonta(CorLinha::Azul, true, true) == AcaoCruzamento::Portao);       // saindo com cubo pelo NOSSO portao (entrega): portao, nao borda
  assert(acaoNaPonta(CorLinha::Azul, true, false) == AcaoCruzamento::Navegador);   // com cubo na azul do adversario: nao e portao
  assert(acaoNaPonta(CorLinha::Outra, false) == AcaoCruzamento::Cruzamento);
  assert(acaoNaPonta(CorLinha::Outra, true) == AcaoCruzamento::Cruzamento);   // cubo na mao nao muda um cruzamento normal
  assert(acaoNaPonta(CorLinha::Vermelho, false) == AcaoCruzamento::MeiaVolta);
  assert(acaoNaPonta(CorLinha::Azul, false) == AcaoCruzamento::MeiaVolta);
  assert(acaoNaPonta(CorLinha::Vermelho, true) == AcaoCruzamento::Navegador);
  assert(acaoNaPonta(CorLinha::Azul, true) == AcaoCruzamento::Navegador);

  assert(acaoNaPonta(CorLinha::Azul, false, true) == AcaoCruzamento::Portao);     // primeira azul: nosso portao
  assert(acaoNaPonta(CorLinha::Azul, false, false) == AcaoCruzamento::MeiaVolta); // azul depois do portao: do adversario
  assert(acaoNaPonta(CorLinha::Vermelho, false, true) == AcaoCruzamento::MeiaVolta);   // vermelho nunca e portao
  assert(acaoNaPonta(CorLinha::Outra, false, true) == AcaoCruzamento::Cruzamento);
  const CorLinha igual[3] = {CorLinha::Vermelho, CorLinha::Vermelho, CorLinha::Vermelho};
  const CorLinha mistura[3] = {CorLinha::Vermelho, CorLinha::Outra, CorLinha::Vermelho};
  assert(corConfirmada(igual, 3) == CorLinha::Vermelho);
  assert(corConfirmada(mistura, 3) == CorLinha::Outra);   // leituras discordando: nao acredita
  assert(corConfirmada(igual, 0) == CorLinha::Outra);
  assert(reposicionadoPeloVerde(CorLinha::Verde, false, false));    // verde depois do portao, sem entregar: foi reposicionado
  assert(!reposicionadoPeloVerde(CorLinha::Verde, true, false));    // ainda nao passou pelo portao: e o verde de partida
  assert(!reposicionadoPeloVerde(CorLinha::Verde, false, true));    // entregando o cubo de proposito
  assert(!reposicionadoPeloVerde(CorLinha::Azul, false, false));
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
