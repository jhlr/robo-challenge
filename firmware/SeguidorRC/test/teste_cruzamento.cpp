// Teste do cruzamento no PC: g++ -std=c++17 -Wall -Isrc test/teste_cruzamento.cpp -o /tmp/teste_cruz && /tmp/teste_cruz
#include <cassert>
#include <cstdio>

#include "seguidor/cruzamento.h"

using namespace seguidor;

static bool passo(MemoriaCruzamento& m, const char* padrao, unsigned long ms) {
  bool p[kSensores];
  for (int i = 0; i < kSensores; i++) p[i] = padrao[i] == '#';
  return cruzamentoConfirmado(p, ms, AjustesCruzamento{}, m);
}

int main() {
  { MemoriaCruzamento m;   // precisa de 2 leituras seguidas
    assert(!passo(m, "########", 0));
    assert(passo(m, "########", 10)); }
  { MemoriaCruzamento m;   // um sensor fora zera a contagem
    assert(!passo(m, "########", 0));
    assert(!passo(m, "#######.", 10));
    assert(!passo(m, "########", 20));
    assert(passo(m, "########", 30)); }
  { MemoriaCruzamento m;   // conta uma vez so enquanto sai
    passo(m, "########", 0);
    assert(passo(m, "########", 10));
    assert(!passo(m, "########", 20));
    assert(!passo(m, "...##...", 100));   // saiu do preto, mas cedo demais
    passo(m, "########", 150);
    assert(!passo(m, "########", 160));   // janela ainda aberta
    assert(!passo(m, "...##...", 400));   // janela fecha
    passo(m, "########", 410);
    assert(passo(m, "########", 420)); }
  { MemoriaCruzamento m;   // ficou parado: o relogio de saida recomeca ao voltar a andar
    passo(m, "########", 0);
    passo(m, "########", 10);
    comecarASair(m, 2000);
    assert(!passo(m, "...##...", 2100));
    assert(!passo(m, "...##...", 2400));
    passo(m, "########", 2410);
    assert(passo(m, "########", 2420)); }
  std::puts("teste_cruzamento: ok");
  return 0;
}
