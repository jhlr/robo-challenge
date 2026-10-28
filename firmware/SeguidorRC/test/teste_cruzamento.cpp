// Teste do cruzamento no PC: g++ -std=c++17 -Wall -Isrc test/teste_cruzamento.cpp -o /tmp/teste_cruz && /tmp/teste_cruz
#include <cassert>
#include <cstdio>

#include "seguidor/cruzamento.h"
#include "seguidor/saude.h"

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
  { MemoriaCruzamento m;   // so as pontas bastam: S1 e S8, sem os do meio
    assert(!passo(m, "#......#", 0));
    assert(passo(m, "#......#", 10)); }
  { MemoriaCruzamento m;   // uma ponta sozinha nao e cruzamento
    assert(!passo(m, "#.......", 0));
    assert(!passo(m, "#.......", 10));
    assert(!passo(m, ".......#", 20));
    assert(!passo(m, ".......#", 30)); }
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
  // ---- o que a linha sumida significa ----
  { AjustesCruzamento aj;   // beco: 127 mm a 250 mm/s = 508 ms
    assert(classificarPerda(0, 700, 120, aj) == Perda::Nenhuma);
    assert(classificarPerda(60, 700, 120, aj) == Perda::Lacuna);                       // falha curta
    assert(classificarPerda(200, 508 + 200, 120, aj) == Perda::FimDeBeco);             // sumiu ~508 ms depois do cruzamento
    assert(classificarPerda(200, 1500 + 200, 120, aj) == Perda::Perdida);              // sumiu muito depois: nao e beco
    assert(classificarPerda(200, 150 + 200, 120, aj) == Perda::Perdida); }             // sumiu cedo demais: nao e beco
  // ---- sensor travado no preto ----
  { SaudeSensores s; bool ok[kSensores]; for (bool& o : ok) o = true;
    bool b[kSensores] = {true, false, false, false, false, false, false, false};
    assert(atualizarSaude(b, 0, s, ok) == 0 && ok[0]);
    assert(atualizarSaude(b, 29000, s, ok) == 0 && ok[0]);
    assert(atualizarSaude(b, 31000, s, ok) == 1 && !ok[0]);      // 31 s seguidos: marcado
    assert(atualizarSaude(b, 62000, s, ok) == 0); }               // ja marcado: nao conta de novo
  { SaudeSensores s; bool ok[kSensores]; for (bool& o : ok) o = true;   // piscar nao conta como travado
    bool preto[kSensores] = {true, true, true, true, true, true, true, true};
    bool branco[kSensores] = {};
    for (unsigned long t = 0; t < 100000; t += 1000) atualizarSaude((t / 1000) % 2 ? preto : branco, t, s, ok);
    for (bool o : ok) assert(o); }
  std::puts("teste_cruzamento: ok");
  return 0;
}
