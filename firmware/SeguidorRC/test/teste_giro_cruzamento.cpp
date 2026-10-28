// Teste do giro de confirmacao no PC: g++ -std=c++17 -Wall -Isrc test/teste_giro_cruzamento.cpp -o /tmp/teste_giro && /tmp/teste_giro
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "seguidor/giro_cruzamento.h"

using namespace seguidor;

// Amostras de um giro de -90 a +90: S1 e S8 pretos perto de 0; a 90 graus acendem 2 sensores se a rua do lado existe.
static std::vector<AmostraGiro> simular(bool ruaEsq, bool ruaDir, int acesosNaRua = 2) {
  std::vector<AmostraGiro> v;
  for (float g = -90; g <= 90.01f; g += 0.5f) {
    const bool perto = std::fabs(g) < 1.0f;
    const bool rua = (g > 70.0f && ruaEsq) || (g < -70.0f && ruaDir);
    v.push_back({g, perto, perto, perto ? 8 : (rua ? acesosNaRua : 0)});
  }
  return v;
}

int main() {
  const AjustesGiroCruzamento aj;
  { auto v = simular(true, true);   // cruzamento de verdade
    const ResultadoGiro r = avaliarGiro(v.data(), (int)v.size(), aj);
    assert(r.cruzamento && r.ruaEsquerda == 1 && r.ruaDireita == 1); }
  { auto v = simular(true, false);   // so existe a rua da esquerda: nao e cruzamento, mas diz qual rua existe
    const ResultadoGiro r = avaliarGiro(v.data(), (int)v.size(), aj);
    assert(!r.cruzamento && r.ruaEsquerda == 1 && r.ruaDireita == 0); }
  { auto v = simular(false, false);   // mancha: nada do lado
    assert(!avaliarGiro(v.data(), (int)v.size(), aj).cruzamento); }
  { auto v = simular(true, true, kSensores);   // a barra inteira acesa a 90 graus: mancha larga, nao rua
    const ResultadoGiro r = avaliarGiro(v.data(), (int)v.size(), aj);
    assert(!r.cruzamento && r.ruaEsquerda == 0 && r.ruaDireita == 0); }
  { auto v = simular(true, true);   // sem as pontas acesas no comeco: nem chegou no cruzamento
    for (auto& s : v) { s.s1 = false; s.s8 = false; }
    assert(!avaliarGiro(v.data(), (int)v.size(), aj).cruzamento); }
  { auto v = simular(true, true);   // o angulo estimado errou 15%: ainda dentro da janela
    for (auto& s : v) s.graus *= 0.85f;
    assert(avaliarGiro(v.data(), (int)v.size(), aj).cruzamento); }
  { std::vector<AmostraGiro> v;      // giro curto que nao chega a 90 graus: nao mediu as ruas
    for (float g = -25; g <= 25; g += 0.5f) v.push_back({g, std::fabs(g) < 1.0f, std::fabs(g) < 1.0f, 8});
    const ResultadoGiro r = avaliarGiro(v.data(), (int)v.size(), aj);
    assert(!r.cruzamento && r.ruaEsquerda == -1 && r.ruaDireita == -1); }
  std::puts("teste_giro_cruzamento: ok");
  return 0;
}
