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
  { // classificacao: vinha seguindo -> cruzamento so com rua dos dois lados; perdido -> endireitou se ficou estreito e centrado depois de virar
    auto v = simular(true, true, 5);   // ruas largas a 90 graus (5 acesos), nada estreito
    const ResultadoGiro cruz = avaliarGiro(v.data(), (int)v.size(), aj);
    assert(classificarOito(true, cruz) == Oito::Cruzamento);
    assert(classificarOito(false, cruz) == Oito::FalsoAlarme);   // perdido e SEM estreito: nao endireitou
    for (auto& s : v) if (s.graus > 70.0f) s.acesos = 2;   // a 90 graus a esquerda: 2 acesos (uma linha cruzando a barra)
    const ResultadoGiro lado = avaliarGiro(v.data(), (int)v.size(), aj);
    assert(lado.estreitoEsq && !lado.estreitoDir);
    assert(classificarOito(false, lado) == Oito::Endireitou);
    assert(classificarOito(true, lado) == Oito::Cruzamento);     // vinha seguindo: o que vale e a rua dos dois lados
    assert(!padraoEstreito(8, aj) && !padraoEstreito(0, aj) && padraoEstreito(2, aj)); }
  { // a varredura passo a passo, com um robo ficticio que gira a 100 graus/s
    AjustesGiroCruzamento a;
    bool soUm = false;
    auto rodar = [&](bool vinha, int lado, bool ruaEsq, bool ruaDir, bool estreitoEsq, bool estreitoDir, Varredura& v) {
      static AmostraGiro buf[2000];
      iniciarVarredura(v, lado, vinha, soUm);
      float graus = 0.0f;
      for (unsigned long t = 0; t < 20000; t += 5) {
        AmostraGiro l{graus, std::fabs(graus) < 1.0f, std::fabs(graus) < 1.0f, 0};
        if (std::fabs(graus) < 1.0f) l.acesos = 8;
        else if (graus > 70.0f) l.acesos = estreitoEsq ? 2 : (ruaEsq ? 5 : 0);   // rua sem estreito: 5 acesos (uma rua larga, nao uma linha cruzando)
        else if (graus < -70.0f) l.acesos = estreitoDir ? 2 : (ruaDir ? 5 : 0);
        const ComandoVarredura c = passoVarredura(v, l, t, 80, a, buf, 2000);
        if (c.terminou) return graus;
        graus += (c.dir > 0 ? 1.0f : -1.0f) * 100.0f * 0.005f;   // dir > esq (e dir > 0) = gira para a esquerda
      }
      return 9999.0f;
    };
    Varredura v;
    float fim = rodar(true, 1, true, true, false, false, v);              // vinha seguindo, ruas dos dois lados
    assert(v.resultado == Oito::Cruzamento && std::fabs(fim) < 2.0f);     // voltou ao rumo de chegada
    fim = rodar(true, -1, true, true, false, false, v);                   // comecando pela direita: o mesmo resultado, tambem volta a 0
    assert(v.resultado == Oito::Cruzamento && std::fabs(fim) < 2.0f);
    fim = rodar(true, 1, false, false, false, false, v);                  // mancha
    assert(v.resultado == Oito::FalsoAlarme && std::fabs(fim) < 2.0f);
    fim = rodar(false, 1, false, false, true, false, v);                  // perdido: estreito ja no 1o lado -> PARA ali
    assert(v.resultado == Oito::Endireitou && v.ladoEndireitou == 1 && fim > 80.0f && fim < 100.0f);
    fim = rodar(false, 1, false, false, false, true, v);                  // so no outro lado
    assert(v.resultado == Oito::Endireitou && v.ladoEndireitou == -1 && fim < -80.0f);
    fim = rodar(false, -1, false, false, true, false, v);                 // comecando pela direita: o lado esquerdo e o "outro"
    assert(v.resultado == Oito::Endireitou && v.ladoEndireitou == 1 && fim > 80.0f);
    fim = rodar(false, 1, true, true, false, false, v);                   // perdido, ruas mas nada estreito: falso alarme, volta a 0
    assert(v.resultado == Oito::FalsoAlarme && std::fabs(fim) < 2.0f);
    // SO UM LADO: uma rua perpendicular basta (nao olha o outro lado) e ele volta a 0
    soUm = true;
    fim = rodar(true, 1, true, false, false, false, v);                   // rua so na esquerda, comeca por ela: ja e cruzamento
    assert(v.resultado == Oito::Cruzamento && std::fabs(fim) < 2.0f && v.ruas.ruaEsquerda == 1 && v.ruas.ruaDireita == -1 && v.ruas.usDir == -2.0f);
    fim = rodar(true, -1, false, true, false, false, v);                  // comeca pela direita: idem
    assert(v.resultado == Oito::Cruzamento && std::fabs(fim) < 2.0f && v.ruas.ruaDireita == 1 && v.ruas.ruaEsquerda == -1 && v.ruas.usEsq == -2.0f);
    fim = rodar(true, 1, false, true, false, false, v);                   // sem rua no 1o lado: olha o outro (varredura completa)
    assert(v.resultado == Oito::FalsoAlarme && v.ruas.ruaDireita == 1 && std::fabs(fim) < 2.0f);
    fim = rodar(true, 1, false, false, false, false, v);                  // mancha: falso alarme
    assert(v.resultado == Oito::FalsoAlarme && std::fabs(fim) < 2.0f);
    soUm = false; }
  { // ultrassom: o eco mais perto perto de +90, -90 e 0 graus vira a distancia ate o que esta em cada rua
    std::vector<AmostraGiro> v;
    for (float g = -90; g <= 90.01f; g += 1.0f) {
      AmostraGiro a{g, false, false, 0};
      if (g > 80.0f) a.us = 190.0f;                  // cubo na rua da esquerda
      if (g > 85.0f) a.us = 170.0f;                  // o menor vale
      if (std::fabs(g) < 6.0f) a.us = 640.0f;        // a rua em frente: so o fundo
      v.push_back(a);
    }
    const ResultadoGiro r = avaliarGiro(v.data(), (int)v.size(), aj);
    assert(r.usEsq == 170.0f && r.usDir < 0.0f && r.usFrente == 640.0f); }
  std::puts("teste_giro_cruzamento: ok");
  return 0;
}
