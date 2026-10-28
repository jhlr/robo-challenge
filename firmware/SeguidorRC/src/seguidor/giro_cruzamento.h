#pragma once
// Confirmar o cruzamento girando no eixo (nao toca em pino nem em millis(): so conta, entao testa no PC). NAO depende da largura da linha.
// O robo para com S1 e S8 no preto, gira giroGraus (90) para a esquerda, depois para a direita, e le a barra. A 90 graus a barra fica ao longo da
// rua do lado: se ha rua ali, algum sensor acende. Cruzamento = existe rua dos DOIS lados (uma mancha ou ruido nao tem rua do lado).
// O angulo vem de contar tempo girando (grausPorSegundoBusca), sem referencia na pista; por isso a janela em volta de 90 graus e larga.
#include "decisao.h"   // kSensores

namespace seguidor {

struct AmostraGiro {
  float graus;   // angulo estimado desde o inicio do giro: + = esquerda, - = direita (0 = como chegou)
  bool s1;       // S1 (ponta esquerda) no preto
  bool s8;       // S8 (ponta direita) no preto
  int acesos;    // quantos sensores da barra estao no preto
};

struct AjustesGiroCruzamento {
  float giroGraus = 90.0f;     // quanto gira para cada lado
  float tolerancia = 0.2f;     // as amostras entre giroGraus*(1-tol) e giroGraus*(1+tol) contam como "a 90 graus"
  int acesosMaxRua = 6;        // mais sensores acesos que isto a 90 graus nao e uma rua (e mancha): uma rua cruza a barra em poucos sensores
};

struct ResultadoGiro {
  bool cruzamento;   // chegou com as duas pontas no preto e ha rua dos dois lados
  int ruaEsquerda;   // a 90 graus a esquerda algum sensor acendeu: ha rua? 1 = sim, 0 = nao, -1 = nao mediu
  int ruaDireita;
};

namespace detalhe {
// Perto de giroGraus a esquerda (ou a direita): algum sensor acendeu, e sem ser mancha? -1 se nenhuma amostra chegou la.
inline int ruaNoAngulo(const AmostraGiro* a, int n, bool esquerda, const AjustesGiroCruzamento& aj) {
  bool mediu = false, rua = false;
  for (int i = 0; i < n; i++) {
    const float g = esquerda ? a[i].graus : -a[i].graus;
    if (g < aj.giroGraus * (1.0f - aj.tolerancia) || g > aj.giroGraus * (1.0f + aj.tolerancia)) continue;
    mediu = true;
    if (a[i].acesos >= 1 && a[i].acesos <= aj.acesosMaxRua) rua = true;
  }
  return !mediu ? -1 : (rua ? 1 : 0);
}
}  // namespace detalhe

// `a` tem as amostras do giro inteiro, em qualquer ordem. Precisa ter pelo menos uma amostra em ~0 grau com S1 e S8 pretos (chegou no cruzamento).
inline ResultadoGiro avaliarGiro(const AmostraGiro* a, int n, const AjustesGiroCruzamento& aj = {}) {
  bool chegou = false;
  for (int i = 0; i < n; i++) {
    if (a[i].graus > -1.0f && a[i].graus < 1.0f && a[i].s1 && a[i].s8) chegou = true;
  }
  ResultadoGiro r{false, detalhe::ruaNoAngulo(a, n, true, aj), detalhe::ruaNoAngulo(a, n, false, aj)};
  r.cruzamento = chegou && r.ruaEsquerda == 1 && r.ruaDireita == 1;
  return r;
}

}  // namespace seguidor
