#pragma once
// Confirmar o cruzamento girando no eixo (nao toca em pino nem em millis(): so conta, entao testa no PC). NAO depende da largura da linha.
// O robo para com S1 e S8 no preto, gira giroGraus (90) para a esquerda, depois para a direita, e le a barra. A 90 graus a barra fica ao longo da
// rua do lado: se ha rua ali, algum sensor acende. Cruzamento = existe rua dos DOIS lados (uma mancha ou ruido nao tem rua do lado).
// Vale em QUALQUER situacao em que as duas pontas acendem, inclusive perdido/buscando (a barra acende inteira ao ficar paralela a uma linha):
//   - vinha seguindo a linha  -> cruzamento se ha rua dos dois lados (senao, falso alarme);
//   - estava perdido/buscando -> era o robo de lado em cima da linha: depois de virar 90 graus o padrao fica ESTREITO (ele endireitou) e ele segue por ela.
// A varredura comeca pelo lado pedido (o MENOS EXPLORADO do mapa, ver exploracao.h) e a maquina de estados fica aqui, pura: o sketch e o simulador so
// alimentam leituras e motores (nao duplicam o giro).
// O angulo vem de contar tempo girando (grausPorSegundoBusca), sem referencia na pista; por isso a janela em volta de 90 graus e larga.
#include <stdint.h>

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
  int estreitoMax = 3;         // padrao estreito (uma linha cruzando a barra): de 1 a isto acesos. Nao exige o centro: com a barra sobre a linha o eixo fica ~30 mm ao lado dela
  int acesosMaxRua = 6;        // mais sensores acesos que isto a 90 graus nao e uma rua (e mancha): uma rua cruza a barra em poucos sensores
};

struct ResultadoGiro {
  bool cruzamento;   // chegou com as duas pontas no preto e ha rua dos dois lados
  int ruaEsquerda;   // a 90 graus a esquerda algum sensor acendeu: ha rua? 1 = sim, 0 = nao, -1 = nao mediu
  int ruaDireita;
  bool estreitoEsq;  // a 90 graus a esquerda o padrao ficou estreito (uma linha cruzando a barra): ele endireitou nela
  bool estreitoDir;
};

inline bool padraoEstreito(int acesos, const AjustesGiroCruzamento& aj) { return acesos >= 1 && acesos <= aj.estreitoMax; }

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

// Perto de giroGraus a esquerda (ou a direita): o padrao ficou estreito em alguma amostra?
inline bool estreitoNoAngulo(const AmostraGiro* a, int n, bool esquerda, const AjustesGiroCruzamento& aj) {
  for (int i = 0; i < n; i++) {
    const float g = esquerda ? a[i].graus : -a[i].graus;
    if (g < aj.giroGraus * (1.0f - aj.tolerancia) || g > aj.giroGraus * (1.0f + aj.tolerancia)) continue;
    if (padraoEstreito(a[i].acesos, aj)) return true;
  }
  return false;
}
}  // namespace detalhe

// `a` tem as amostras do giro inteiro, em qualquer ordem. Precisa ter pelo menos uma amostra em ~0 grau com S1 e S8 pretos (chegou no cruzamento).
inline ResultadoGiro avaliarGiro(const AmostraGiro* a, int n, const AjustesGiroCruzamento& aj = {}) {
  bool chegou = false;
  for (int i = 0; i < n; i++) {
    if (a[i].graus > -1.0f && a[i].graus < 1.0f && a[i].s1 && a[i].s8) chegou = true;
  }
  ResultadoGiro r{false, detalhe::ruaNoAngulo(a, n, true, aj), detalhe::ruaNoAngulo(a, n, false, aj), detalhe::estreitoNoAngulo(a, n, true, aj), detalhe::estreitoNoAngulo(a, n, false, aj)};
  r.cruzamento = chegou && r.ruaEsquerda == 1 && r.ruaDireita == 1;
  return r;
}

// O que as duas pontas acesas eram.
enum class Oito : uint8_t {
  Cruzamento,   // vinha seguindo a linha e ha rua dos dois lados
  Endireitou,   // estava perdido/buscando e, virado 90 graus, o padrao ficou estreito e centrado: era ele de lado em cima da linha; segue por ela
  FalsoAlarme,  // mancha, ruido, ou nada disso
};

inline Oito classificarOito(bool vinhaSeguindo, const ResultadoGiro& r) {
  if (vinhaSeguindo) return r.cruzamento ? Oito::Cruzamento : Oito::FalsoAlarme;
  return (r.estreitoEsq || r.estreitoDir) ? Oito::Endireitou : Oito::FalsoAlarme;
}

// ---------------------------------------------------------------- a varredura, passo a passo (pura: o sketch e o simulador so alimentam leituras e motores)
//   Seguindo a linha: gira ate +lado*90, depois ate -lado*90, e volta a 0; no fim avaliarGiro diz se ha rua dos dois lados.
//   Perdido/buscando: gira ate +lado*90; se o padrao ficou estreito, PARA ali (endireitou). Senao vai ate -lado*90 (olha o outro lado);
//   se la tambem nao, volta a 0 (falso alarme).
struct Varredura {
  enum Fase : uint8_t { Parada, AteLado, AteOutro, Volta, Pronta } fase = Parada;
  int lado = 1;                // +1 = comeca para a esquerda, -1 = para a direita
  bool vinhaSeguindo = true;
  unsigned long faseDesde = 0; // +1
  int n = 0;                   // amostras guardadas
  Oito resultado = Oito::FalsoAlarme;
  ResultadoGiro ruas{false, -1, -1, false, false};
  int ladoEndireitou = 0;      // +1 esquerda, -1 direita, 0 nao
};

struct ComandoVarredura {
  int esq, dir;    // gira no eixo (sentidos opostos)
  bool terminou;
};

inline void iniciarVarredura(Varredura& v, int lado, bool vinhaSeguindo) {
  v = Varredura{};
  v.lado = lado >= 0 ? 1 : -1;
  v.vinhaSeguindo = vinhaSeguindo;
  v.fase = Varredura::AteLado;
}

// `leitura.graus` = angulo desde o INICIO da varredura (+ esquerda), do giroscopio ou estimado. Guarda a leitura em `buf` (capacidade `cap`).
// `velGiro` = PWM do giro (ja fora da zona morta). Cada fase tem 3 s no maximo (nao fica girando para sempre se o angulo nao anda).
inline ComandoVarredura passoVarredura(Varredura& v, const AmostraGiro& leitura, unsigned long agoraMs, int velGiro, const AjustesGiroCruzamento& aj, AmostraGiro* buf, int cap) {
  if (v.fase == Varredura::Parada || v.fase == Varredura::Pronta) return {0, 0, true};
  if (v.n < cap) buf[v.n++] = leitura;
  if (v.faseDesde == 0) v.faseDesde = agoraMs + 1;
  const bool estourou = agoraMs + 1 - v.faseDesde > 3000;
  const float g = leitura.graus * v.lado;   // positivo = para o lado em que comecou
  auto para = [&](Varredura::Fase f) { v.fase = f; v.faseDesde = 0; };
  auto girar = [&](int sinal) -> ComandoVarredura {   // sinal +1 = esquerda
    return sinal > 0 ? ComandoVarredura{-velGiro, velGiro, false} : ComandoVarredura{velGiro, -velGiro, false};
  };
  auto terminar = [&](bool endireitou) {
    v.ruas = avaliarGiro(buf, v.n, aj);
    v.resultado = endireitou ? Oito::Endireitou : classificarOito(v.vinhaSeguindo, v.ruas);
    v.fase = Varredura::Pronta;
  };

  switch (v.fase) {
    case Varredura::AteLado:
      if (g >= aj.giroGraus || estourou) {
        if (!v.vinhaSeguindo && padraoEstreito(leitura.acesos, aj)) { v.ladoEndireitou = v.lado; terminar(true); return {0, 0, true}; }
        para(Varredura::AteOutro);
      }
      return girar(v.lado);
    case Varredura::AteOutro:
      if (g <= -aj.giroGraus || estourou) {
        if (!v.vinhaSeguindo && padraoEstreito(leitura.acesos, aj)) { v.ladoEndireitou = -v.lado; terminar(true); return {0, 0, true}; }
        para(Varredura::Volta);
      }
      return girar(-v.lado);
    case Varredura::Volta:
      if (g >= 0.0f || estourou) { terminar(false); return {0, 0, true}; }
      return girar(v.lado);
    default:
      return {0, 0, true};
  }
}

}  // namespace seguidor
