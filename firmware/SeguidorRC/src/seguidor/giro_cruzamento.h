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
  float us = -1.0f;   // ultrassom a frente, em mm (-1 = sem eco ou sem sensor)
};

struct AjustesGiroCruzamento {
  float giroGraus = 90.0f;     // quanto gira para cada lado
  float tolerancia = 0.2f;     // as amostras entre giroGraus*(1-tol) e giroGraus*(1+tol) contam como "a 90 graus"
  int estreitoMax = 3;         // padrao estreito (uma linha cruzando a barra): de 1 a isto acesos. Nao exige o centro: com a barra sobre a linha o eixo fica ~30 mm ao lado dela
  int acesosMaxRua = 6;
  // O robo gira sobre o eixo e a frente (170 mm de largura) varre ~130 mm: um cubo a ~110 mm do cruzamento na rua do lado seria atingido pelo giro. Se o ultrassom, apontando
  // para o lado, ve algo a menos disto, PARA de girar para esse lado (ha rua E tem cubo ali: nao empurra, o cubo entra na memoria pelo eco).
  float usParaGiroMm = 95.0f;
  float usParaGiroDepoisDeGraus = 25.0f;   // so vale depois de girar isto (a frente, antes de girar, e a rua de onde veio)        // mais sensores acesos que isto a 90 graus nao e uma rua (e mancha): uma rua cruza a barra em poucos sensores
};

struct ResultadoGiro {
  bool cruzamento;   // chegou com as duas pontas no preto e ha rua dos dois lados
  int ruaEsquerda;   // a 90 graus a esquerda algum sensor acendeu: ha rua? 1 = sim, 0 = nao, -1 = nao mediu
  int ruaDireita;
  bool estreitoEsq;  // a 90 graus a esquerda o padrao ficou estreito (uma linha cruzando a barra): ele endireitou nela
  bool estreitoDir;
  float usEsq = -1.0f;     // menor distancia do ultrassom perto de 90 graus a esquerda (a rua da esquerda), -1 = nada ou nao mediu
  float usDir = -1.0f;     // idem a direita
  float usFrente = -1.0f;  // e perto de 0 grau (a rua em frente), olhando tambem a chegada e a volta
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

// Menor eco do ultrassom perto de `centroGraus` (+- janela): a distancia ate o que esta ao longo daquela rua. -1 se nenhuma leitura teve eco.
inline float usNoAngulo(const AmostraGiro* a, int n, float centroGraus, float janelaGraus) {
  float melhor = -1.0f;
  for (int i = 0; i < n; i++) {
    if (a[i].us <= 0.0f || fabsf(a[i].graus - centroGraus) > janelaGraus) continue;
    if (melhor < 0.0f || a[i].us < melhor) melhor = a[i].us;
  }
  return melhor;
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
  ResultadoGiro r{false, detalhe::ruaNoAngulo(a, n, true, aj), detalhe::ruaNoAngulo(a, n, false, aj), detalhe::estreitoNoAngulo(a, n, true, aj), detalhe::estreitoNoAngulo(a, n, false, aj),
                 detalhe::usNoAngulo(a, n, aj.giroGraus, aj.giroGraus * aj.tolerancia), detalhe::usNoAngulo(a, n, -aj.giroGraus, aj.giroGraus * aj.tolerancia), detalhe::usNoAngulo(a, n, 0.0f, 12.0f)};
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
//   SO UM LADO (soUmLado, so seguindo a linha): para confirmar o cruzamento basta UMA rua perpendicular: se a 90 graus do lado pedido ha rua, ja e cruzamento e volta a 0 sem olhar
//   o outro lado (o que nao mediu fica -1 e o ultrassom dele -2 = "nao mediu"). Sem rua ali, segue como a varredura completa. Completa quando o robo precisa VER as duas ruas ou escolher uma.
//   Perdido/buscando: gira ate +lado*90; se o padrao ficou estreito, PARA ali (endireitou). Senao vai ate -lado*90 (olha o outro lado);
//   se la tambem nao, volta a 0 (falso alarme).
struct Varredura {
  enum Fase : uint8_t { Parada, AteLado, AteOutro, Volta, Pronta } fase = Parada;
  bool soUmLado = false;       // confirma com uma rua so (ver acima)
  bool voltaDoPositivo = false;   // a Volta parte do lado em que comecou (+): gira para o outro sentido
  int lado = 1;                // +1 = comeca para a esquerda, -1 = para a direita
  bool vinhaSeguindo = true;
  unsigned long faseDesde = 0; // +1
  int n = 0;                   // amostras guardadas
  Oito resultado = Oito::FalsoAlarme;
  ResultadoGiro ruas{false, -1, -1, false, false, -1.0f, -1.0f, -1.0f};
  int ladoEndireitou = 0;      // +1 esquerda, -1 direita, 0 nao
  bool parouPorCubo[2] = {false, false};   // [0] = o lado em que comecou, [1] = o outro: parou de girar porque o ultrassom viu um cubo perto
};

struct ComandoVarredura {
  int esq, dir;    // gira no eixo (sentidos opostos)
  bool terminou;
};

inline void iniciarVarredura(Varredura& v, int lado, bool vinhaSeguindo, bool soUmLado = false) {
  v = Varredura{};
  v.soUmLado = soUmLado && vinhaSeguindo;
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
    // parou por causa de um cubo perto: ha rua ali (e ha cubo nela), mesmo sem ter chegado aos 90 graus para ler a linha
    if (v.parouPorCubo[0]) { (v.lado > 0 ? v.ruas.ruaEsquerda : v.ruas.ruaDireita) = 1; }
    if (v.parouPorCubo[1]) { (v.lado > 0 ? v.ruas.ruaDireita : v.ruas.ruaEsquerda) = 1; }
    v.ruas.cruzamento = v.ruas.cruzamento || (v.vinhaSeguindo && v.ruas.ruaEsquerda == 1 && v.ruas.ruaDireita == 1 && (v.parouPorCubo[0] || v.parouPorCubo[1]));
    if (v.soUmLado && v.voltaDoPositivo) {   // confirmou com a rua do primeiro lado: o outro nao foi medido
      const int primeiro = v.lado > 0 ? v.ruas.ruaEsquerda : v.ruas.ruaDireita;
      v.ruas.cruzamento = primeiro == 1;
      (v.lado > 0 ? v.ruas.usDir : v.ruas.usEsq) = -2.0f;
    }
    v.resultado = endireitou ? Oito::Endireitou : classificarOito(v.vinhaSeguindo, v.ruas);
    v.fase = Varredura::Pronta;
  };

  switch (v.fase) {
    case Varredura::AteLado:
      if (leitura.us > 0.0f && leitura.us <= aj.usParaGiroMm && g > aj.usParaGiroDepoisDeGraus) {
        v.parouPorCubo[0] = true;
        if (v.soUmLado) { v.voltaDoPositivo = true; para(Varredura::Volta); return girar(-v.lado); }   // ha rua (com cubo): basta
        para(Varredura::AteOutro);
        return girar(-v.lado);
      }
      if (g >= aj.giroGraus || estourou) {
        if (!v.vinhaSeguindo && padraoEstreito(leitura.acesos, aj)) { v.ladoEndireitou = v.lado; terminar(true); return {0, 0, true}; }
        if (v.soUmLado && leitura.acesos >= 1 && leitura.acesos <= aj.acesosMaxRua) { v.voltaDoPositivo = true; para(Varredura::Volta); return girar(-v.lado); }   // rua a 90 graus: e cruzamento
        para(Varredura::AteOutro);
      }
      return girar(v.lado);
    case Varredura::AteOutro:
      if (leitura.us > 0.0f && leitura.us <= aj.usParaGiroMm && g < -aj.usParaGiroDepoisDeGraus) { v.parouPorCubo[1] = true; para(Varredura::Volta); return girar(v.lado); }
      if (g <= -aj.giroGraus || estourou) {
        if (!v.vinhaSeguindo && padraoEstreito(leitura.acesos, aj)) { v.ladoEndireitou = -v.lado; terminar(true); return {0, 0, true}; }
        para(Varredura::Volta);
      }
      return girar(-v.lado);
    case Varredura::Volta:
      if (v.voltaDoPositivo ? g <= 0.0f : g >= 0.0f) { terminar(false); return {0, 0, true}; }
      if (estourou) { terminar(false); return {0, 0, true}; }
      return girar(v.voltaDoPositivo ? -v.lado : v.lado);
    default:
      return {0, 0, true};
  }
}

}  // namespace seguidor
