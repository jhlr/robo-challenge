#pragma once
// Exploracao: por onde o robo ja passou, em cruzamentos, para escolher o lado MENOS EXPLORADO quando precisa virar (as duas pontas acesas).
// Pura. O rumo vem do MPU (bussola na grade: 0 = de frente para dentro da cidade; multiplos de 90 depois de ancorado); as coordenadas sao RELATIVAS
// ao ponto de partida (a posicao absoluta e do localizador, core/localizer.h, quando for ligado aqui: ele pode alimentar os mesmos contadores).
#include <math.h>
#include <stdint.h>

namespace seguidor {

constexpr int kRaio = 8;                        // o mapa tem 6 x 5 cruzamentos: 17 x 17 relativos cobre qualquer ponto de partida
constexpr int kLadoGrade = 2 * kRaio + 1;

struct Exploracao {
  uint8_t visitas[kLadoGrade][kLadoGrade] = {};   // [y][x], contagem saturada em 255
  int x = 0, y = 0;                               // posicao atual, em cruzamentos, relativa a partida (rumo 0 = +x)
};

inline int quartos(float rumoGraus) { return static_cast<int>(lroundf(rumoGraus / 90.0f)); }

// Anda um cruzamento na direcao do rumo e marca o ponto como visitado. Chame a cada cruzamento confirmado, ANTES de virar.
inline void registrarCruzamento(Exploracao& e, float rumoGraus) {
  const int q = ((quartos(rumoGraus) % 4) + 4) % 4;   // 0 = +x, 1 = +y (esquerda), 2 = -x, 3 = -y
  static const int dx[4] = {1, 0, -1, 0}, dy[4] = {0, 1, 0, -1};
  e.x += dx[q];
  e.y += dy[q];
  if (e.x < -kRaio) e.x = -kRaio;
  if (e.x > kRaio) e.x = kRaio;
  if (e.y < -kRaio) e.y = -kRaio;
  if (e.y > kRaio) e.y = kRaio;
  uint8_t& v = e.visitas[e.y + kRaio][e.x + kRaio];
  if (v < 255) v++;
}

// Quantos cruzamentos ja visitados ha de um lado do robo (a metade do mapa a esquerda ou a direita da direcao em que ele aponta).
inline int visitadosDoLado(const Exploracao& e, float rumoGraus, int lado) {
  const int q = ((quartos(rumoGraus) % 4) + 4) % 4;
  static const int fx[4] = {1, 0, -1, 0}, fy[4] = {0, 1, 0, -1};   // frente
  const int lx = -fy[q], ly = fx[q];                              // esquerda
  int soma = 0;
  for (int yy = -kRaio; yy <= kRaio; yy++)
    for (int xx = -kRaio; xx <= kRaio; xx++) {
      const int lateral = (xx - e.x) * lx + (yy - e.y) * ly;     // >0: a esquerda do robo
      if (lado > 0 ? lateral > 0 : lateral < 0) soma += e.visitas[yy + kRaio][xx + kRaio] ? 1 : 0;
    }
  return soma;
}

// +1 = esquerda, -1 = direita: o lado com MENOS cruzamentos visitados. Empate (ou rumo desconhecido): o lado da ultima correcao; sem ele, esquerda.
inline int ladoMenosExplorado(const Exploracao& e, float rumoGraus, bool rumoValido, int ultimoLado) {
  const int fallback = ultimoLado != 0 ? ultimoLado : 1;
  if (!rumoValido) return fallback;
  const int esq = visitadosDoLado(e, rumoGraus, 1), dir = visitadosDoLado(e, rumoGraus, -1);
  if (esq == dir) return fallback;
  return esq < dir ? 1 : -1;
}

}  // namespace seguidor
