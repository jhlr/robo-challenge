#pragma once
// Assimetria dos motores com o giroscopio (pura: so conta). Com o MESMO PWM nos dois lados o carro puxa para o lado do motor mais fraco; o giroscopio mede esse giro em reta e,
// como w = (vD - vE) / bitola, a razao entre as forcas dos lados sai direto: vD/vE = (v + w*b/2) / (v - w*b/2). A compensacao e AUMENTAR a forca do lado fraco (o forte fica como esta).
// Anda em 3 rodadas curtas, alternando re e frente (o carro quase volta ao ponto de partida), cada uma ja com o ganho da anterior: a derrapagem das rodas faz o giro medido ser
// menor que o ideal, e as rodadas seguintes fecham o que sobrou. Use com o robo parado no chao, em espaco livre (~150 mm de cada lado).
#include <math.h>

#include "inercial.h"

namespace seguidor {

struct AjustesAssimetria {
  int pwm = 180;                    // o PWM de andar reto (Ajustes::velReto)
  float velMmS = 250.0f;            // velocidade real nesse PWM (AjustesCruzamento::velMmS)
  float bitolaMm = 140.0f;          // distancia entre as rodas (Ajustes::bitolaMm)
  unsigned long descartaMs = 250;   // aceleracao inicial, nao conta
  unsigned long mediaMs = 400;
  int rodadas = 3;
  float ganhoMaximo = 1.3f;         // acima disso o motor esta com defeito, nao e assimetria
};

struct Assimetria {
  enum Fase : uint8_t { Medindo, Pronto, Falhou } fase = Medindo;
  int rodada = 0;
  unsigned long desde = 0;   // +1
  float soma = 0.0f;
  int n = 0;
  float ganhoEsq = 1.0f, ganhoDir = 1.0f;   // resultado: >= 1; o lado forte fica em 1
};

struct ComandoAssimetria {
  int esq, dir;
  bool terminou;
};

// Razao vD / vE para um giro w (rad/s, + = anti-horario = a direita mais forte) medido andando a velocidade v (mm/s, negativa = de re).
inline float razaoDosLados(float w, float vMmS, float bitolaMm) {
  const float metade = w * bitolaMm / 2.0f;
  const float den = vMmS - metade;
  if (fabsf(den) < 1.0f) return 1.0f;
  return (vMmS + metade) / den;
}

// Chame a cada leitura do giroscopio (rad/s, JA sem o vies). Devolve o que mandar aos motores (os ganhos atuais ja aplicados; a ponte deve estar SEM compensacao enquanto isto roda).
inline ComandoAssimetria passoAssimetria(Assimetria& a, float gzRadS, unsigned long agoraMs, const AjustesAssimetria& aj) {
  if (a.fase != Assimetria::Medindo) return {0, 0, true};
  const bool re = (a.rodada % 2) == 0;
  const int sinal = re ? -1 : 1;
  if (a.desde == 0) a.desde = agoraMs + 1;
  const unsigned long t = agoraMs + 1 - a.desde;
  if (t >= aj.descartaMs) { a.soma += gzRadS; a.n++; }
  if (t >= aj.descartaMs + aj.mediaMs) {
    const float w = a.n > 0 ? a.soma / a.n : 0.0f;
    // o que o carro fez COM os ganhos de agora: a razao que falta corrigir, por cima do que ja se corrigiu
    // m = vD/vE medido com os ganhos de agora = R * ganhoDir / ganhoEsq, onde R e a razao das forcas dos motores SEM ganho. Logo R = m * ganhoEsq / ganhoDir.
    const float m = razaoDosLados(w, sinal * aj.velMmS, aj.bitolaMm);
    const float razao = m > 0.0f ? m * a.ganhoEsq / a.ganhoDir : 1.0f;
    if (razao >= 1.0f) { a.ganhoEsq = razao; a.ganhoDir = 1.0f; }      // direita mais forte: sobe a esquerda
    else { a.ganhoEsq = 1.0f; a.ganhoDir = 1.0f / razao; }             // esquerda mais forte: sobe a direita
    a.rodada++;
    a.desde = 0; a.soma = 0.0f; a.n = 0;
    if (a.ganhoEsq > aj.ganhoMaximo || a.ganhoDir > aj.ganhoMaximo) { a.fase = Assimetria::Falhou; return {0, 0, true}; }
    if (a.rodada >= aj.rodadas) { a.fase = Assimetria::Pronto; return {0, 0, true}; }
    return {0, 0, false};   // um ciclo parado entre as rodadas
  }
  const int e = static_cast<int>(sinal * aj.pwm * a.ganhoEsq), d = static_cast<int>(sinal * aj.pwm * a.ganhoDir);
  return {e > 255 ? 255 : e < -255 ? -255 : e, d > 255 ? 255 : d < -255 ? -255 : d, false};
}

}  // namespace seguidor
