#pragma once
// Calibracao dos motores com o giroscopio (pura: so conta). O robo gira no eixo com rodas opostas, sozinho, e mede:
//   1. a ZONA MORTA: sobe o PWM aos poucos ate o giroscopio acusar giro de verdade (e a unica calibracao de hardware da decisao, ver decisao.h);
//   2. os GRAUS POR SEGUNDO do giro na velocidade de busca (hoje chute em Ajustes::grausPorSegundoBusca; ja inclui a derrapagem das 4 rodas).
// Use quando o robo estiver parado no chao (o do verde, antes de comecar). Gasta ~2 s e gira uma volta ou menos no lugar.
#include <math.h>

#include "inercial.h"

namespace seguidor {

struct AjustesCalibracao {
  int pwmInicial = 40;
  int pwmMaximo = 200;           // nao achou giro ate aqui: desiste (falhou)
  unsigned long passoMs = 40;    // cada degrau de PWM dura isto
  float giroDetectadoGrauS = 8.0f;
  int amostrasDetectar = 3;      // leituras seguidas acima do limiar
  int folgaAbaixo = 2;           // a zona e um pouco abaixo do PWM em que comecou a girar (o giro ja estava vencendo o atrito)
  int velMedida = 140;           // velocidade em que mede os graus por segundo, e que passa a ser a da busca (mais rapido acha mais e antes: simulador)
  unsigned long descartaMs = 300;   // aceleracao inicial, nao conta
  unsigned long mediaMs = 500;
};

struct Calibracao {
  enum Fase : uint8_t { Rampa, Medindo, Pronto, Falhou } fase = Rampa;
  int pwm = 0;
  unsigned long degrauDesde = 0;   // +1
  int seguidas = 0;
  unsigned long medindoDesde = 0;  // +1
  float soma = 0.0f;
  int n = 0;
  // resultados
  int zonaMortaPwm = 0;
  float grausPorSegundo = 0.0f;
  int velMedida = 0;
};

struct ComandoCalibracao {
  int esq, dir;       // gira no eixo: esquerda para tras, direita para frente (anti-horario)
  bool terminou;
};

// Chame a cada leitura do giroscopio (rad/s, JA sem o vies). Devolve o que mandar aos motores; quando terminou, c.fase diz se deu certo.
inline ComandoCalibracao passoCalibracao(Calibracao& c, float gzRadS, unsigned long agoraMs, const AjustesCalibracao& aj) {
  const float giro = fabsf(gzRadS) * kRadParaGrau;
  if (c.fase == Calibracao::Pronto || c.fase == Calibracao::Falhou) return {0, 0, true};

  if (c.fase == Calibracao::Rampa) {
    if (c.pwm == 0) c.pwm = aj.pwmInicial;
    if (c.degrauDesde == 0) c.degrauDesde = agoraMs + 1;
    c.seguidas = giro > aj.giroDetectadoGrauS ? c.seguidas + 1 : 0;
    if (c.seguidas >= aj.amostrasDetectar) {
      c.zonaMortaPwm = c.pwm - aj.folgaAbaixo;
      c.velMedida = aj.velMedida > c.zonaMortaPwm + 8 ? aj.velMedida : c.zonaMortaPwm + 8;
      c.fase = Calibracao::Medindo;
      c.medindoDesde = agoraMs + 1;
      return {-c.velMedida, c.velMedida, false};
    }
    if (agoraMs + 1 - c.degrauDesde >= aj.passoMs) {
      c.pwm++;
      c.degrauDesde = agoraMs + 1;
      if (c.pwm > aj.pwmMaximo) { c.fase = Calibracao::Falhou; return {0, 0, true}; }
    }
    return {-c.pwm, c.pwm, false};
  }

  // Medindo: depois do descarte, media do giro
  const unsigned long t = agoraMs + 1 - c.medindoDesde;
  if (t >= aj.descartaMs) { c.soma += giro; c.n++; }
  if (t >= aj.descartaMs + aj.mediaMs) {
    c.grausPorSegundo = c.n > 0 ? c.soma / c.n : 0.0f;
    c.fase = c.grausPorSegundo > 1.0f ? Calibracao::Pronto : Calibracao::Falhou;
    return {0, 0, true};
  }
  return {-c.velMedida, c.velMedida, false};
}

}  // namespace seguidor
