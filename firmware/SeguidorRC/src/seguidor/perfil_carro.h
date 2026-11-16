#pragma once
// Ajustes da decisao do carro REAL (STS3215 + barra QRE-8D), lidos de include/params.h. O firmware (src/robo/main.cpp) e o simulador (test/simulador.cpp)
// partem daqui: um numero so, em params.h, vale para os dois. Mudar velocidade/curva na pista = mudar Ajustes em decisao.h; mudar o carro = params.h.
#include <params.h>

#include "cruzamento.h"
#include "decisao.h"

namespace seguidor {

constexpr float kMmSPorPwm = params::kStsFullSpeedMmS / params::kPwmFullSts;   // velocidade de cada roda por unidade de PWM do firmware
constexpr float kLacunaMm = 60.0f;   // falha na fita que o carro atravessa em frente sem girar (a maior emenda esperada)

inline Ajustes ajustesDoCarro() {
  Ajustes aj;
  aj.barraAFrenteMm = params::kAxleToSensorMm;
  aj.bitolaMm = params::kTrackMm;             // a EFETIVA, ja com a derrapagem das 4 rodas
  aj.sensoresMortos = params::kLineDeadMask;  // sensores que a barra nao le (copia do vizinho)
  aj.zonaMortaPwm = 0;                        // STS3215: velocidade em malha fechada, sem zona morta
  // A busca (sem MPU) mede o giro pelo TEMPO: os graus por segundo vem da fisica do carro, nao de um chute. Giro no eixo com a roda de fora a velBuscaExterna e a de dentro
  // ao contrario: w = 2 v / bitola.
  aj.grausPorSegundoBusca = 2.0f * aj.velBuscaExterna * kMmSPorPwm / aj.bitolaMm * 57.29578f;
  // A lacuna e uma DISTANCIA (o carro e mais lento que o do simulador antigo): tempo = distancia / velocidade de reta.
  aj.lacunaMs = static_cast<unsigned long>(kLacunaMm / (aj.velReto * kMmSPorPwm) * 1000.0f + 0.5f);
  return aj;
}

// O que o cruzamento supoe da velocidade: a de reta do carro (o robo ainda a corrige pelas amostras do encoder).
inline AjustesCruzamento cruzamentoDoCarro(const Ajustes& aj) {
  AjustesCruzamento c;
  c.velMmS = aj.velReto * kMmSPorPwm;
  return c;
}

}  // namespace seguidor
