#pragma once
// A configuracao COMPLETA do robo real (STS3215: o rumo vem dos encoders), lida de include/params.h. O firmware (src/robo/main.cpp) e o simulador
// (test/simulador.cpp) partem desta funcao e so mudam o que a linha de comando pede: a logica (robo.h, decisao.cpp) e uma so.
#include "perfil_carro.h"
#include "robo.h"

namespace seguidor {

inline ConfigRobo configDoCarro() {
  ConfigRobo cfg;
  cfg.dtMs = params::kLoopDtMs;
  cfg.tauMs = params::kWheelTauMs;
  cfg.encoder = true;
  cfg.ajustes = normalizar(ajustesDoCarro());
  cfg.cruzamento = cruzamentoDoCarro(cfg.ajustes);
  return cfg;
}

}  // namespace seguidor
