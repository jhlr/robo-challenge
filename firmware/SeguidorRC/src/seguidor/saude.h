#pragma once
// Saude dos sensores: um sensor preto por tempo demais e sujeira ou defeito, nao linha (um cruzamento dura centenas de ms).
// Pura: o tempo entra como argumento. Quem chama passa barra.ok[] e o sensor marcado passa a ser ignorado.
#include "decisao.h"   // kSensores

namespace seguidor {

constexpr unsigned long kPretoTravadoMs = 30000;   // 30 s seguidos no preto: sem linha da arena que dure isso

struct SaudeSensores {
  unsigned long pretoDesde[kSensores] = {};   // 0 = nao esta preto; senao, desde quando (+1 para nunca ser 0)
};

// Chame uma vez por leitura. Marca ok[i] = false nos sensores travados e devolve quantos foram marcados agora.
inline int atualizarSaude(const bool preto[kSensores], unsigned long agoraMs, SaudeSensores& s, bool ok[kSensores],
                          unsigned long limiteMs = kPretoTravadoMs) {
  int marcados = 0;
  for (int i = 0; i < kSensores; i++) {
    if (!ok[i]) continue;
    if (!preto[i]) {
      s.pretoDesde[i] = 0;
      continue;
    }
    if (s.pretoDesde[i] == 0) s.pretoDesde[i] = agoraMs + 1;
    if (agoraMs + 1 - s.pretoDesde[i] > limiteMs) {
      ok[i] = false;
      s.pretoDesde[i] = 0;
      marcados++;
    }
  }
  return marcados;
}

}  // namespace seguidor
