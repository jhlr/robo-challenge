#pragma once
// Cruzamento: os 8 sensores no preto ao mesmo tempo. Nao toca em pino e nao chama millis(): o tempo entra como argumento, entao da para testar no PC.
// Duas leituras seguidas confirmam (ruido). Depois de confirmar, os proximos sao ignorados ate a barra sair do preto
// e passar um tempo minimo (o robo ainda esta andando por cima do mesmo cruzamento).
#include "decisao.h"   // kSensores

namespace seguidor {

struct AjustesCruzamento {
  int confirma = 2;                      // leituras seguidas com todos no preto
  unsigned long saidaMs = 300;           // depois de um cruzamento, ignora novos por pelo menos esse tempo
  unsigned long paradaMs = 2000;         // quanto o robo fica parado no cruzamento
};

struct MemoriaCruzamento {
  int vistas = 0;
  bool saindo = false;
  unsigned long saindoDesde = 0;
};

inline int contarPretos(const bool preto[kSensores]) {
  int n = 0;
  for (int i = 0; i < kSensores; i++) n += preto[i] ? 1 : 0;
  return n;
}

// Chame uma vez por leitura. Devolve true UMA vez por cruzamento.
inline bool cruzamentoConfirmado(const bool preto[kSensores], unsigned long agoraMs, const AjustesCruzamento& aj, MemoriaCruzamento& m) {
  const bool todos = contarPretos(preto) == kSensores;
  if (m.saindo) {
    m.vistas = 0;
    if (!todos && agoraMs - m.saindoDesde > aj.saidaMs) m.saindo = false;
    return false;
  }
  m.vistas = todos ? m.vistas + 1 : 0;
  if (m.vistas < aj.confirma) return false;
  m.vistas = 0;
  m.saindo = true;
  m.saindoDesde = agoraMs;
  return true;
}

// O robo ficou parado no cruzamento: o relogio de saida comeca quando ele volta a andar, nao quando chegou.
inline void comecarASair(MemoriaCruzamento& m, unsigned long agoraMs) {
  m.saindo = true;
  m.saindoDesde = agoraMs;
}

}  // namespace seguidor
