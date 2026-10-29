#pragma once
// Cruzamento: S1 e S8 (as duas pontas) no preto ao mesmo tempo. Quem CONFIRMA que e cruzamento e o giro leve (giro_cruzamento.h). A rua que cruza cobre a barra de ponta a ponta; as pontas so acendem juntas num cruzamento. Nao toca em pino e nao chama millis(): o tempo entra como argumento, entao da para testar no PC.
// Duas leituras seguidas confirmam (ruido). Uma ponta sozinha NAO e cruzamento (decidir() a marca como estranha). Depois de confirmar, os proximos sao ignorados ate a barra sair do preto
// e passar um tempo minimo (o robo ainda esta andando por cima do mesmo cruzamento).
#include <math.h>

#include "decisao.h"   // kSensores

namespace seguidor {

struct AjustesCruzamento {
  int confirma = 2;                      // leituras seguidas com todos no preto
  unsigned long saidaMs = 300;           // depois de um cruzamento, ignora novos por pelo menos esse tempo

  // Para saber se a linha sumida e fim de beco. CHUTES: meca e corrija.
  float velMmS = 250.0f;                 // velocidade real andando reto com velReto
  float becoMm = 168.0f;                 // comprimento do beco depois do ultimo cruzamento (medido na imagem do mapa)
  float toleranciaBecoPct = 0.4f;        // quanto o tempo de andar o beco pode variar e ainda contar como beco
};

struct MemoriaCruzamento {
  int vistas = 0;
  bool saindo = false;
  unsigned long saindoDesde = 0;
};

// As duas pontas no preto ao mesmo tempo: a assinatura de um cruzamento.
inline bool pontasJuntas(const bool preto[kSensores]) { return preto[0] && preto[kSensores - 1]; }

// Chame uma vez por leitura. Devolve true UMA vez por cruzamento.
inline bool cruzamentoConfirmado(const bool preto[kSensores], unsigned long agoraMs, const AjustesCruzamento& aj, MemoriaCruzamento& m) {
  const bool todos = pontasJuntas(preto);

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

namespace seguidor {

// O que a linha sumida significa. Falha curta na fita (lacuna) ou fim de beco (a rua acabou no limite do mapa: ~becoMm depois do ultimo
// cruzamento) ou perdida de verdade. O fim de beco e evidencia de borda para a localizacao.
enum class Perda { Nenhuma, Lacuna, FimDeBeco, Perdida };

inline Perda classificarPerda(unsigned long perdidoHaMs, unsigned long desdeCruzamentoMs, unsigned long lacunaMs, const AjustesCruzamento& aj) {
  if (perdidoHaMs == 0) return Perda::Nenhuma;
  if (perdidoHaMs < lacunaMs) return Perda::Lacuna;
  if (desdeCruzamentoMs < perdidoHaMs) return Perda::Perdida;
  const float becoMs = aj.becoMm / aj.velMmS * 1000.0f;
  const float quandoSumiu = static_cast<float>(desdeCruzamentoMs - perdidoHaMs);   // tempo andado desde o cruzamento ate a linha sumir
  const float erro = quandoSumiu > becoMs ? quandoSumiu - becoMs : becoMs - quandoSumiu;
  return erro <= becoMs * aj.toleranciaBecoPct ? Perda::FimDeBeco : Perda::Perdida;
}

}  // namespace seguidor
