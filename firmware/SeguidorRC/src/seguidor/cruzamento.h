#pragma once
// Cruzamento: S1 e S8 (as duas pontas) no preto ao mesmo tempo. Quem CONFIRMA que e cruzamento e o giro leve (giro_cruzamento.h). A rua que cruza cobre a barra de ponta a ponta; as pontas so acendem juntas num cruzamento. Nao toca em pino e nao chama millis(): o tempo entra como argumento, entao da para testar no PC.
// Duas leituras seguidas confirmam (ruido). Uma ponta sozinha NAO e cruzamento (decidir() a marca como estranha). Depois de confirmar, os proximos sao ignorados ate a barra sair do preto
// e passar um tempo minimo (o robo ainda esta andando por cima do mesmo cruzamento).
#include <math.h>

#include "alvo.h"      // kBecoMm
#include "decisao.h"   // kSensores

namespace seguidor {

struct AjustesCruzamento {
  int confirma = 2;                      // leituras seguidas com todos no preto
  unsigned long saidaMs = 300;           // depois de um cruzamento, ignora novos por pelo menos esse tempo
  float saidaMm = 40.0f;                 // e, se o odometro for dado, ate ter andado isso: parado (esperando um eco, por exemplo) o tempo passa mas o carro nao saiu de cima do cruzamento

  // Para saber se a linha sumida e fim de beco. CHUTES: meca e corrija.
  float velMmS = 160.0f;                 // velocidade real andando reto com velReto (medida no simulador e no carro: ~160 mm/s; o robo a recalibra pela distancia entre cruzamentos)
  float becoMm = kBecoMm;                // comprimento do beco depois do ultimo cruzamento (medido na imagem do mapa)
  float toleranciaBecoMm = 50.0f;        // quanto a distancia andada ate a linha sumir pode variar e ainda contar como beco (erro de odometria/velocidade, nao proporcional ao beco)
};

struct MemoriaCruzamento {
  int vistas = 0;
  bool saindo = false;
  unsigned long saindoDesde = 0;
  float saindoOdoMm = 0.0f;   // odometro ao sair (so vale se o chamador da odometro)
};

// As duas pontas no preto ao mesmo tempo: a assinatura de um cruzamento.
inline bool pontasJuntas(const bool preto[kSensores]) { return preto[0] && preto[kSensores - 1]; }

// Chame uma vez por leitura. Devolve true UMA vez por cruzamento.
// `odoMm` < 0 = sem odometro (so o tempo conta); com odometro (encoder dos STS3215) a saida exige tambem andar `saidaMm`.
inline bool cruzamentoConfirmado(const bool preto[kSensores], unsigned long agoraMs, const AjustesCruzamento& aj, MemoriaCruzamento& m, float odoMm = -1.0f) {
  const bool todos = pontasJuntas(preto);

  if (m.saindo) {
    m.vistas = 0;
    const bool andouOBastante = odoMm < 0.0f || odoMm - m.saindoOdoMm >= aj.saidaMm;
    if (!todos && agoraMs - m.saindoDesde > aj.saidaMs && andouOBastante) m.saindo = false;
    return false;
  }
  m.vistas = todos ? m.vistas + 1 : 0;
  if (m.vistas < aj.confirma) return false;
  m.vistas = 0;
  m.saindo = true;
  m.saindoDesde = agoraMs;
  m.saindoOdoMm = odoMm;
  return true;
}

// O robo ficou parado no cruzamento: o relogio de saida comeca quando ele volta a andar, nao quando chegou.
inline void comecarASair(MemoriaCruzamento& m, unsigned long agoraMs, float odoMm = -1.0f) {
  m.saindo = true;
  m.saindoDesde = agoraMs;
  m.saindoOdoMm = odoMm;
}

}  // namespace seguidor

namespace seguidor {

// O que a linha sumida significa. Falha curta na fita (lacuna) ou fim de beco (a rua acabou no limite do mapa: ~becoMm depois do ultimo
// cruzamento) ou perdida de verdade. O fim de beco e evidencia de borda para a localizacao.
enum class Perda { Nenhuma, Lacuna, FimDeBeco, Perdida };

// `andadoMm` = quanto o carro ANDOU (so enquanto avancava: sem paradas e giros) desde que saiu do cruzamento. A linha sumiu ha `perdidoHaMs`, entao sumiu em andadoMm - perdidoHa x vel;
// no fim de um beco isso e ~becoMm (a barra fica a frente do eixo, que para antes do centro: os dois erros quase se cancelam). Medir por DISTANCIA andada e nao pelo tempo desde o
// cruzamento: esse tempo inclui a parada e o giro de 90 graus, que variam muito mais que o beco.
inline Perda classificarPerda(unsigned long perdidoHaMs, float andadoMm, unsigned long lacunaMs, const AjustesCruzamento& aj) {
  if (perdidoHaMs == 0) return Perda::Nenhuma;
  if (perdidoHaMs < lacunaMs) return Perda::Lacuna;
  const float quandoSumiuMm = andadoMm - static_cast<float>(perdidoHaMs) / 1000.0f * aj.velMmS;
  const float erro = quandoSumiuMm > aj.becoMm ? quandoSumiuMm - aj.becoMm : aj.becoMm - quandoSumiuMm;
  return erro <= aj.toleranciaBecoMm ? Perda::FimDeBeco : Perda::Perdida;
}

}  // namespace seguidor
