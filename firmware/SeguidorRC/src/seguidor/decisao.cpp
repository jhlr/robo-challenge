#include "decisao.h"

namespace seguidor {

namespace {

Comando mover(Memoria& m, int esq, int dir, const char* acao) {
  m.ultimoEsq = esq;
  m.ultimoDir = dir;
  return {esq, dir, false, acao};
}

// Sem linha nenhuma: segue o ultimo comando por uma lacuna curta; depois gira no eixo e abre em espiral,
// repetindo o ciclo ate a linha voltar.
Comando linhaPerdida(unsigned long agora, const Ajustes& aj, Memoria& m) {
  if (m.perdeuEm == 0) m.perdeuEm = agora;
  const unsigned long perdido = agora - m.perdeuEm;

  if (perdido < aj.lacunaMs) return mover(m, m.ultimoEsq, m.ultimoDir, "PERDEU -> lacuna (mantem)");

  // Onde esta no ciclo "gira no eixo + espiral"
  const unsigned long noCiclo = (perdido - aj.lacunaMs) % (aj.buscaGiroMs + aj.buscaEspiralMs);

  // A roda de fora gira sempre igual. A de dentro comeca para tras (giro no eixo) e sobe ate quase a de fora:
  // o raio do circulo cresce e vira espiral.
  const int fora = aj.velBuscaExterna;
  int dentro = -aj.velBuscaExterna;
  const char* texto = "PERDEU -> gira no eixo";
  if (noCiclo >= aj.buscaGiroMs) {
    const float progresso = float(noCiclo - aj.buscaGiroMs) / float(aj.buscaEspiralMs);   // 0 a 1
    dentro = -aj.velBuscaExterna + progresso * (aj.velBuscaInternaFim + aj.velBuscaExterna);
    texto = "PERDEU -> espiral";
  }

  // Gira para o lado onde a linha sumiu; se nunca viu a linha, comeca pela direita
  if (m.ultimoLado < 0) return mover(m, dentro, fora, texto);
  return mover(m, fora, dentro, texto);
}

}  // namespace

Comando decidir(const bool preto[kSensores], unsigned long agoraMs, const Ajustes& aj, Memoria& m) {
  // Ponta (S1/S8) so vale depois de confirmaPonta leituras seguidas
  m.contPontaEsq = preto[0] ? m.contPontaEsq + 1 : 0;
  m.contPontaDir = preto[7] ? m.contPontaDir + 1 : 0;

  const bool pontaEsq = m.contPontaEsq >= aj.confirmaPonta;   // S1 (confirmado)
  const bool esq      = preto[1] || preto[2];                 // S2, S3
  const bool centro   = preto[3] || preto[4];                 // S4, S5
  const bool dir      = preto[5] || preto[6];                 // S6, S7
  const bool pontaDir = m.contPontaDir >= aj.confirmaPonta;   // S8 (confirmado)
  const bool temLinha = preto[0] || esq || centro || dir || preto[7];

  if (!temLinha) return linhaPerdida(agoraMs, aj, m);
  m.perdeuEm = 0;

  if (pontaEsq && !pontaDir) {
    m.ultimoLado = -1;
    return mover(m, -aj.velGiro, aj.velGiro, "<<< GIRO ESQUERDA");
  }
  if (pontaDir && !pontaEsq) {
    m.ultimoLado = 1;
    return mover(m, aj.velGiro, -aj.velGiro, "GIRO DIREITA >>>");
  }
  if (esq && !dir) {
    m.ultimoLado = -1;
    return mover(m, aj.velLenta, aj.velRapida, "<< esquerda");
  }
  if (dir && !esq) {
    m.ultimoLado = 1;
    return mover(m, aj.velRapida, aj.velLenta, "direita >>");
  }
  // centro, ou esq e dir juntos, ou so uma ponta ainda nao confirmada
  return mover(m, aj.velReto, aj.velReto, "^ RETO ^");
}

void esquecerUltimoComando(Memoria& m) {
  m.ultimoEsq = 0;
  m.ultimoDir = 0;
}

}  // namespace seguidor
