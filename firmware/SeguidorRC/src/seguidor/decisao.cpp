#include "decisao.h"

#include <math.h>

namespace seguidor {

namespace {

Comando mover(Memoria& m, int esq, int dir, const char* acao) {
  m.ultimoEsq = esq;
  m.ultimoDir = dir;
  Comando c;
  c.esq = esq;
  c.dir = dir;
  c.parar = false;
  c.acao = acao;
  return c;
}

// Sem linha nenhuma: segue o ultimo comando por uma lacuna curta; depois gira no eixo uma volta inteira e abre em espiral LIMITADA
// (o raio nunca passa de raioMaxEspiralMm), e volta a girar no eixo; repete o ciclo ate a linha voltar.
Comando linhaPerdida(unsigned long agora, const Ajustes& aj, Memoria& m, bool bateuDeFrente = false) {
  if (m.perdeuEm == 0) m.perdeuEm = agora;
  const unsigned long perdido = agora - m.perdeuEm;

  if (perdido < aj.lacunaMs) return mover(m, m.ultimoEsq, m.ultimoDir, "PERDEU -> lacuna (mantem)");

  // Onde esta no percurso da busca. Cada volta k: gira no eixo (uma volta inteira) -> espiral limitada -> trecho reto (cresce com k) -> vira 90 graus.
  // O trecho reto e a virada levam a busca para fora do alcance da barra (o giro no lugar so alcanca ~48 mm).
  unsigned long t = perdido - aj.lacunaMs;
  unsigned long leg = aj.buscaAvancoMs;
  const unsigned long giro = buscaGiroMs(aj), esp = aj.buscaEspiralMs, vira = buscaViradaMs(aj);
  unsigned long giroAqui = giro;   // quanto dura o giro de uma volta NESTA volta da busca (so a primeira, com buscaGiraSoNaPrimeira)
  for (int k = 0;; k++) {
    const int fator = k + 1 < aj.buscaAvancoMaxFator ? k + 1 : aj.buscaAvancoMaxFator;
    leg = aj.buscaAvancoMs * static_cast<unsigned long>(fator);
    const unsigned long giroK = (k == 0 || !aj.buscaGiraSoNaPrimeira) ? giro : 0;   // so a primeira volta gira uma volta inteira
    const unsigned long total = giroK + esp + leg + vira;
    if (t < total || fator == aj.buscaAvancoMaxFator) {
      if (t >= total) t %= total;   // trecho no maximo: as voltas seguintes sao todas iguais
      giroAqui = giroK;
      break;
    }
    t -= total;
  }
  const int ladoGiro = m.ultimoLado < 0 ? -1 : 1;   // sem ter visto a linha, comeca pela direita (alternar a cada volta foi pior no simulador)
  const int fora = aj.velBuscaExterna;
  // trecho reto
  if (t >= giroAqui + esp && t < giroAqui + esp + leg) {
    // Barra acesa inteira no meio do trecho reto: bateu de frente numa linha (a barra ficou paralela a ela). Nao atravessa: gira no eixo ate a linha cruzar a barra (padrao estreito).
    if (bateuDeFrente) return ladoGiro < 0 ? mover(m, -fora, fora, "PERDEU -> bateu na linha de frente: gira") : mover(m, fora, -fora, "PERDEU -> bateu na linha de frente: gira");
    return mover(m, fora, fora, "PERDEU -> avanca reto (busca)");
  }
  // virada de 90 graus
  if (t >= giroAqui + esp + leg) return ladoGiro < 0 ? mover(m, -fora, fora, "PERDEU -> vira 90 (busca)") : mover(m, fora, -fora, "PERDEU -> vira 90 (busca)");
  // giroAqui no eixo, e depois a espiral: a roda de fora gira sempre igual, a de dentro comeca para tras e sobe ate quase a de fora (o raio cresce)
  int dentro = -aj.velBuscaExterna;
  const char* texto = "PERDEU -> gira no eixo";
  if (t >= giroAqui) {
    const float progresso = float(t - giroAqui) / float(esp > 0 ? esp : 1);   // 0 a 1
    dentro = -aj.velBuscaExterna + progresso * (velBuscaInternaFim(aj) + aj.velBuscaExterna);
    texto = "PERDEU -> espiral";
  }

  // Gira para o lado onde a linha sumiu; se nunca viu a linha, comeca pela direita
  if (m.ultimoLado < 0) return mover(m, dentro, fora, texto);
  return mover(m, fora, dentro, texto);
}

// Anota a correcao de agora e diz se as ultimas trocaram de lado vezes demais (oscilando).
bool anotarSinal(Memoria& m, int sinal, const Ajustes& aj) {
  m.sinais[m.proximoSinal] = sinal;
  m.proximoSinal = (m.proximoSinal + 1) % kJanelaOsc;
  const int janela = aj.janelaOscilacao < kJanelaOsc ? aj.janelaOscilacao : kJanelaOsc;
  int trocas = 0, anterior = 0;
  for (int k = janela; k >= 1; k--) {   // do mais antigo ao mais novo
    const int s = m.sinais[((m.proximoSinal - k) % kJanelaOsc + kJanelaOsc) % kJanelaOsc];
    if (s == 0) continue;
    if (anterior != 0 && s != anterior) trocas++;
    anterior = s;
  }
  return trocas >= aj.trocasOscilacao;
}

int contarAcesos(const bool preto[kSensores]) {
  int n = 0;
  for (int i = 0; i < kSensores; i++) n += preto[i] ? 1 : 0;
  return n;
}

}  // namespace

// Regras (S1 = ponta esquerda ... S8 = ponta direita). Toda curva gira no eixo (rodas em sentidos opostos).
//   S4 e/ou S5, sem os outros         -> em cima da linha: reto
//   S3 ou S6 tambem                   -> correcao LEVE para esse lado
//   S2 ou S7                          -> correcao FORTE para esse lado; entre os dois a velocidade sobe em rampa (posicao = media dos acesos)
//   S1 e S8                           -> as duas pontas acendem JUNTAS (cruzamento)
//   S1 ou S8 sozinho, COM sensores do meio -> estranho: nao vira a favor dele, so avisa
//   S1 ou S8 sozinho, SEM o meio      -> a linha esta na ponta: correcao FORTE para esse lado (giro intercalado com avanco)
//   padrao largo DURANTE a busca      -> barra paralela a linha (robo de lado): continua girando ate a linha cruzar a barra
//   padrao largo (>= larguraAviso acesos, sem ser cruzamento) -> anda mais devagar
//   correcoes trocando de lado seguidas (oscilando)           -> anda mais devagar
Comando decidir(const bool preto[kSensores], unsigned long agoraMs, const Ajustes& aj, Memoria& m) {
  const bool pontaSozinha = preto[0] != preto[7];             // S1 xor S8: leitura suspeita
  const bool pontasJuntas = preto[0] && preto[7];

  // Posicao da linha so pelos sensores do meio (S2..S7): media dos acesos
  float soma = 0.0f;
  int n = 0;
  for (int i = 1; i <= 6; i++) {
    if (preto[i]) {
      soma += i - 3.5f;
      n++;
    }
  }

  // So S1 ou so S8 e mais nada: a linha esta na ponta, longe do centro. Vira correcao FORTE para esse lado (posicao +-3,5), com o avanco
  // intercalado: girar no lugar nao leva a linha ao centro (o eixo esta quase embaixo da barra), so gira a barra ate ficar paralela a linha.
  if (n == 0 && pontaSozinha) {
    soma = preto[0] ? -3.5f : 3.5f;
    n = 1;
  }

  const bool buscando = estaBuscando(m, agoraMs, aj);
  const int acesos = contarAcesos(preto);

  Comando c;
  if (n == 0) {
    c = linhaPerdida(agoraMs, aj, m);   // pontas juntas sem nada no meio, ou nada: segue procurando
  } else if (buscando && acesos >= aj.larguraAviso) {
    // Girando para achar a linha, a barra acende inteira ao ficar PARALELA a ela (o robo de lado): nao e linha para seguir. Continua a busca
    // ate a linha cruzar a barra (padrao estreito).
    c = linhaPerdida(agoraMs, aj, m, true);
  } else {
    m.perdeuEm = 0;
    const float pos = soma / n;
    const float mag = fabsf(pos);
    const int sinal = mag <= aj.posReto ? 0 : (pos < 0 ? -1 : 1);
    const bool oscilando = anotarSinal(m, sinal, aj);

    if (sinal == 0) {
      const bool largo = !pontasJuntas && contarAcesos(preto) >= aj.larguraAviso;
      float v = aj.velReto;
      if (largo) v *= aj.fatorLargura;
      if (oscilando) v *= aj.fatorOscilacao;
      if (v < aj.velMinima) v = aj.velMinima;
      if (v > aj.velReto) v = aj.velReto;
      c = mover(m, static_cast<int>(v), static_cast<int>(v), largo ? "^ reto (padrao largo: devagar)" : (oscilando ? "^ reto (oscilando: devagar)" : "^ RETO ^"));
      c.largo = largo;
      c.oscilando = oscilando;
    } else {
      // rampa: ate posLeve gira na velocidade leve; de posLeve a posForte sobe ate a forte
      float t = (mag - aj.posLeve) / (aj.posForte - aj.posLeve);
      t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
      const int v = aj.velCorrecaoLeve + static_cast<int>((aj.velCorrecaoForte - aj.velCorrecaoLeve) * t + 0.5f);
      m.ultimoLado = sinal;
      // Giro intercalado com avanco: nos primeiros fracaoGiro do ciclo gira no eixo, no resto anda reto
      const float fracao = aj.fracaoGiroLeve + (aj.fracaoGiroForte - aj.fracaoGiroLeve) * t;
      // Curva fechada: so de um lado, 3 ou mais acesos: gira direto, sem avancar entre os giros
      int esqN = 0, dirN = 0;
      for (int i = 0; i < 3; i++) esqN += preto[i] ? 1 : 0;
      for (int i = 5; i < 8; i++) dirN += preto[i] ? 1 : 0;
      const bool curvaFechada = aj.curvaFechadaSensores > 0 && ((esqN >= aj.curvaFechadaSensores && dirN == 0) || (dirN >= aj.curvaFechadaSensores && esqN == 0));
      const bool girando = curvaFechada || (agoraMs % aj.periodoCorrecaoMs) < static_cast<unsigned long>(aj.periodoCorrecaoMs * fracao);
      if (!girando) {
        int avanco = static_cast<int>(aj.velReto * aj.fatorAvancoCorrecao);
        if (avanco < aj.velMinima) avanco = aj.velMinima;
        c = mover(m, avanco, avanco, sinal < 0 ? "< avanca (entre giros)" : "avanca (entre giros) >");
        c.oscilando = oscilando;
        c.estranho = pontaSozinha;
        return c;
      }
      c = sinal < 0 ? mover(m, -v, v, curvaFechada ? "<<<< CURVA FECHADA esquerda" : (v >= aj.velCorrecaoForte ? "<<< gira esquerda forte" : "<< gira esquerda"))
                    : mover(m, v, -v, curvaFechada ? "CURVA FECHADA direita >>>>" : (v >= aj.velCorrecaoForte ? "gira direita forte >>>" : "gira direita >>"));
      c.oscilando = oscilando;
    }
  }
  c.estranho = pontaSozinha;
  return c;
}

void esquecerUltimoComando(Memoria& m) {
  m.ultimoEsq = 0;
  m.ultimoDir = 0;
}

}  // namespace seguidor
