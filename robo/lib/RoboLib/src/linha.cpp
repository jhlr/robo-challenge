#include "linha.h"

#include <Arduino.h>

#include "config.h"
#include "rodas.h"
#include "sensores.h"

namespace {
int contarBits(uint8_t m) {
  int n = 0;
  for (; m; m >>= 1) n += m & 1;
  return n;
}
constexpr uint8_t kCentro = (1 << 3) | (1 << 4);  // S4 e S5
constexpr uint32_t kIgnorarCruzamentoMs = 900;
}  // namespace

const char* nomeParada(Parada p) {
  switch (p) {
    case Parada::Cruzamento: return "cruzamento";
    case Parada::PerdeuLinha: return "perdeu a linha";
    case Parada::Vermelho: return "vermelho";
    case Parada::Obstaculo: return "obstaculo";
    case Parada::Tempo: return "tempo";
  }
  return "?";
}

float linhaPosicao(uint8_t mascara) {
  if (mascara == 0) return kSemLinha;
  float soma = 0;
  for (int i = 0; i < 8; i++)
    if (mascara & (1 << i)) soma += i - 3.5f;   // S1 = -3,5 ... S8 = +3,5
  return soma / contarBits(mascara);
}

bool eCruzamento(uint8_t mascara) {
  const bool pontas = (mascara & 0x01) && (mascara & 0x80);
  return contarBits(mascara) >= 6 || pontas;
}

bool seguirLinhaPasso(uint8_t mascara, float vel) {
  const float pos = linhaPosicao(mascara);
  if (pos == kSemLinha) return false;
  const float giro = cfg::kGanhoLinha * pos;   // linha a direita (pos > 0): roda esquerda mais rapida, o robo vira para a direita
  rodasVelocidade(vel + giro, vel - giro);
  return true;
}

Parada seguirAteParar(float vel, int obstaculoMm, uint32_t tempoMaxMs) {
  const uint32_t inicio = millis();
  uint32_t ultimaVez = millis();   // ultima vez que a linha estava embaixo
  Parada motivo = Parada::Tempo;
  int vermelhos = 0;
  while (millis() - inicio < tempoMaxMs) {
    const uint8_t m = linhaLer();
    // Logo apos sair de um cruzamento a barra ainda ve a linha larga: so olha cruzamento depois de andar ~70 mm.
    if (millis() - inicio > kIgnorarCruzamentoMs && eCruzamento(m)) { motivo = Parada::Cruzamento; break; }
    vermelhos = (corChao() == VERMELHO) ? vermelhos + 1 : 0;
    if (vermelhos >= cfg::kVermelhoLeituras) { motivo = Parada::Vermelho; break; }
    if (obstaculoMm > 0) {
      const int d = distanciaMm();
      if (d > 0 && d < obstaculoMm) { motivo = Parada::Obstaculo; break; }
    }
    if (seguirLinhaPasso(m, vel)) ultimaVez = millis();
    else if (millis() - ultimaVez > cfg::kLinhaPerdidaMs) { motivo = Parada::PerdeuLinha; break; }
  }
  rodasParar();
  return motivo;
}

// A barra enxerga o cruzamento na borda da linha larga; o eixo esta kEixoAteBarraMm atras da barra, e o centro fica meia linha adiante.
void centrarNoCruzamento() { andarMm(cfg::kEixoAteBarraMm + cfg::kLarguraLinhaMm / 2, 0.3f); }

void virar(int graus) {
  // Gira a maior parte do angulo pelos encoders e termina girando devagar ate o centro da barra pegar a linha.
  // Assim um erro de odometria nao deixa o robo torto sobre a rua.
  const float sinal = graus >= 0 ? 1.0f : -1.0f;
  if (abs(graus) > 20) girarGraus(graus * cfg::kFracaoGiroPorEncoder, 0.4f);
  const uint32_t inicio = millis();
  while (millis() - inicio < 3000) {
    if (linhaLer() & kCentro) break;
    rodasVelocidade(sinal * 0.25f, -sinal * 0.25f);
  }
  rodasParar();
}
