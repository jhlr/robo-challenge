#include "rodas.h"

#include <Arduino.h>
#include <math.h>

#include "config.h"

namespace {
SMS_STS sts;

// ---- odometria: le a posicao (12 bits, 0..4095) de cada roda e soma as diferencas ----
uint16_t ultimaEsq = 0, ultimaDir = 0;
float mmEsq = 0, mmDir = 0;  // quanto cada lado andou desde zerarOdometria(), frente positivo

constexpr float kPi = 3.14159265f;
constexpr float kMmPorPasso = kPi * cfg::kDiametroRodaMm / 4096.0f;

// diferenca de posicao tratando a volta completa (4095 -> 0)
int diferenca(uint16_t antes, uint16_t agora) {
  int d = static_cast<int>(agora) - static_cast<int>(antes);
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

void zerarOdometria() {
  mmEsq = mmDir = 0;
  const int e = sts.ReadPos(cfg::kIdEsquerda), d = sts.ReadPos(cfg::kIdDireita);
  if (e >= 0) ultimaEsq = e;
  if (d >= 0) ultimaDir = d;
}

void atualizarOdometria() {
  const int e = sts.ReadPos(cfg::kIdEsquerda), d = sts.ReadPos(cfg::kIdDireita);
  if (e >= 0) { mmEsq += cfg::kSinalEsquerda * diferenca(ultimaEsq, e) * kMmPorPasso; ultimaEsq = e; }
  if (d >= 0) { mmDir += cfg::kSinalDireita * diferenca(ultimaDir, d) * kMmPorPasso; ultimaDir = d; }
}

float distanciaMm() { return 0.5f * (mmEsq + mmDir); }
float giroGraus() { return (mmEsq - mmDir) / (cfg::kBitolaMm * cfg::kDerrapagem) * 180.0f / kPi; }  // + = girou para a direita

// A janela de tensao do STS vem de fabrica em 8 V: com bateria de 12 V todo comando volta com erro. So grava se estiver diferente (a EEPROM tem vida limitada).
void ajustarJanelaDeTensao(uint8_t id) {
  constexpr uint8_t kRegMax = 14, kRegMin = 15;
  if (sts.readByte(id, kRegMax) == cfg::kTensaoMax && sts.readByte(id, kRegMin) == cfg::kTensaoMin) return;
  sts.unLockEprom(id);
  sts.writeByte(id, kRegMax, cfg::kTensaoMax);
  sts.writeByte(id, kRegMin, cfg::kTensaoMin);
  sts.LockEprom(id);
}
}  // namespace

SMS_STS& barramentoSts() { return sts; }

void rodasIniciar() {
  Serial2.begin(1000000, SERIAL_8N1, pinos::kStsRx, pinos::kStsTx);
  sts.pSerial = &Serial2;
  ajustarJanelaDeTensao(cfg::kIdEsquerda);
  ajustarJanelaDeTensao(cfg::kIdDireita);
  sts.WheelMode(cfg::kIdEsquerda);
  sts.WheelMode(cfg::kIdDireita);
  rodasParar();
}

void rodasVelocidade(float esq, float dir) {
  const int e = constrain(esq, -1.0f, 1.0f) * cfg::kVelMaximaSts;
  const int d = constrain(dir, -1.0f, 1.0f) * cfg::kVelMaximaSts;
  sts.WriteSpe(cfg::kIdEsquerda, cfg::kSinalEsquerda * e);
  sts.WriteSpe(cfg::kIdDireita, cfg::kSinalDireita * d);
}

void rodasParar() { rodasVelocidade(0, 0); }

void andarMm(float mm, float vel) {
  const float sinal = mm >= 0 ? 1.0f : -1.0f;
  const float alvo = fabsf(mm);
  const uint32_t limite = 1000 + static_cast<uint32_t>(alvo / 50.0f / vel * 1000.0f) * 3;  // folga de 3x o tempo esperado
  zerarOdometria();
  const uint32_t inicio = millis();
  while (millis() - inicio < limite) {
    atualizarOdometria();
    if (fabsf(distanciaMm()) >= alvo) break;
    rodasVelocidade(sinal * vel, sinal * vel);
  }
  rodasParar();
}

void girarGraus(float graus, float vel) {
  const float sinal = graus >= 0 ? 1.0f : -1.0f;
  const float alvo = fabsf(graus);
  zerarOdometria();
  const uint32_t inicio = millis();
  while (millis() - inicio < 4000) {
    atualizarOdometria();
    if (fabsf(giroGraus()) >= alvo) break;
    rodasVelocidade(sinal * vel, -sinal * vel);
  }
  rodasParar();
}
