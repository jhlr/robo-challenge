#include "braco.h"

#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"
#include "rodas.h"

namespace {
Servo garra;

// Vai ate `alvo` (passos do STS, 0..4095) em passos de kOmbroPasso, conferindo cada um.
bool irPara(int alvo) {
  SMS_STS& sts = barramentoSts();
  int pos = sts.ReadPos(cfg::kIdOmbro);
  if (pos < 0) return false;                                    // posicao ilegivel: nao se mexe
  if (abs(alvo - pos) > 90.0f * 4096 / 360) return false;      // regra: nunca mais de 90 graus de uma vez
  while (abs(pos - alvo) > 12) {                                // ~1 grau de tolerancia
    const int passo = abs(alvo - pos) <= cfg::kOmbroPasso ? alvo : pos + (alvo > pos ? cfg::kOmbroPasso : -cfg::kOmbroPasso);
    if (sts.WritePosEx(cfg::kIdOmbro, passo, cfg::kOmbroVelocidade, cfg::kOmbroAceleracao) == 0) return false;
    delay(400);
    pos = sts.ReadPos(cfg::kIdOmbro);
    const int corrente = sts.ReadCurrent(cfg::kIdOmbro);
    const bool naoSegue = pos < 0 || abs(pos - passo) > 40;
    if (naoSegue || abs(corrente) > cfg::kOmbroCorrenteMax) {
      if (pos >= 0) sts.WritePosEx(cfg::kIdOmbro, pos, cfg::kOmbroVelocidade, cfg::kOmbroAceleracao);   // segura onde parou
      return false;
    }
  }
  return true;
}
}  // namespace

bool bracoIniciar() {
  SMS_STS& sts = barramentoSts();
  if (sts.Ping(cfg::kIdOmbro) == -1) return false;
  // Um STS que ficou em modo roda tem limite de angulo 0 e nao anda: poe em modo posicao (so grava se precisar, a EEPROM tem vida limitada).
  const int limite = sts.readWord(cfg::kIdOmbro, SMS_STS_MAX_ANGLE_LIMIT_L), modo = sts.readByte(cfg::kIdOmbro, SMS_STS_MODE);
  if (limite != 4095 || modo != 0) {
    sts.unLockEprom(cfg::kIdOmbro);
    sts.writeWord(cfg::kIdOmbro, SMS_STS_MAX_ANGLE_LIMIT_L, 4095);
    sts.writeByte(cfg::kIdOmbro, SMS_STS_MODE, 0);
    sts.LockEprom(cfg::kIdOmbro);
  }
  garra.setPeriodHertz(50);
  garra.attach(pinos::kGarra, 500, 2400);
  return sts.EnableTorque(cfg::kIdOmbro, 1) != 0;
}

bool bracoSubir() { return irPara(cfg::kOmbroTopo); }
bool bracoPlataforma() { return irPara(cfg::kOmbroPlataforma); }
bool bracoPegar() { return irPara(cfg::kOmbroChao); }

void garraAbrir() {
  garra.write(cfg::kGarraAbertaGraus);
  delay(cfg::kGarraTempoMs);
}

void garraFechar() {
  garra.write(cfg::kGarraFechadaGraus);
  delay(cfg::kGarraTempoMs);
}
