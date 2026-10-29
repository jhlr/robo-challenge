#include "ponte_h.h"

#include <Arduino.h>

namespace seguidor {

namespace {

void motor(int in1, int in2, int pwm, int vel) {
  vel = constrain(vel, -255, 255);
  if (vel >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    vel = -vel;
  }
  analogWrite(pwm, vel);
}

CompensacaoMotores gCompensacao;

int comGanho(int vel, float ganho) {
  const float v = vel * ganho;
  return v > 255.0f ? 255 : v < -255.0f ? -255 : static_cast<int>(v);
}

}  // namespace

void definirCompensacao(const CompensacaoMotores& compensacao) { gCompensacao = compensacao; }

void iniciar(const PinosPonteH& p, int frequenciaPwmHz) {
  const int saidas[] = {p.ain1, p.ain2, p.pwma, p.bin1, p.bin2, p.pwmb, p.stby};
  for (int pino : saidas) pinMode(pino, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteFrequency(p.pwma, frequenciaPwmHz);
  analogWriteFrequency(p.pwmb, frequenciaPwmHz);
#endif
  digitalWrite(p.stby, HIGH);
  parar(p);
}

void mover(const PinosPonteH& p, int velEsq, int velDir) {
  motor(p.ain1, p.ain2, p.pwma, comGanho(velEsq, gCompensacao.esq));
  motor(p.bin1, p.bin2, p.pwmb, comGanho(velDir, gCompensacao.dir));
}

void parar(const PinosPonteH& p) {
  digitalWrite(p.ain1, LOW);
  digitalWrite(p.ain2, LOW);
  analogWrite(p.pwma, 0);
  digitalWrite(p.bin1, LOW);
  digitalWrite(p.bin2, LOW);
  analogWrite(p.pwmb, 0);
}

void aplicar(const PinosPonteH& p, const Comando& c) {
  if (c.parar) parar(p);
  else mover(p, c.esq, c.dir);
}

}  // namespace seguidor
