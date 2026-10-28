// g++ -std=c++17 -Wall -Isrc test/teste_calibracao.cpp -o /tmp/teste_calibracao && /tmp/teste_calibracao
#include <cassert>
#include <cmath>
#include <cstdio>

#include "seguidor/calibracao.h"

using namespace seguidor;

// Robo ficticio: gira a `grausPorPwm` por PWM acima da zona (so a diferenca entre as rodas importa); abaixo, parado.
static Calibracao rodar(int zonaReal, float grausPorPwm, bool* terminou = nullptr) {
  Calibracao c;
  const AjustesCalibracao aj;
  int esq = 0, dir = 0;
  float gz = 0.0f;
  bool fim = false;
  for (unsigned long t = 0; t < 30000 && !fim; t += 5) {
    const ComandoCalibracao cmd = passoCalibracao(c, gz, t, aj);
    fim = cmd.terminou;
    esq = cmd.esq; dir = cmd.dir;
    const int v = (dir - esq) / 2;
    gz = (v > zonaReal ? (v - zonaReal + 20) * grausPorPwm : 0.0f) / kRadParaGrau;   // rad/s, ja sem o vies
  }
  if (terminou) *terminou = fim;
  return c;
}

int main() {
  { bool fim = false;
    const Calibracao c = rodar(65, 2.0f, &fim);
    assert(fim && c.fase == Calibracao::Pronto);
    assert(c.zonaMortaPwm >= 64 && c.zonaMortaPwm <= 70);         // achou perto de 65
    assert(c.velMedida >= c.zonaMortaPwm + 8);
    assert(c.grausPorSegundo > 50.0f); }
  { const Calibracao c = rodar(90, 2.0f);   // zona maior: a medida acompanha, e a velocidade de medida sobe acima dela
    assert(c.fase == Calibracao::Pronto && c.zonaMortaPwm >= 89 && c.zonaMortaPwm <= 95 && c.velMedida >= c.zonaMortaPwm + 8); }
  { bool fim = false;
    const Calibracao c = rodar(500, 2.0f, &fim);   // nunca gira (motor desligado, fio solto): falha, nao trava
    assert(fim && c.fase == Calibracao::Falhou); }
  std::puts("teste_calibracao: ok");
  return 0;
}
