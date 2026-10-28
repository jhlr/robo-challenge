#pragma once
#include "decisao.h"

namespace seguidor {

// Pinos da ponte H TB6612FNG. Motor A = roda esquerda | Motor B = roda direita.
struct PinosPonteH {
  int ain1, ain2, pwma;
  int bin1, bin2, pwmb;
  int stby;
};

void iniciar(const PinosPonteH& pinos, int frequenciaPwmHz = 20000);   // 20 kHz: fora da faixa audivel

// Velocidade -255 a 255, positivo = frente.
void mover(const PinosPonteH& pinos, int velEsq, int velDir);

// Ponte em repouso (IN1 = IN2 = LOW, PWM 0).
void parar(const PinosPonteH& pinos);

// Faz o que a decisao mandou.
void aplicar(const PinosPonteH& pinos, const Comando& comando);

}  // namespace seguidor
