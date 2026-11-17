#pragma once
#include "hardware/sensors.h"

// Claw (jaw) SG90. The only code that commands it.
// Sem pino (pins::kClaw < 0) nada e comandado: open()/close() so respondem sim. Com pino, o SG90 nao tem retorno de forca nem de posicao: a pegada NAO e lida aqui,
// quem decide e a sequencia (robo/carro_real.h, Manipulador::holding), pelo sonar com o ombro no topo.
namespace hal {

class Claw {
 public:
  void begin(int pin);          // pin < 0: stub mode
  bool open();                  // true when the command was sent (waits params::kClawMoveMs)
  bool close();
  bool isStub() const { return pin_ < 0; }

 private:
  int pin_ = -1;
  Servo servo_;
};

}  // namespace hal
