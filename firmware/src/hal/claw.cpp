#include "hal/claw.h"

#include <Arduino.h>

#include "params.h"

namespace hal {

void Claw::begin(int pin) {
  pin_ = pin;
  if (pin_ >= 0) servo_.begin(pin_);
}

bool Claw::open() {
  if (isStub()) return true;   // stub: o CarroReal registra "(garra stub)"
  servo_.write(params::kClawOpenDeg);
  delay(params::kClawMoveMs);
  return true;
}

bool Claw::close() {
  if (isStub()) return true;
  servo_.write(params::kClawClosedDeg);
  delay(params::kClawMoveMs);
  return true;
}

bool Claw::holding() const { return true; }   // no feedback on the SG90: see the header

}  // namespace hal
