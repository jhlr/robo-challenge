#pragma once
#include "hal/sensors.h"

// Claw (jaw) SG90. The only code that commands it.
// STUB until the jaw is built and calibrated (user, 2026-11-09): with pins::kClaw < 0 nothing is attached, open()/close() only log and succeed, and holding() answers true.
// Even with a pin the SG90 gives no force or position feedback (the user closed it hard and the reading did not change), so holding() has no real source yet.
namespace hal {

class Claw {
 public:
  void begin(int pin);          // pin < 0: stub mode
  bool open();                  // true when the command was sent (waits params::kClawMoveMs)
  bool close();
  bool holding() const;         // STUB: always true; replace with a real check before trusting a grip
  bool isStub() const { return pin_ < 0; }

 private:
  int pin_ = -1;
  Servo servo_;
};

}  // namespace hal
