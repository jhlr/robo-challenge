#pragma once
// Pure logic (no Arduino.h): the match starts when the floor under the car reads GREEN (the start zone) for a few readings in a row (user rule, 2026-11-09).
// A single green reading (a green cube edge, a reflection) must not launch the car.
#include <cstdint>

namespace core {

constexpr uint8_t kStartGreenSamples = 5;   // consecutive green readings (~0.5 s at one reading per 100 ms)
constexpr uint16_t kMatchSeconds = 720;     // rules: a match lasts 12 minutes (overtime is decided by the judges)

class StartTrigger {
 public:
  // One floor reading. Returns true once, when the count of consecutive green readings reaches kStartGreenSamples.
  bool update(bool green) {
    if (!green) { count_ = 0; fired_ = false; return false; }
    if (fired_) return false;
    if (++count_ < kStartGreenSamples) return false;
    fired_ = true;
    return true;
  }

 private:
  uint8_t count_ = 0;
  bool fired_ = false;
};

}  // namespace core
