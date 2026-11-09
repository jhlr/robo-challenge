#pragma once
// Pure logic (no Arduino.h): is the car being held in the air by a hand? (user rule, 2026-11-09)
// Something VERY close to the sonar makes the car back up. If, after backing up, the SAME obstacle is still that close, it moves with the car: a hand. Then the app stops the wheels
// and pauses everything (spinning the motors or reading sensors is pointless in the air) until the sonar is free again. A wall or cube stays behind when the car backs away: not a hand.
// The sonar alone decides: the front color sensor does not read dark under a hand (measured: clear 130..330, same as a white wall).
#include <cstdint>

namespace core {

constexpr uint16_t kHandMaxMm = 30;       // "very close": nearer than this (0 = echo too short to time, which also means touching)
constexpr uint8_t kHandEnterSamples = 3;  // consecutive close samples before it backs up (one odd echo does nothing)
constexpr uint8_t kHandExitSamples = 5;   // consecutive free samples before a held hand counts as gone
constexpr float kHandBackUpMm = 50.0f;    // how far the car backs up to test whether the obstacle follows

inline bool tooClose(uint16_t sonarMm) { return sonarMm <= kHandMaxMm; }   // 0xFFFF (no echo) is not close

enum class HandStep : uint8_t { None, BackUp, Hold };

class HandGuard {
 public:
  // One sonar reading. BackUp: the caller backs up kHandBackUpMm and then calls backedUp(). Hold: stop everything and keep feeding readings until it returns None.
  HandStep update(uint16_t sonarMm) {
    if (lifted_) {
      if (tooClose(sonarMm)) { free_ = 0; return HandStep::Hold; }
      if (++free_ >= kHandExitSamples) { lifted_ = false; free_ = 0; return HandStep::None; }
      return HandStep::Hold;
    }
    if (!tooClose(sonarMm)) { close_ = 0; return HandStep::None; }
    if (++close_ < kHandEnterSamples) return HandStep::None;
    close_ = 0;
    return HandStep::BackUp;
  }
  // Reading taken after the back-up: still that close = the obstacle came along = a hand.
  HandStep backedUp(uint16_t sonarMm) {
    if (!tooClose(sonarMm)) return HandStep::None;
    lifted_ = true;
    free_ = 0;
    return HandStep::Hold;
  }
  bool lifted() const { return lifted_; }

 private:
  uint8_t close_ = 0, free_ = 0;
  bool lifted_ = false;
};

}  // namespace core
