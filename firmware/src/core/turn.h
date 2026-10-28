#pragma once
// Pure logic (no Arduino.h): when is an in-place turn finished? One question, two ways to answer it, chosen by a flag so the code that
// turns does not care whether an MPU is fitted:
//   gyro:  the heading change from yaw.h reached the target (with a time limit, so a dead gyro cannot spin the robot forever);
//   timed: the turn has lasted deg / degPerSec (open loop; drifts with battery and floor, calibrate degPerSec on the track).
#include "core/yaw.h"

namespace core {

struct TurnSpec {
  float targetDeg;           // magnitude: 90 or 180
  bool useGyro;              // false until the MPU is fitted
  float timedDegPerSec;      // spin rate for the timed turn (measure it)
  float gyroTimeoutFactor;   // gyro turn gives up after this many times the timed duration
};

inline unsigned long timedTurnMs(const TurnSpec& t) {
  return t.timedDegPerSec > 0.0f ? static_cast<unsigned long>(t.targetDeg / t.timedDegPerSec * 1000.0f + 0.5f) : 0UL;
}

inline bool turnFinished(const TurnSpec& t, unsigned long elapsedMs, float yawDeg) {
  const unsigned long timed = timedTurnMs(t);
  if (!t.useGyro) return elapsedMs >= timed;
  return turnReached(yawDeg, t.targetDeg) || elapsedMs >= static_cast<unsigned long>(timed * t.gyroTimeoutFactor);
}

}  // namespace core
