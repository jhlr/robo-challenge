#pragma once
// Pure logic (no Arduino.h): when is an in-place turn finished? One question, two ways to answer it, chosen by a flag:
//   closed loop: the heading change measured by the wheel encoders reached the target (with a time limit, so a stuck wheel cannot spin the robot forever);
//   timed: the turn has lasted deg / degPerSec (open loop; drifts with battery and floor, calibrate degPerSec on the track).
#include <math.h>

namespace core {

struct TurnSpec {
  float targetDeg;           // magnitude: 90 or 180
  bool closedLoop;           // true when the heading change comes from the wheel encoders
  float timedDegPerSec;      // spin rate for the timed turn (measure it)
  float timeoutFactor;       // a closed-loop turn gives up after this many times the timed duration
};

// True once the robot has turned `targetDeg` (a magnitude) in either direction.
inline bool turnReached(float yawDeg, float targetDeg) { return fabsf(yawDeg) >= targetDeg; }

inline unsigned long timedTurnMs(const TurnSpec& t) {
  return t.timedDegPerSec > 0.0f ? static_cast<unsigned long>(t.targetDeg / t.timedDegPerSec * 1000.0f + 0.5f) : 0UL;
}

inline bool turnFinished(const TurnSpec& t, unsigned long elapsedMs, float yawDeg) {
  const unsigned long timed = timedTurnMs(t);
  if (!t.closedLoop) return elapsedMs >= timed;
  return turnReached(yawDeg, t.targetDeg) || elapsedMs >= static_cast<unsigned long>(timed * t.timeoutFactor);
}

}  // namespace core
