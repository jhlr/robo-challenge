#pragma once
#include <stdint.h>
#include <math.h>
// STS3215 reports a 12-bit absolute shaft angle (4096 counts / rev). These helpers
// unwrap it into travel and convert robot motions into servo counts.
namespace core {

constexpr float kPi = 3.14159265f;
constexpr float kCountsPerRev = 4096.0f;

inline int wrapDelta(uint16_t prev, uint16_t now) {
  int d = static_cast<int>(now) - static_cast<int>(prev);
  if (d > 2048) d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

// Servo counts for a straight run of `mm` at the wheel.
inline int32_t countsForDistance(float mm, float wheelDiaMm, float gearRatio) {
  return lroundf(mm / (kPi * wheelDiaMm) * gearRatio * kCountsPerRev);
}

// Wheel travel in mm for a number of servo counts (inverse of countsForDistance).
inline float mmForCounts(int counts, float wheelDiaMm, float gearRatio) {
  return counts / kCountsPerRev / gearRatio * kPi * wheelDiaMm;
}

// Servo counts each side must travel to spin in place by `deg`.
inline int32_t countsForSpin(float deg, float trackMm, float wheelDiaMm, float gearRatio) {
  const float arcMm = trackMm * kPi * fabsf(deg) / 360.0f;
  return countsForDistance(arcMm, wheelDiaMm, gearRatio);
}

}  // namespace core
