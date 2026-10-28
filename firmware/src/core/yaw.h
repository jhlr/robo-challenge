#pragma once
// Pure logic (no Arduino.h): heading change from a gyro (MPU6050 gz, rad/s). Turns by gyro drift far less than turns by time:
// calibrate the bias while the robot is still, subtract it, integrate. Time comes in as dt, so it runs on the PC.
#include <math.h>

namespace core {

constexpr float kRadToDeg = 57.29578f;

// Running mean of gz while the robot stands still. Feed every still sample; read bias() when done.
struct GyroBias {
  float sum = 0.0f;
  int n = 0;
  float bias() const { return n > 0 ? sum / n : 0.0f; }
};

inline void addStillSample(GyroBias& b, float gzRadS) {
  b.sum += gzRadS;
  ++b.n;
}

// Heading change in degrees (+ = counter-clockwise, the gz sign of the MPU6050 mounted flat) accumulated over dt seconds.
inline float integrateYaw(float yawDeg, float gzRadS, float biasRadS, float dtSeconds) {
  return yawDeg + (gzRadS - biasRadS) * dtSeconds * kRadToDeg;
}

// True once the robot has turned `targetDeg` (a magnitude) in either direction.
inline bool turnReached(float yawDeg, float targetDeg) { return fabsf(yawDeg) >= targetDeg; }

}  // namespace core
