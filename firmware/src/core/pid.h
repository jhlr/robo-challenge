#pragma once
namespace core {

struct Pid {
  float kp, ki, kd, outMax;
  float integ = 0.0f, prev = 0.0f;
  bool first = true;

  Pid(float p, float i, float d, float limit) : kp(p), ki(i), kd(d), outMax(limit) {}

  void reset() { integ = 0.0f; prev = 0.0f; first = true; }

  float step(float err, float dt) {
    integ += err * dt;
    if (ki > 0.0f) {  // anti-windup: keep the integral term inside the output range
      const float lim = outMax / ki;
      if (integ > lim) integ = lim;
      if (integ < -lim) integ = -lim;
    }
    const float deriv = first ? 0.0f : (err - prev) / dt;
    first = false;
    prev = err;
    float out = kp * err + ki * integ + kd * deriv;
    if (out > outMax) out = outMax;
    if (out < -outMax) out = -outMax;
    return out;
  }
};

}  // namespace core
