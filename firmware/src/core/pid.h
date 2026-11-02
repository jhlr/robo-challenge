#pragma once
namespace core {

struct Pid {
  float kp, ki, kd, outMax;
  float dTau = 0.0f;  // seconds: low-pass on the derivative (0 = raw). Needed when err is quantised (digital line sensors): raw D is an impulse on every step.
  float integ = 0.0f, prev = 0.0f, derivF = 0.0f;
  bool first = true;

  Pid(float p, float i, float d, float limit) : kp(p), ki(i), kd(d), outMax(limit) {}

  void reset() { integ = 0.0f; prev = 0.0f; derivF = 0.0f; first = true; }

  float step(float err, float dt) {
    if (ki > 0.0f) {  // no integral at all when ki = 0 (it would only grow); anti-windup keeps it inside the output range
      integ += err * dt;
      const float lim = outMax / ki;
      if (integ > lim) integ = lim;
      if (integ < -lim) integ = -lim;
    }
    const float raw = first ? 0.0f : (err - prev) / dt;
    derivF = (dTau > 0.0f && !first) ? derivF + (dt / (dTau + dt)) * (raw - derivF) : raw;
    first = false;
    prev = err;
    float out = kp * err + ki * integ + kd * derivF;
    if (out > outMax) out = outMax;
    if (out < -outMax) out = -outMax;
    return out;
  }
};

}  // namespace core
