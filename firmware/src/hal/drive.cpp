#include "hal/drive.h"

#include "core/odo.h"
#include "params.h"
#include "pins.h"

namespace hal {

#if !defined(CAR_DC)
// ---- STS3215 ----
void StsDrive::begin() {
  bus_.setMode(params::kIdLeft, sts::Mode::Wheel);
  bus_.setMode(params::kIdRight, sts::Mode::Wheel);
  setSpeeds(0.0f, 0.0f);
  resetTravel();
}

void StsDrive::setSpeeds(float left, float right) {
  const int l = static_cast<int>(constrain(left, -1.0f, 1.0f) * params::kStsMaxSteps);
  const int r = static_cast<int>(constrain(right, -1.0f, 1.0f) * params::kStsMaxSteps);
  bus_.setWheelSpeeds(params::kIdLeft, params::kLeftSign * l, params::kIdRight, params::kRightSign * r);
}

void StsDrive::resetTravel() {
  mmL_ = mmR_ = 0.0f;
  bus_.readPosition(params::kIdLeft, lastL_);
  bus_.readPosition(params::kIdRight, lastR_);
}

void StsDrive::poll() {
  uint16_t l, r;
  if (bus_.readPosition(params::kIdLeft, l)) {
    mmL_ += params::kLeftSign * core::mmForCounts(core::wrapDelta(lastL_, l), params::kWheelDiaMm, params::kGearRatio);
    lastL_ = l;
  }
  if (bus_.readPosition(params::kIdRight, r)) {
    mmR_ += params::kRightSign * core::mmForCounts(core::wrapDelta(lastR_, r), params::kWheelDiaMm, params::kGearRatio);
    lastR_ = r;
  }
}

float StsDrive::forwardMm() {
  poll();
  return 0.5f * (mmL_ + mmR_);
}

float StsDrive::spinDeg() {
  poll();
  return (mmL_ - mmR_) / params::kTrackMm * 180.0f / core::kPi;
}

#else
// ---- DC (yellow TT) ----
void DcDrive::begin() {
  for (const int p : {pins::kMotorLA, pins::kMotorLB, pins::kMotorRA, pins::kMotorRB}) ledcAttach(p, 20000, 10);
  setSpeeds(0.0f, 0.0f);
  pinMode(pins::kMotorStby, OUTPUT);
  digitalWrite(pins::kMotorStby, HIGH);  // wake the TB6612FNG only once the inputs are idle
}

void DcDrive::drive(int pinA, int pinB, float v) {
  v = constrain(v, -1.0f, 1.0f);
  // below kDcMinDuty the TT motors do not turn: map 0..1 onto kDcMinDuty..1
  const float duty = (v == 0.0f) ? 0.0f : params::kDcMinDuty + (1.0f - params::kDcMinDuty) * fabsf(v);
  const uint32_t d = static_cast<uint32_t>(duty * 1023.0f);
  ledcWrite(pinA, v > 0 ? d : 0);
  ledcWrite(pinB, v < 0 ? d : 0);
}

void DcDrive::setSpeeds(float left, float right) {
  drive(pins::kMotorLA, pins::kMotorLB, params::kLeftSign * left);
  drive(pins::kMotorRA, pins::kMotorRB, params::kRightSign * right);
}

#endif

}  // namespace hal
