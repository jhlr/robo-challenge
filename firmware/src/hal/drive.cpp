#include "hal/drive.h"

#include "core/odo.h"
#include "params.h"
#include "pins.h"

namespace hal {

#if !defined(CAR_DC)
// ---- STS3215 ----
void beginStsBus(SMS_STS& st) {
  Serial2.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  st.pSerial = &Serial2;
}

bool ensureStsVoltageWindow(SMS_STS& st, uint8_t id) {
  constexpr uint8_t kRegMaxVolt = 14, kRegMinVolt = 15;
  const int maxV = st.readByte(id, kRegMaxVolt), minV = st.readByte(id, kRegMinVolt);
  if (maxV < 0 || minV < 0) return false;
  if (maxV == params::kStsVoltMax && minV == params::kStsVoltMin) return true;
  st.unLockEprom(id);
  st.writeByte(id, kRegMaxVolt, params::kStsVoltMax);
  st.writeByte(id, kRegMinVolt, params::kStsVoltMin);
  st.LockEprom(id);
  return st.readByte(id, kRegMaxVolt) == params::kStsVoltMax && st.readByte(id, kRegMinVolt) == params::kStsVoltMin;
}

void StsDrive::begin() {
  for (const uint8_t id : {params::kIdLeft, params::kIdRight})
    if (!ensureStsVoltageWindow(st_, id)) Serial.printf("WARNING: STS %u did not confirm the voltage window (no reply or EEPROM write failed)\n", id);
  st_.WheelMode(params::kIdLeft);
  st_.WheelMode(params::kIdRight);
  setSpeeds(0.0f, 0.0f);
  resetTravel();
}

void StsDrive::setSpeeds(float left, float right) {
  const int l = static_cast<int>(constrain(left, -1.0f, 1.0f) * params::kStsMaxSteps);
  const int r = static_cast<int>(constrain(right, -1.0f, 1.0f) * params::kStsMaxSteps);
  st_.WriteSpe(params::kIdLeft, params::kLeftSign * l);
  st_.WriteSpe(params::kIdRight, params::kRightSign * r);
}

void StsDrive::resetTravel() {
  mmL_ = mmR_ = 0.0f;
  const int l = st_.ReadPos(params::kIdLeft), r = st_.ReadPos(params::kIdRight);
  if (l >= 0) lastL_ = l;
  if (r >= 0) lastR_ = r;
}

void StsDrive::poll() {
  const int l = st_.ReadPos(params::kIdLeft), r = st_.ReadPos(params::kIdRight);
  if (l >= 0) {
    mmL_ += params::kLeftSign * core::mmForCounts(core::wrapDelta(lastL_, l), params::kWheelDiaMm, params::kGearRatio);
    lastL_ = l;
  }
  if (r >= 0) {
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

// below kDcMinDuty the TT motors do not turn: map 0..1 onto kDcMinDuty..1
float DcDrive::duty(float v) {
  const float a = fabsf(constrain(v, -1.0f, 1.0f));
  return a == 0.0f ? 0.0f : params::kDcMinDuty + (1.0f - params::kDcMinDuty) * a;
}

float DcDrive::mmPerSecond(float cmd) const { return params::kDcFullSpeedMmS * duty(cmd); }

void DcDrive::drive(int pinA, int pinB, float v) {
  v = constrain(v, -1.0f, 1.0f);
  const uint32_t d = static_cast<uint32_t>(duty(v) * 1023.0f);
  ledcWrite(pinA, v > 0 ? d : 0);
  ledcWrite(pinB, v < 0 ? d : 0);
}

void DcDrive::setSpeeds(float left, float right) {
  drive(pins::kMotorLA, pins::kMotorLB, params::kLeftSign * left);
  drive(pins::kMotorRA, pins::kMotorRB, params::kRightSign * right);
}

#endif

}  // namespace hal
