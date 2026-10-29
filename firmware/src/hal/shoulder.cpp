#include "hal/shoulder.h"

#include "core/arm.h"
#include "params.h"

namespace {
SMS_STS* st = nullptr;
constexpr uint16_t kArrivedSteps = 12;  // ~1 deg
constexpr uint32_t kTimeoutMs = 3000;
}  // namespace

bool shoulderBegin(SMS_STS& s) {
  st = &s;
  if (st->Ping(params::kIdShoulder) == -1) return false;
  // Same sequence as the Waveshare driver's setMode(servo): a servo left in wheel/motor mode has max angle 0 and would not move.
  st->unLockEprom(params::kIdShoulder);
  st->writeWord(params::kIdShoulder, SMS_STS_MAX_ANGLE_LIMIT_L, 4095);
  st->writeByte(params::kIdShoulder, SMS_STS_MODE, 0);
  st->LockEprom(params::kIdShoulder);
  return st->EnableTorque(params::kIdShoulder, 1) != 0;
}

bool shoulderRotate(float servoDeg) {
  uint16_t target = 0;
  const int now = st ? st->ReadPos(params::kIdShoulder) : -1;
  if (now < 0 || !core::shoulderTarget(now, servoDeg, params::kShoulderUpSign, params::kShoulderMaxMoveDeg, &target)) return false;
  if (st->WritePosEx(params::kIdShoulder, target, params::kShoulderSpeed, params::kShoulderAcc) == 0) return false;
  const uint32_t t0 = millis();
  while (millis() - t0 < kTimeoutMs) {
    delay(30);
    const int p = st->ReadPos(params::kIdShoulder);
    if (p >= 0 && abs(p - static_cast<int>(target)) <= kArrivedSteps) break;
  }
  return true;
}
