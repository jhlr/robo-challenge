#include "hal/shoulder.h"

#include <Arduino.h>

#include "core/arm.h"
#include "params.h"

namespace {
SMS_STS* st = nullptr;
constexpr uint16_t kArrivedTicks = 12;  // ~1 deg

bool moveTo(uint16_t goal) {
  int pos = st ? st->ReadPos(params::kIdShoulder) : -1;
  if (pos < 0 || !core::shoulderWithinReach(pos, goal, params::kShoulderMaxMoveDeg)) return false;
  while (abs(pos - static_cast<int>(goal)) > kArrivedTicks) {
    const uint16_t step = core::shoulderNextTarget(pos, goal, params::kShoulderChunkTicks);
    if (st->WritePosEx(params::kIdShoulder, step, params::kShoulderSpeed, params::kShoulderAcc) == 0) return false;
    delay(params::kShoulderStepMs);
    pos = st->ReadPos(params::kIdShoulder);
    const int cur = st->ReadCurrent(params::kIdShoulder);
    if (pos < 0 || abs(pos - static_cast<int>(step)) > params::kShoulderFollowTicks || abs(cur) > params::kShoulderMaxCurrent) {
      if (pos >= 0) st->WritePosEx(params::kIdShoulder, pos, params::kShoulderSpeed, params::kShoulderAcc);  // hold where it stopped
      return false;
    }
  }
  return true;
}
}  // namespace

bool shoulderBegin(SMS_STS& s) {
  st = &s;
  if (st->Ping(params::kIdShoulder) == -1) return false;
  // Same sequence as the Waveshare driver's setMode(servo): a servo left in wheel/motor mode has max angle 0 and would not move.
  // EEPROM has a limited write life: write only when the servo is not already in position mode with the full angle range.
  const int limit = st->readWord(params::kIdShoulder, SMS_STS_MAX_ANGLE_LIMIT_L), mode = st->readByte(params::kIdShoulder, SMS_STS_MODE);
  if (limit != 4095 || mode != 0) {
    st->unLockEprom(params::kIdShoulder);
    st->writeWord(params::kIdShoulder, SMS_STS_MAX_ANGLE_LIMIT_L, 4095);
    st->writeByte(params::kIdShoulder, SMS_STS_MODE, 0);
    st->LockEprom(params::kIdShoulder);
  }
  return st->EnableTorque(params::kIdShoulder, 1) != 0;
}

int shoulderTicks() { return st ? st->ReadPos(params::kIdShoulder) : -1; }

bool shoulderGoTo(ShoulderLevel level) {
  switch (level) {
    case ShoulderLevel::Up: return moveTo(params::kShoulderTicksUp);
    case ShoulderLevel::Place: return moveTo(params::kShoulderTicksPlace);
    case ShoulderLevel::Pick: return moveTo(params::kShoulderTicksPick);
  }
  return false;
}

bool shoulderRotate(float deg) {
  uint16_t target = 0;
  const int now = shoulderTicks();
  if (now < 0 || !core::shoulderTarget(now, deg, params::kShoulderUpSign, params::kShoulderMaxMoveDeg, &target)) return false;
  return moveTo(target);
}
