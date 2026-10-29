#include "hal/shoulder.h"

#include "core/arm.h"
#include "params.h"

namespace {
sts::Bus* bus = nullptr;
constexpr uint16_t kArrivedSteps = 12;  // ~1 deg
constexpr uint32_t kTimeoutMs = 3000;
}  // namespace

bool shoulderBegin(sts::Bus& b) {
  bus = &b;
  return bus->setMode(params::kIdShoulder, sts::Mode::Servo) && bus->setTorque(params::kIdShoulder, true);
}

bool shoulderRotate(float servoDeg) {
  uint16_t now = 0, target = 0;
  if (!bus || !bus->readPosition(params::kIdShoulder, now)) return false;
  if (!core::shoulderTarget(now, servoDeg, params::kShoulderUpSign, params::kShoulderMaxMoveDeg, &target)) return false;
  if (!bus->moveTo(params::kIdShoulder, target, params::kShoulderSpeed, params::kShoulderAcc)) return false;
  const uint32_t t0 = millis();
  while (millis() - t0 < kTimeoutMs) {
    delay(30);
    if (bus->readPosition(params::kIdShoulder, now) && abs(static_cast<int>(now) - static_cast<int>(target)) <= kArrivedSteps) break;
  }
  return true;
}
