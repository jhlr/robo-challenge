#pragma once
#include <cstdint>
namespace core {

// Shoulder = STS3215 driven directly (no gear): 4096 ticks per turn, so 1 tick = 360/4096 deg of arm.
constexpr float kTicksPerDeg = 4096.0f / 360.0f;

// Target position (0..4095) for a delta of `servoDeg` degrees from the CURRENT position. `upSign` is the position direction that raises the arm.
// Refuses (returns false, target untouched) a zero delta, one above `maxDeg`, or a target outside the servo's single-turn range.
inline bool shoulderTarget(uint16_t current, float servoDeg, int upSign, float maxDeg, uint16_t* target) {
  const float mag = servoDeg < 0.0f ? -servoDeg : servoDeg;
  const int steps = static_cast<int>(mag * kTicksPerDeg + 0.5f);
  if (steps == 0 || mag > maxDeg) return false;
  const int t = static_cast<int>(current) + (servoDeg > 0.0f ? upSign : -upSign) * steps;
  if (t < 0 || t > 4095) return false;
  *target = static_cast<uint16_t>(t);
  return true;
}

// Next intermediate target on the way from `current` to `goal`: at most `chunk` ticks away from `current` (small steps, user rule), never past the goal.
inline uint16_t shoulderNextTarget(uint16_t current, uint16_t goal, uint16_t chunk) {
  const int d = static_cast<int>(goal) - static_cast<int>(current);
  const int mag = d < 0 ? -d : d;
  if (mag <= chunk) return goal;
  return static_cast<uint16_t>(static_cast<int>(current) + (d > 0 ? chunk : -static_cast<int>(chunk)));
}

// True when a move from `current` to `goal` is within `maxDeg` of arm travel (the hard 90 deg limit applies to a calibrated level too).
inline bool shoulderWithinReach(uint16_t current, uint16_t goal, float maxDeg) {
  const int d = static_cast<int>(goal) - static_cast<int>(current);
  const float deg = static_cast<float>(d < 0 ? -d : d) / kTicksPerDeg;
  return deg <= maxDeg;
}

}  // namespace core
