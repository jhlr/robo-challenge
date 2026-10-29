#pragma once
#include <cstdint>
namespace core {

// Shoulder servo angle for a given arm angle (deg). Bars horizontal (arm = 0) is the tooth-mesh phase at `center`; the 4:1 reduction and the gear reversal live in `ratio` and `sign`.
// The result is clamped to the servo travel [0, 180]; `clamped` (optional) reports when the request had to be cut.
inline float shoulderServoDeg(float armDeg, float center, int sign, float ratio, bool* clamped = nullptr) {
  float s = center + static_cast<float>(sign) * ratio * armDeg;
  bool cut = false;
  if (s < 0.0f) { s = 0.0f; cut = true; }
  if (s > 180.0f) { s = 180.0f; cut = true; }
  if (clamped) *clamped = cut;
  return s;
}

// Target position (0..4095, 4096 steps/rev) for a delta of `servoDeg` real servo degrees from the CURRENT position. `upSign` is the position direction that raises the arm.
// Refuses (returns false, target untouched) a zero delta, one above `maxDeg`, or a target outside the servo's single-turn range.
inline bool shoulderTarget(uint16_t current, float servoDeg, int upSign, float maxDeg, uint16_t* target) {
  const float mag = servoDeg < 0.0f ? -servoDeg : servoDeg;
  const int steps = static_cast<int>(mag * 4096.0f / 360.0f + 0.5f);
  if (steps == 0 || mag > maxDeg) return false;
  const int t = static_cast<int>(current) + (servoDeg > 0.0f ? upSign : -upSign) * steps;
  if (t < 0 || t > 4095) return false;
  *target = static_cast<uint16_t>(t);
  return true;
}

}  // namespace core
