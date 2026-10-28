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

// How long to hold a constant-speed pulse to turn `deg` at `degPerSec`. Refuses (returns 0) a non-positive angle or one above `maxDeg`.
inline uint32_t spinDurationMs(float deg, float degPerSec, float maxDeg) {
  if (deg <= 0.0f || deg > maxDeg || degPerSec <= 0.0f) return 0;
  return static_cast<uint32_t>(deg / degPerSec * 1000.0f + 0.5f);
}

}  // namespace core
