#pragma once
#include <stdint.h>
// Floor classification from TCS34725 raw r,g,b,clear. Thresholds are uncalibrated
// starting points: tune with the bringup `s` readout (hue/sat/clear are printed).
namespace core {

enum class Floor : uint8_t { Unknown, Black, White, Red, Green, Blue, Yellow };

struct Hsv { float h, s; };  // h in degrees 0..360, s in 0..1

inline Hsv toHsv(uint16_t r, uint16_t g, uint16_t b) {
  const float mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
  const float mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
  Hsv o{0.0f, 0.0f};
  if (mx <= 0.0f) return o;
  const float d = mx - mn;
  o.s = d / mx;
  if (d <= 0.0f) return o;
  float h;
  if (mx == r) h = 60.0f * ((g - b) / d);
  else if (mx == g) h = 60.0f * (2.0f + (b - r) / d);
  else h = 60.0f * (4.0f + (r - g) / d);
  if (h < 0.0f) h += 360.0f;
  o.h = h;
  return o;
}

constexpr uint16_t kDarkClear = 400;    // below this the surface is black
constexpr uint16_t kWhiteClear = 5000;  // grey/white surface brighter than this
constexpr float kMinSat = 0.30f;

inline Floor classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < kDarkClear) return Floor::Black;
  const Hsv x = toHsv(r, g, b);
  if (x.s < kMinSat) return c >= kWhiteClear ? Floor::White : Floor::Unknown;
  if (x.h < 20.0f || x.h >= 340.0f) return Floor::Red;
  if (x.h >= 40.0f && x.h < 80.0f) return Floor::Yellow;
  if (x.h >= 90.0f && x.h < 170.0f) return Floor::Green;
  if (x.h >= 190.0f && x.h < 270.0f) return Floor::Blue;
  return Floor::Unknown;
}

inline const char* name(Floor f) {
  switch (f) {
    case Floor::Black: return "black";
    case Floor::White: return "white";
    case Floor::Red: return "red";
    case Floor::Green: return "green";
    case Floor::Blue: return "blue";
    case Floor::Yellow: return "yellow";
    default: return "unknown";
  }
}

}  // namespace core
