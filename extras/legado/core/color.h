#pragma once
#include <stdint.h>
// Helpers de cor (HSV). A classificacao das cores e dona de core/color_table.h (medida no carro).
namespace core {

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

}  // namespace core
