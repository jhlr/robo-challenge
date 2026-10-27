#pragma once
// Pure logic (no Arduino.h): turns normalized IR readings into a line position.
// Input v[i] in 0..1, 1 = sensor sees the line. Index 0 is leftmost.
namespace core {

constexpr int kMaxSensors = 8;

struct LineState {
  float pos;       // -1 (line far left) .. +1 (line far right)
  int active;      // sensors above threshold
  bool lost;       // no sensor sees the line
  bool junction;   // all but at most one sensor see the line (crossing / T)
};

inline LineState readLine(const float* v, int n, float threshold = 0.5f) {
  LineState s{0.0f, 0, true, false};
  float sum = 0.0f, weighted = 0.0f;
  const float half = (n - 1) * 0.5f;
  for (int i = 0; i < n; ++i) {
    if (v[i] >= threshold) ++s.active;
    sum += v[i];
    weighted += v[i] * ((i - half) / half);
  }
  s.lost = (s.active == 0);
  s.junction = (s.active >= n - 1);
  s.pos = (sum > 1e-3f) ? weighted / sum : 0.0f;
  return s;
}

}  // namespace core
