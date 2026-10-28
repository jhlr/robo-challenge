#pragma once
// Pure logic (no Arduino.h): "every sensor sees black" means a crossing. Time comes in as an argument, so it runs on the PC.
// Two readings in a row confirm it (noise), and after one is confirmed the next ones are ignored until the bar leaves the black
// and a minimum time has passed (the robot is still driving over the same crossing).
namespace core {

struct CrossingParams {
  int confirm = 2;                  // consecutive all-black readings that make a crossing
  unsigned long exitMs = 300;       // after a crossing, ignore new ones for at least this long
};

struct CrossingState {
  int seen = 0;
  bool leaving = false;
  unsigned long leaveSince = 0;
};

// Call once per reading with how many of `total` sensors see black. True exactly once per crossing.
inline bool crossingStep(int blackCount, int total, unsigned long nowMs, CrossingState& s, const CrossingParams& p = {}) {
  const bool allBlack = blackCount >= total;
  if (s.leaving) {
    s.seen = 0;
    if (!allBlack && nowMs - s.leaveSince > p.exitMs) s.leaving = false;
    return false;
  }
  s.seen = allBlack ? s.seen + 1 : 0;
  if (s.seen < p.confirm) return false;
  s.seen = 0;
  s.leaving = true;
  s.leaveSince = nowMs;
  return true;
}

// The robot stopped on the crossing for a while: the "leaving" clock starts when it drives off, not when it arrived.
inline void leaveCrossing(CrossingState& s, unsigned long nowMs) {
  s.leaving = true;
  s.leaveSince = nowMs;
}

}  // namespace core
