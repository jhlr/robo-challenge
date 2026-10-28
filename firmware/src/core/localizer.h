#pragma once
// Pure logic (no Arduino.h): where is the robot on the street graph? A histogram (Bayes) filter over every junction and heading.
// The arena looks the same from most junctions (all 30 are 4-way), so one crossing says little; what tells places apart is the EDGE of the map:
// a street that ends after the last junction (the 127 mm stubs), the red border and the other team's blue line. Coordinates are relative to
// our side (map.h): c = 0 is the column next to the blue line we came in by, heading 0 = +c ("into the city").
//
// Events, in the order they happen on the track:
//   onOwnBlue()       we read the blue line of OUR green area: now at the gate, facing +c, row unknown (the stubs of the 5 rows look alike)
//   onCrossing()      a crossing was confirmed (8 sensors on black, then the 90 degree check): moved one junction ahead
//   onTurn(k)         the robot spun k quarter turns to the left (negative = right)
//   onLineEnd()       the line ended right after a junction (end of a stub)
//   onForbidden(kind) red under any sensor, or the other team's blue: both only exist beyond the last junction
// estimate() gives the best pose and how sure we are. Below a confidence bar the robot should not trust the map and go find an anchor.
#include <string.h>

#include "core/map.h"

namespace core {

constexpr int kHeadings = 4;  // 0 = +c, 1 = +r, 2 = -c, 3 = -r (a left turn adds 1)
constexpr int8_t kDirs[kHeadings][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

struct PoseBelief {
  float p[kMapCols][kMapRows][kHeadings];
  bool atGate;  // true between onOwnBlue() and the first crossing: still in the stub, p holds the row distribution at (0, r) facing +c
};

enum class Forbidden : uint8_t { Red, OtherBlue };

struct LocalizerModel {
  float pMissCrossing = 0.05f;      // a crossing went unnoticed: we moved two junctions, not one
  float pEndAtBoundary = 0.95f;     // line ends after a boundary junction facing out
  float pEndInterior = 0.03f;       // line "ends" in the middle of the map (gap in the tape, sensor glitch)
  float pForbiddenOutside = 0.95f;  // a forbidden colour is read only beyond the last junction
  float pForbiddenInside = 0.02f;
};

inline float totalMass(const PoseBelief& b) {
  float s = 0.0f;
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r)
      for (int h = 0; h < kHeadings; ++h) s += b.p[c][r][h];
  return s;
}

// Nothing known: every junction and heading equally likely.
inline void initUniform(PoseBelief& b) {
  const float v = 1.0f / (kMapCols * kMapRows * kHeadings);
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r)
      for (int h = 0; h < kHeadings; ++h) b.p[c][r][h] = v;
  b.atGate = false;
}

// Rescale to 1. If everything was ruled out the model and the world disagree (we are lost): start again from nothing and say so.
inline bool normalize(PoseBelief& b) {
  const float s = totalMass(b);
  if (s < 1e-9f) {
    initUniform(b);
    return false;
  }
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r)
      for (int h = 0; h < kHeadings; ++h) b.p[c][r][h] /= s;
  return true;
}

inline void onOwnBlue(PoseBelief& b) {
  memset(b.p, 0, sizeof(b.p));
  for (int r = 0; r < kMapRows; ++r) b.p[0][r][0] = 1.0f / kMapRows;
  b.atGate = true;
}

inline void onTurn(PoseBelief& b, int leftQuarterTurns) {
  PoseBelief n;
  memset(&n, 0, sizeof(n));
  n.atGate = b.atGate;
  const int k = ((leftQuarterTurns % kHeadings) + kHeadings) % kHeadings;
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r)
      for (int h = 0; h < kHeadings; ++h) n.p[c][r][(h + k) % kHeadings] = b.p[c][r][h];
  b = n;
}

// True when the junction after (c, r) in direction h is outside the map.
inline bool facesOutside(int c, int r, int h) { return !inMap(c + kDirs[h][0], r + kDirs[h][1]); }

// A crossing arrived. The first one after the gate is the first junction itself (column 0); after that we moved one junction (two if one was missed).
// Facing outside at the edge cannot produce a crossing, so those states disappear: that is evidence too.
inline bool onCrossing(PoseBelief& b, const LocalizerModel& m = {}) {
  if (b.atGate) {
    b.atGate = false;
    return true;
  }
  PoseBelief n;
  memset(&n, 0, sizeof(n));
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r)
      for (int h = 0; h < kHeadings; ++h) {
        const float mass = b.p[c][r][h];
        if (mass <= 0.0f) continue;
        const int c1 = c + kDirs[h][0], r1 = r + kDirs[h][1];
        if (!inMap(c1, r1)) continue;
        n.p[c1][r1][h] += mass * (1.0f - m.pMissCrossing);
        const int c2 = c1 + kDirs[h][0], r2 = r1 + kDirs[h][1];
        if (inMap(c2, r2)) n.p[c2][r2][h] += mass * m.pMissCrossing;
      }
  n.atGate = false;
  b = n;
  return normalize(b);
}

// Weight every state by how likely the reading is there, then rescale.
template <typename Likelihood>
inline bool weigh(PoseBelief& b, Likelihood&& like) {
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r)
      for (int h = 0; h < kHeadings; ++h) b.p[c][r][h] *= like(c, r, h);
  return normalize(b);
}

inline bool onLineEnd(PoseBelief& b, const LocalizerModel& m = {}) {
  return weigh(b, [&](int c, int r, int h) { return facesOutside(c, r, h) ? m.pEndAtBoundary : m.pEndInterior; });
}

inline bool onForbidden(PoseBelief& b, Forbidden kind, const LocalizerModel& m = {}) {
  return weigh(b, [&](int c, int r, int h) {
    const bool outside = facesOutside(c, r, h);
    const bool rightEdge = (kind == Forbidden::Red) ? (h == 1 || h == 3) : (h == 0);
    return (outside && rightEdge) ? m.pForbiddenOutside : m.pForbiddenInside;
  });
}

// How sure we are about one coordinate alone. Walking along a street teaches the column but never the row (and the other way round), so the
// full pose can be unsure while one of these is firm.
inline float columnMass(const PoseBelief& b, int c) {
  float s = 0.0f;
  for (int r = 0; r < kMapRows; ++r)
    for (int h = 0; h < kHeadings; ++h) s += b.p[c][r][h];
  return s;
}

inline float rowMass(const PoseBelief& b, int r) {
  float s = 0.0f;
  for (int c = 0; c < kMapCols; ++c)
    for (int h = 0; h < kHeadings; ++h) s += b.p[c][r][h];
  return s;
}

struct Estimate {
  Pose pose;
  float confidence;  // probability of the best state (0..1)
};

inline Estimate estimate(const PoseBelief& b) {
  Estimate e{{{0, 0}, 1, 0}, 0.0f};
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r)
      for (int h = 0; h < kHeadings; ++h)
        if (b.p[c][r][h] > e.confidence) {
          e.confidence = b.p[c][r][h];
          e.pose = Pose{{static_cast<int8_t>(c), static_cast<int8_t>(r)}, kDirs[h][0], kDirs[h][1]};
        }
  return e;
}

}  // namespace core
