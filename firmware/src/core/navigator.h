#pragma once
// Pure logic (no Arduino.h): what to do at a crossing. Glue between the street graph (map.h) and the cube belief (belief.h); no pins, no time.
#include "core/localizer.h"
#include "core/map.h"

namespace core {

struct Decision {
  Node next;     // junction to drive to
  Turn turn;     // how to face it from the current pose
  bool toCube;   // true: heading for a place that likely holds a cube; false: nothing likely, just exploring
};

// Neighbouring junction straight ahead, else the first one that exists (fixed order). Used when no place is worth a trip.
inline Node explorationStep(const Pose& pose) {
  const Node ahead{static_cast<int8_t>(pose.at.c + pose.dc), static_cast<int8_t>(pose.at.r + pose.dr)};
  if (inMap(ahead.c, ahead.r)) return ahead;
  static const int8_t kStep[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  for (const auto& s : kStep) {
    const int c = pose.at.c + s[0], r = pose.at.r + s[1];
    if (inMap(c, r)) return Node{static_cast<int8_t>(c), static_cast<int8_t>(r)};
  }
  return pose.at;
}

// Called when a crossing is confirmed. Goes to the best likely cube; if none is likely enough, to the least unlikely one anywhere; if the
// map is empty, explores. The caller turns by `turn`, drives to `next`, then updates the pose with moveTo().
inline Decision decideAtCrossing(const Pose& pose, const CubeBelief& belief) {
  Node next{};
  if (nextTowardBestCube(pose, belief, &next)) return {next, turnToward(pose, next), true};
  if (nextTowardBestCube(pose, belief, &next, 1e-6f)) return {next, turnToward(pose, next), false};
  next = explorationStep(pose);
  return {next, turnToward(pose, next), false};
}

// What to do at a crossing when we may not know where we are. Each coordinate is trusted only above `minCoordBelief`; otherwise the map is NOT
// used to pick a cube. The robot walks to a vertical edge (+r) and lets the red/line-end reading pin the row down (localizer.h), which it must
// answer by backing off at once (rules: red under any sensor means act immediately).
struct Plan {
  Decision move;
  bool anchoring;  // true: re-anchoring, not collecting
};

constexpr float kMinCoordBelief = 0.8f;

inline Plan planAtCrossing(const PoseBelief& loc, const CubeBelief& cubes, float minCoordBelief = kMinCoordBelief) {
  const Estimate e = estimate(loc);
  const bool colSure = columnMass(loc, e.pose.at.c) >= minCoordBelief;
  const bool rowSure = rowMass(loc, e.pose.at.r) >= minCoordBelief;
  if (colSure && rowSure) return {decideAtCrossing(e.pose, cubes), false};
  const Node up{e.pose.at.c, static_cast<int8_t>(e.pose.at.r + 1)};   // may be off the map: "keep going until the end of the street"
  return {{up, turnToward(e.pose, up), false}, true};
}

}  // namespace core
