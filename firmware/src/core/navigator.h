#pragma once
// Pure logic (no Arduino.h): what to do at a crossing. Glue between the street graph (map.h) and the cube belief (belief.h); no pins, no time.
#include "core/explore.h"
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
// A street whose P fell below this was LOOKED AT and was empty (a first empty look takes the prior 0.4 to ~0.07): it can never hold a cube again, so it is not a place worth a trip.
constexpr float kKnownEmpty = 0.1f;

inline Decision decideAtCrossing(const Pose& pose, const CubeBelief& belief, const Visits* visits = nullptr, const Blocked* blocked = nullptr) {
  Node next{};
  if (nextTowardBestCube(pose, belief, &next, kMinBelief, blocked)) return {next, turnToward(pose, next), true};               // a cube is probably there
  if (visits != nullptr && stepToUnseen(pose, belief, *visits, &next, blocked)) return {next, turnToward(pose, next), false};   // explore: where nobody has looked yet
  if (nextTowardBestCube(pose, belief, &next, kKnownEmpty, blocked)) return {next, turnToward(pose, next), false};              // nothing blank left: the least unlikely street that is not known empty
  if (visits != nullptr && stepToLeastVisited(pose, *visits, &next, blocked)) return {next, turnToward(pose, next), false};     // everything seen: wander toward the least visited junction
  next = explorationStep(pose);
  return {next, turnToward(pose, next), false};
}

// Carrying a cube: head for our gate (column 0, then out through the blue line to the green). While c > 0 each step is one junction toward c = 0; at c = 0 the
// step is the "junction" c = -1, i.e. out of the map through the gate: the turn says how to face the gate. atGate = true then.
inline Decision decideDelivery(const Pose& pose, bool* atGate) {
  if (pose.at.c > 0) {
    if (atGate) *atGate = false;
    const Node next{static_cast<int8_t>(pose.at.c - 1), pose.at.r};
    return {next, turnToward(pose, next), false};
  }
  if (atGate) *atGate = true;
  const Node gate{-1, pose.at.r};
  return {gate, turnToward(pose, gate), false};
}

// What to do at a crossing when we may not know where we are. Each coordinate is trusted only above `minCoordBelief`; otherwise the map is NOT
// used to pick a cube. The robot walks to a vertical edge (+r) and lets the red/line-end reading pin the row down (localizer.h), which it must
// answer by backing off at once (rules: red under any sensor means act immediately); if the ROW is firm but the column is not, it walks to the far side (+c) and lets the
// other team's blue line pin the column down.
enum class Mode : uint8_t { Collect, Anchor, FindBlue };

struct Plan {
  Decision move;  // meaningless when mode is FindBlue
  Mode mode;      // Collect: go for a cube | Anchor: pin the row down at an edge | FindBlue: still in the green, find the gate (no pose yet)
};

constexpr float kMinCoordBelief = 0.8f;

inline Plan planAtCrossing(const PoseBelief& loc, const CubeBelief& cubes, float minCoordBelief = kMinCoordBelief, const Visits* visits = nullptr, const Blocked* blocked = nullptr) {
  if (loc.inGreen) return {{{0, 0}, Turn::Straight, false}, Mode::FindBlue};
  const Estimate e = estimate(loc);
  const bool colSure = columnMass(loc, e.pose.at.c) >= minCoordBelief;
  const bool rowSure = rowMass(loc, e.pose.at.r) >= minCoordBelief;
  if (colSure && rowSure) return {decideAtCrossing(e.pose, cubes, visits, blocked), Mode::Collect};
  // The row is pinned by the red border (top/bottom), the column by the blue side lines: go to the edge that fixes the coordinate we lack (row first). The target may
  // be off the map: "keep going until the end of the street". Without the column branch the robot kept going up (which teaches the row, never the column) forever.
  const Node anchor = !rowSure ? Node{e.pose.at.c, static_cast<int8_t>(e.pose.at.r + 1)} : Node{static_cast<int8_t>(e.pose.at.c + 1), e.pose.at.r};
  return {{anchor, turnToward(e.pose, anchor), false}, Mode::Anchor};
}

}  // namespace core
