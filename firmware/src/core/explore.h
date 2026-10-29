#pragma once
// Pure logic (no Arduino.h): where we have been. The cube belief says where cubes probably are, but with nothing likely the robot used to take the first
// street that existed and walk the same corner again and again. Visits counts how often each junction was reached; with no cube worth a trip, the next
// step goes toward the NEAREST junction among the LEAST visited ones (and a street blocked by a cube we may not move is never a way there).
#include <stdint.h>

#include "core/map.h"

namespace core {

struct Visits {
  uint8_t n[kMapCols][kMapRows];
};

inline void clearVisits(Visits& v) {
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r) v.n[c][r] = 0;
}

inline void markVisited(Visits& v, Node at) {
  if (inMap(at.c, at.r) && v.n[at.c][at.r] < 255) ++v.n[at.c][at.r];
}

// First junction of the cheapest way to the nearest junction among the least visited ones (soft penalty on empty streets when `belief` is given). false when there is nowhere to go.
inline bool stepToLeastVisited(const Pose& pose, const Visits& visits, Node* next, const Blocked* blocked = nullptr, const CubeBelief* belief = nullptr) {
  Routes routes;
  routesFrom(pose.at, belief, blocked, routes);
  // least visited, then cheapest, then the first in a fixed order (deterministic); the junction we stand on does not count
  bool found = false;
  Node target{0, 0};
  int bestVisits = 1 << 20;
  float bestDist = 1e9f;
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r) {
      const Node n{static_cast<int8_t>(c), static_cast<int8_t>(r)};
      if (n == pose.at || !reachable(routes, n)) continue;
      const int v = visits.n[c][r];
      if (v < bestVisits || (v == bestVisits && routes.dist[c][r] < bestDist)) {
        bestVisits = v;
        bestDist = routes.dist[c][r];
        target = n;
        found = true;
      }
    }
  if (!found) return false;
  *next = firstStepTo(routes, pose.at, target);
  return true;
}

// Where the cube map is still BLANK: a street nobody has looked at keeps P near the prior (0.2 to 0.6); once seen it is either "cube here" (high) or "never again" (low).
// With no cube worth a trip, go to the junction that borders the MOST unseen streets per street walked (ties: fewer visits), not just to a junction we have not stood on.
inline bool streetUnseen(float p) { return p > 0.2f && p < 0.6f; }

inline int unseenAround(const CubeBelief& b, Node n) {
  int k = 0;
  if (n.c + 1 < kMapCols && streetUnseen(b.east[n.c][n.r])) ++k;
  if (n.c > 0 && streetUnseen(b.east[n.c - 1][n.r])) ++k;
  if (n.r + 1 < kMapRows && streetUnseen(b.north[n.c][n.r])) ++k;
  if (n.r > 0 && streetUnseen(b.north[n.c][n.r - 1])) ++k;
  return k;
}

inline bool stepToUnseen(const Pose& pose, const CubeBelief& belief, const Visits& visits, Node* next, const Blocked* blocked = nullptr) {
  Routes routes;
  routesFrom(pose.at, &belief, blocked, routes);
  bool found = false;
  Node target{0, 0};
  float bestScore = 0.0f;
  int bestVisits = 1 << 20;
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r) {
      const Node n{static_cast<int8_t>(c), static_cast<int8_t>(r)};
      if (n == pose.at || !reachable(routes, n)) continue;
      const int unseen = unseenAround(belief, n);
      if (unseen == 0) continue;
      const float score = static_cast<float>(unseen) / routes.dist[c][r];
      if (score > bestScore || (score == bestScore && visits.n[c][r] < bestVisits)) {
        bestScore = score;
        bestVisits = visits.n[c][r];
        target = n;
        found = true;
      }
    }
  if (!found) return false;
  *next = firstStepTo(routes, pose.at, target);
  return true;
}

}  // namespace core
