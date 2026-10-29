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

// First junction of the shortest way to the nearest junction among the least visited ones. false when there is nowhere to go.
inline bool stepToLeastVisited(const Pose& pose, const Visits& visits, Node* next, const Blocked* blocked = nullptr) {
  int dist[kMapCols][kMapRows];
  Node from[kMapCols][kMapRows];
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r) dist[c][r] = -1;
  Node queue[kMapNodes];
  int head = 0, tail = 0;
  dist[pose.at.c][pose.at.r] = 0;
  queue[tail++] = pose.at;
  static const int8_t kStep[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  while (head < tail) {
    const Node n = queue[head++];
    for (const auto& s : kStep) {
      const int c = n.c + s[0], r = n.r + s[1];
      if (!inMap(c, r) || dist[c][r] >= 0) continue;
      if (isBlocked(blocked, n, Node{static_cast<int8_t>(c), static_cast<int8_t>(r)})) continue;
      dist[c][r] = dist[n.c][n.r] + 1;
      from[c][r] = n;
      queue[tail++] = Node{static_cast<int8_t>(c), static_cast<int8_t>(r)};
    }
  }
  // least visited, then nearest, then the first in a fixed order (deterministic); the junction we stand on does not count
  bool found = false;
  Node target{0, 0};
  int bestVisits = 1 << 20, bestDist = 1 << 20;
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r) {
      if (dist[c][r] <= 0) continue;
      const int v = visits.n[c][r];
      if (v < bestVisits || (v == bestVisits && dist[c][r] < bestDist)) {
        bestVisits = v;
        bestDist = dist[c][r];
        target = Node{static_cast<int8_t>(c), static_cast<int8_t>(r)};
        found = true;
      }
    }
  if (!found) return false;
  Node cur = target;
  while (!(from[cur.c][cur.r] == pose.at)) cur = from[cur.c][cur.r];
  *next = cur;
  return true;
}

}  // namespace core
