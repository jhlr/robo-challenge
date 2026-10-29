#pragma once
// Pure logic (no Arduino.h): the arena street graph, the robot pose on it, and "which way to the nearest cube".
// Graph = the sim's (sim/arena.py): kCols x kRows junctions, one street between each pair of neighbours. Junction (c, r): c grows to the
// right, r grows "up" (north). The street graph is fixed; where the cubes are is not (the referee sets it), so occupancy is a parameter.
#include <stdint.h>

namespace core {

constexpr int kMapCols = 6;
constexpr int kMapRows = 5;
constexpr int kMapNodes = kMapCols * kMapRows;

struct Node {
  int8_t c, r;
};
inline bool operator==(Node a, Node b) { return a.c == b.c && a.r == b.r; }

// Where the robot is and which way it faces (unit step: (1,0) east, (0,1) north, ...).
struct Pose {
  Node at;
  int8_t dc, dr;
};

enum class Turn : uint8_t { Straight, Left, Right, UTurn };

// P(cube is there) for every place a cube can be picked up: ON a street (an edge of the graph). Never on a junction, and the cubes in the
// middle of the squares (brown marks, between the black lines) are out of reach by the rules, so they are not modelled at all.
// A certain map (all 0 or 1) is the special case; belief.h keeps these numbers honest as the other robot moves cubes.
struct CubeBelief {
  float east[kMapCols][kMapRows];   // street from (c, r) to (c + 1, r)
  float north[kMapCols][kMapRows];  // street from (c, r) to (c, r + 1)
};

// Streets we cannot drive through (a cube we must not move sits on it: white, or a colour without a platform). Same layout as CubeBelief.
struct Blocked {
  bool east[kMapCols][kMapRows];
  bool north[kMapCols][kMapRows];
};

// Visits every street as f(p, a, z), where a and z are the neighbouring junctions it joins. B may be const.
template <typename B, typename F>
inline void forEachPlace(B& b, F&& f) {
  for (int c = 0; c < kMapCols; ++c) {
    for (int r = 0; r < kMapRows; ++r) {
      const Node here{static_cast<int8_t>(c), static_cast<int8_t>(r)};
      if (c + 1 < kMapCols) f(b.east[c][r], here, Node{static_cast<int8_t>(c + 1), here.r});
      if (r + 1 < kMapRows) f(b.north[c][r], here, Node{here.c, static_cast<int8_t>(r + 1)});
    }
  }
}

// The slot of the street between two neighbouring junctions, in either order.
template <typename B>
inline auto& streetSlot(B& b, Node a, Node z) {
  return (a.r == z.r) ? b.east[a.c < z.c ? a.c : z.c][a.r] : b.north[a.c][a.r < z.r ? a.r : z.r];
}

inline bool inMap(int c, int r) { return c >= 0 && c < kMapCols && r >= 0 && r < kMapRows; }

inline void clearBlocked(Blocked& b) {
  for (int c = 0; c < kMapCols; ++c)
    for (int r = 0; r < kMapRows; ++r) b.east[c][r] = b.north[c][r] = false;
}

// Is the street between two neighbouring junctions blocked? (either order; null = nothing is blocked)
inline bool isBlocked(const Blocked* b, Node a, Node z) { return b != nullptr && streetSlot(*b, a, z); }

// The turn that makes `pose` face `next` (a neighbouring junction).
inline Turn turnToward(const Pose& pose, Node next) {
  const int dc = next.c - pose.at.c, dr = next.r - pose.at.r;
  if (dc == pose.dc && dr == pose.dr) return Turn::Straight;
  if (dc == -pose.dc && dr == -pose.dr) return Turn::UTurn;
  return (pose.dc * dr - pose.dr * dc > 0) ? Turn::Left : Turn::Right;  // cross product > 0: counter-clockwise
}

// Pose after driving from `pose.at` to the neighbouring junction `next`.
inline Pose moveTo(const Pose& pose, Node next) {
  return Pose{next, static_cast<int8_t>(next.c - pose.at.c), static_cast<int8_t>(next.r - pose.at.r)};
}

// Street worth a trip: only streets with P >= minBelief count, and among them the best P / streets-to-walk wins (so with every P = 1 it is just
// the nearest). Returns the first junction to drive to; false when nothing is likely enough. The street is walked from its nearer end. Ties keep the first one found in a fixed order, so the result is deterministic.
constexpr float kMinBelief = 0.3f;
// A cube we really SAW (P clearly above the prior of 0.4, before it drifts back) is a known cube: the robot is greedy for those, nearest first, and only wanders to unvisited streets
// when none is known. 0.5 = a seen cube still counts for about 2.5 half-lives (150 s) before it is just another unvisited street.
constexpr float kKnownCube = 0.5f;

// A street already seen EMPTY can never hold a cube, so there is no point INSISTING on it (it is never a target), but it is a perfectly good WAY: routes treat it like any other street
// (penalty 0). `emptyPenalty` stays a parameter in case a small preference for unseen streets is ever wanted (a positive value makes routes through unseen streets slightly cheaper).
constexpr float kEmptyStreetPenalty = 0.0f;
constexpr float kEmptyBelow = 0.1f;   // P below this = looked at and empty

struct Routes {
  float dist[kMapCols][kMapRows];   // cost of the cheapest way from the start (1e9 = unreachable)
  Node from[kMapCols][kMapRows];    // the junction before it on that way
};

inline float streetCost(const CubeBelief* belief, Node a, Node z, float emptyPenalty) {
  if (belief == nullptr) return 1.0f;
  return 1.0f + (streetSlot(*belief, a, z) < kEmptyBelow ? emptyPenalty : 0.0f);
}

// Cheapest ways (Bellman-Ford on 30 junctions: tiny). `belief` may be null: every street then costs 1 (plain shortest path).
inline void routesFrom(Node start, const CubeBelief* belief, const Blocked* blocked, Routes& r, float emptyPenalty = kEmptyStreetPenalty) {
  for (int c = 0; c < kMapCols; ++c)
    for (int rr = 0; rr < kMapRows; ++rr) { r.dist[c][rr] = 1e9f; r.from[c][rr] = Node{-1, -1}; }
  r.dist[start.c][start.r] = 0.0f;
  static const int8_t kStep[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  for (int pass = 0; pass < kMapNodes; ++pass) {
    bool changed = false;
    for (int c = 0; c < kMapCols; ++c)
      for (int rr = 0; rr < kMapRows; ++rr) {
        if (r.dist[c][rr] >= 1e8f) continue;
        const Node n{static_cast<int8_t>(c), static_cast<int8_t>(rr)};
        for (const auto& s : kStep) {
          const int c2 = c + s[0], r2 = rr + s[1];
          if (!inMap(c2, r2)) continue;
          const Node m{static_cast<int8_t>(c2), static_cast<int8_t>(r2)};
          if (isBlocked(blocked, n, m)) continue;
          const float d = r.dist[c][rr] + streetCost(belief, n, m, emptyPenalty);
          if (d < r.dist[c2][r2] - 1e-6f) { r.dist[c2][r2] = d; r.from[c2][r2] = n; changed = true; }
        }
      }
    if (!changed) break;
  }
}

inline bool reachable(const Routes& r, Node n) { return r.dist[n.c][n.r] < 1e8f; }

// The junction right after `start` on the way to `target`.
inline Node firstStepTo(const Routes& r, Node start, Node target) {
  Node cur = target;
  while (!(r.from[cur.c][cur.r] == start)) cur = r.from[cur.c][cur.r];
  return cur;
}

inline bool nextTowardBestCube(const Pose& pose, const CubeBelief& belief, Node* next, float minBelief = kMinBelief, const Blocked* blocked = nullptr, Node* goalA = nullptr, Node* goalZ = nullptr) {
  Routes routes;
  routesFrom(pose.at, &belief, blocked, routes);

  float best = 0.0f;
  Node target{0, 0};
  Node bestA{0, 0}, bestZ{0, 0};
  auto consider = [&](float p, float cost, Node goal) {
    if (p < minBelief || cost < 1.0f) return;
    const float score = p / cost;
    if (score > best) {
      best = score;
      target = goal;
    }
  };
  forEachPlace(belief, [&](float p, Node a, Node z) {
    if (isBlocked(blocked, a, z) || (!reachable(routes, a) && !reachable(routes, z))) return;   // blocked, or unreachable without crossing a blocked street
    // a street cube is walked from its nearer end; if the robot already stands on that end, the goal is the other one
    const bool aNearer = routes.dist[a.c][a.r] <= routes.dist[z.c][z.r];
    const float before = best;
    consider(p, (aNearer ? routes.dist[a.c][a.r] : routes.dist[z.c][z.r]) + 1.0f, aNearer ? z : a);
    if (best > before) { bestA = a; bestZ = z; }   // the street that holds the cube we are heading for
  });
  if (goalA) *goalA = bestA;
  if (goalZ) *goalZ = bestZ;
  if (best <= 0.0f) return false;

  // Walk back from the goal; the junction right after pose.at is the step.
  if (target == pose.at) return false;  // cannot happen for a street (its goal end is never the robot's own junction); kept as a guard
  *next = firstStepTo(routes, pose.at, target);
  return true;
}

}  // namespace core
