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

inline bool nextTowardBestCube(const Pose& pose, const CubeBelief& belief, Node* next, float minBelief = kMinBelief, const Blocked* blocked = nullptr, Node* goalA = nullptr, Node* goalZ = nullptr) {
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
      if (isBlocked(blocked, n, Node{static_cast<int8_t>(c), static_cast<int8_t>(r)})) continue;   // cannot drive through a blocked street
      dist[c][r] = dist[n.c][n.r] + 1;
      from[c][r] = n;
      queue[tail++] = Node{static_cast<int8_t>(c), static_cast<int8_t>(r)};
    }
  }

  float best = 0.0f;
  Node target{0, 0};
  Node bestA{0, 0}, bestZ{0, 0};
  auto consider = [&](float p, int cost, Node goal) {
    if (p < minBelief || cost < 1) return;
    const float score = p / static_cast<float>(cost);
    if (score > best) {
      best = score;
      target = goal;
    }
  };
  forEachPlace(belief, [&](float p, Node a, Node z) {
    if (isBlocked(blocked, a, z) || (dist[a.c][a.r] < 0 && dist[z.c][z.r] < 0)) return;   // blocked, or unreachable without crossing a blocked street
    // a street cube is walked from its nearer end; if the robot already stands on that end, the goal is the other one
    const bool aNearer = dist[a.c][a.r] <= dist[z.c][z.r];
    const float before = best;
    consider(p, (aNearer ? dist[a.c][a.r] : dist[z.c][z.r]) + 1, aNearer ? z : a);
    if (best > before) { bestA = a; bestZ = z; }   // the street that holds the cube we are heading for
  });
  if (goalA) *goalA = bestA;
  if (goalZ) *goalZ = bestZ;
  if (best <= 0.0f) return false;

  // Walk back from the goal; the junction right after pose.at is the step.
  Node cur = target;
  if (cur == pose.at) return false;  // cannot happen for a street (its goal end is never the robot's own junction); kept as a guard
  while (!(from[cur.c][cur.r] == pose.at)) cur = from[cur.c][cur.r];
  *next = cur;
  return true;
}

}  // namespace core
