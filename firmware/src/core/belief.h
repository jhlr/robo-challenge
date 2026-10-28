#pragma once
// Pure logic (no Arduino.h): the probabilistic cube map. Two robots share the arena and the other one moves or takes cubes where we cannot
// see, so what we remember about a place gets less certain with time and is corrected whenever a sensor looks at it.
//   forget():  P drifts back to the prior with a half-life, so an old sighting stops being trusted (sim/README: tau = 60 s worked best).
//   observe(): Bayes update from a noisy "I see a cube / I see none" reading.
#include <math.h>

#include "core/map.h"

namespace core {

// How the sensors and the arena behave. Starting values, to tune on the real arena.
struct BeliefModel {
  float prior = 0.4f;          // P(cube) on a place we know nothing about (36 cubes over the 49 streets is ~0.73, but the other robot takes some and the brown squares are not streets: start low)
  float halfLifeS = 60.0f;     // seconds for (P - prior) to halve
  float pSeeIfCube = 0.9f;     // P(sensor reports a cube | there is one)
  float pSeeIfEmpty = 0.05f;   // P(sensor reports a cube | there is none)
};

inline void initBelief(CubeBelief& b, const BeliefModel& m) {
  forEachPlace(b, [&](float& p, Node, Node) { p = m.prior; });
}

// Time passes: every place slides back toward the prior.
inline void forget(CubeBelief& b, float dtSeconds, const BeliefModel& m) {
  if (dtSeconds <= 0.0f || m.halfLifeS <= 0.0f) return;
  const float keep = powf(0.5f, dtSeconds / m.halfLifeS);
  forEachPlace(b, [&](float& p, Node, Node) { p = m.prior + (p - m.prior) * keep; });
}

// One reading of one place. Bayes: P(cube | reading) from the sensor model and the current P.
inline float updated(float p, bool sawCube, const BeliefModel& m) {
  const float likeCube = sawCube ? m.pSeeIfCube : 1.0f - m.pSeeIfCube;
  const float likeEmpty = sawCube ? m.pSeeIfEmpty : 1.0f - m.pSeeIfEmpty;
  const float num = likeCube * p;
  const float den = num + likeEmpty * (1.0f - p);
  return den > 0.0f ? num / den : p;
}

// One reading of the street between two neighbouring junctions (either order).
inline void observeStreet(CubeBelief& b, Node a, Node z, bool sawCube, const BeliefModel& m) {
  float& p = streetSlot(b, a, z);
  p = updated(p, sawCube, m);
}

// Sure of it: we picked the cube up (gone, p = 0), or read it with a close look (p = 1).
inline void setStreet(CubeBelief& b, Node a, Node z, float p) { streetSlot(b, a, z) = p; }

}  // namespace core
