#include <unity.h>

#include "core/arm.h"
#include "core/belief.h"
#include "core/color.h"
#include "core/color_table.h"
#include "core/crossing.h"
#include "core/line.h"
#include "core/localizer.h"
#include "core/map.h"
#include "core/navigator.h"
#include "core/odo.h"
#include "core/pid.h"
#include "core/turn.h"
#include "core/yaw.h"
#include "params.h"

void setUp() {}
void tearDown() {}

void test_line_centered() {
  const float v[5] = {0, 0, 1, 0, 0};
  const core::LineState s = core::readLine(v, 5);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, s.pos);
  TEST_ASSERT_FALSE(s.lost);
  TEST_ASSERT_FALSE(s.junction);
}

void test_line_left_is_negative() {
  const float v[5] = {1, 0, 0, 0, 0};
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, core::readLine(v, 5).pos);
}

void test_line_between_two_sensors() {
  const float v[5] = {0, 0, 1, 1, 0};
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.25f, core::readLine(v, 5).pos);
}

void test_line_lost_and_junction() {
  const float none[5] = {0, 0, 0, 0, 0};
  TEST_ASSERT_TRUE(core::readLine(none, 5).lost);
  const float all[5] = {1, 1, 1, 1, 1};
  TEST_ASSERT_TRUE(core::readLine(all, 5).junction);
  const float four[5] = {1, 1, 1, 1, 0};
  TEST_ASSERT_TRUE(core::readLine(four, 5).junction);
  const float three[5] = {0, 1, 1, 1, 0};
  TEST_ASSERT_FALSE(core::readLine(three, 5).junction);
}

void test_pid_sign_and_clamp() {
  core::Pid p(1.0f, 0.0f, 0.0f, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, p.step(0.3f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f, p.step(-2.0f, 0.01f));
}

void test_wrap_delta() {
  TEST_ASSERT_EQUAL_INT(10, core::wrapDelta(4090, 4));
  TEST_ASSERT_EQUAL_INT(-10, core::wrapDelta(4, 4090));
  TEST_ASSERT_EQUAL_INT(100, core::wrapDelta(500, 600));
}

void test_spin_counts() {
  // track 100, wheel 66, ratio 1: arc = 100*pi/4 = 78.54 mm -> 78.54/207.35*4096 = 1551
  TEST_ASSERT_INT_WITHIN(2, 1551, core::countsForSpin(90.0f, 100.0f, 66.0f, 1.0f));
  TEST_ASSERT_INT_WITHIN(4, 3102, core::countsForSpin(180.0f, 100.0f, 66.0f, 1.0f));
}

void test_color_classes() {
  TEST_ASSERT_EQUAL(core::Floor::Black, core::classify(10, 10, 10, 100));
  TEST_ASSERT_EQUAL(core::Floor::White, core::classify(2000, 2000, 2000, 6000));
  TEST_ASSERT_EQUAL(core::Floor::Red, core::classify(3000, 600, 500, 4500));
  TEST_ASSERT_EQUAL(core::Floor::Green, core::classify(600, 3000, 900, 4500));
  TEST_ASSERT_EQUAL(core::Floor::Blue, core::classify(500, 1200, 3000, 4700));
  TEST_ASSERT_EQUAL(core::Floor::Yellow, core::classify(3000, 2600, 400, 6000));
}

void test_shoulder_mapping() {
  using namespace params;
  // arm angles from cad/braco/params.json: pick -9.788, mid -16, place +25.468 -> servo 148 / 173 / 7 deg
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 148.0f, core::shoulderServoDeg(-9.788f, kShoulderCenterDeg, kShoulderSign, kShoulderRatio));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 173.0f, core::shoulderServoDeg(-16.0f, kShoulderCenterDeg, kShoulderSign, kShoulderRatio));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 7.0f, core::shoulderServoDeg(25.468f, kShoulderCenterDeg, kShoulderSign, kShoulderRatio));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kShoulderCenterDeg, core::shoulderServoDeg(0.0f, kShoulderCenterDeg, kShoulderSign, kShoulderRatio));
}

void test_shoulder_clamps_to_servo_travel() {
  bool cut = false;
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, core::shoulderServoDeg(40.0f, params::kShoulderCenterDeg, params::kShoulderSign, params::kShoulderRatio, &cut));
  TEST_ASSERT_TRUE(cut);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 180.0f, core::shoulderServoDeg(-40.0f, params::kShoulderCenterDeg, params::kShoulderSign, params::kShoulderRatio, &cut));
  TEST_ASSERT_TRUE(cut);
  core::shoulderServoDeg(-10.0f, params::kShoulderCenterDeg, params::kShoulderSign, params::kShoulderRatio, &cut);
  TEST_ASSERT_FALSE(cut);
}

void test_spin_duration() {
  TEST_ASSERT_EQUAL_UINT32(2000, core::spinDurationMs(45.0f, 22.5f, 90.0f));
  TEST_ASSERT_EQUAL_UINT32(4000, core::spinDurationMs(90.0f, 22.5f, 90.0f));  // exactly the limit
}

void test_spin_refused() {
  TEST_ASSERT_EQUAL_UINT32(0, core::spinDurationMs(0.0f, 22.5f, 90.0f));
  TEST_ASSERT_EQUAL_UINT32(0, core::spinDurationMs(-5.0f, 22.5f, 90.0f));
  TEST_ASSERT_EQUAL_UINT32(0, core::spinDurationMs(90.1f, 22.5f, 90.0f));
  TEST_ASSERT_EQUAL_UINT32(0, core::spinDurationMs(45.0f, 0.0f, 90.0f));
}

core::CubeBelief emptyCubes() {
  core::CubeBelief c{};
  return c;
}

void test_turn_toward_headings() {
  const core::Pose p{{2, 2}, 1, 0};  // facing east
  TEST_ASSERT_EQUAL(core::Turn::Straight, core::turnToward(p, {3, 2}));
  TEST_ASSERT_EQUAL(core::Turn::Left, core::turnToward(p, {2, 3}));   // north is left of east
  TEST_ASSERT_EQUAL(core::Turn::Right, core::turnToward(p, {2, 1}));
  TEST_ASSERT_EQUAL(core::Turn::UTurn, core::turnToward(p, {1, 2}));
}

void test_move_to_updates_heading() {
  const core::Pose q = core::moveTo({{2, 2}, 1, 0}, {2, 3});
  TEST_ASSERT_EQUAL(2, q.at.c);
  TEST_ASSERT_EQUAL(3, q.at.r);
  TEST_ASSERT_EQUAL(0, q.dc);
  TEST_ASSERT_EQUAL(1, q.dr);
}

void test_no_cube_no_step() {
  core::Node n{};
  TEST_ASSERT_FALSE(core::nextTowardBestCube({{0, 0}, 1, 0}, emptyCubes(), &n));
}

void test_nearest_cube_on_street() {
  core::CubeBelief c = emptyCubes();
  c.east[2][0] = 1.0f;   // street (2,0)-(3,0): walk 2 junctions + the street = cost 3
  c.north[0][3] = 1.0f;  // street (0,3)-(0,4): cost 4
  core::Node n{};
  TEST_ASSERT_TRUE(core::nextTowardBestCube({{0, 0}, 1, 0}, c, &n));
  TEST_ASSERT_EQUAL(1, n.c);
  TEST_ASSERT_EQUAL(0, n.r);
}

void test_street_cube_is_walked_from_its_near_end() {
  core::CubeBelief c = emptyCubes();
  c.east[2][0] = 1.0f;   // street (2,0)-(3,0); the robot stands at (0,0)
  core::Node n{};
  TEST_ASSERT_TRUE(core::nextTowardBestCube({{0, 0}, 1, 0}, c, &n));
  TEST_ASSERT_EQUAL(1, n.c);
  // standing on the near end, the step is the other end of that street
  TEST_ASSERT_TRUE(core::nextTowardBestCube({{2, 0}, 1, 0}, c, &n));
  TEST_ASSERT_EQUAL(3, n.c);
  TEST_ASSERT_EQUAL(0, n.r);
}

void test_cube_on_the_next_street_steps_over_it() {
  core::CubeBelief c = emptyCubes();
  c.east[0][0] = 1.0f;   // the street leaving the robot's own junction
  core::Node n{};
  TEST_ASSERT_TRUE(core::nextTowardBestCube({{0, 0}, 1, 0}, c, &n));
  TEST_ASSERT_EQUAL(1, n.c);
  TEST_ASSERT_EQUAL(0, n.r);
}

void test_forget_returns_to_prior() {
  const core::BeliefModel m;
  core::CubeBelief b{};
  b.east[1][1] = 1.0f;
  core::forget(b, m.halfLifeS, m);                       // one half-life: (1 - prior) halves
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, m.prior + (1.0f - m.prior) * 0.5f, b.east[1][1]);
  core::forget(b, 1000.0f * m.halfLifeS, m);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, m.prior, b.east[1][1]);
}

void test_observation_moves_belief_the_right_way() {
  const core::BeliefModel m;
  TEST_ASSERT_TRUE(core::updated(m.prior, true, m) > 0.8f);     // saw a cube: now likely
  TEST_ASSERT_TRUE(core::updated(m.prior, false, m) < 0.1f);    // saw none: now unlikely
  const float a = core::updated(0.5f, true, m);
  TEST_ASSERT_TRUE(core::updated(a, true, m) > a);               // two sightings beat one
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, core::updated(1.0f, true, m));   // certainty stays certain
}

void test_observe_street_picks_the_right_slot() {
  const core::BeliefModel m;
  core::CubeBelief b{};
  core::initBelief(b, m);
  core::observeStreet(b, {3, 2}, {2, 2}, true, m);   // given in either order: street (2,2)-(3,2)
  TEST_ASSERT_TRUE(b.east[2][2] > 0.8f);
  core::observeStreet(b, {1, 1}, {1, 2}, true, m);
  TEST_ASSERT_TRUE(b.north[1][1] > 0.8f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.prior, b.east[0][0]);
}

void test_unlikely_places_are_not_worth_a_trip() {
  core::CubeBelief c = emptyCubes();
  c.east[0][0] = 0.2f;   // very close but unlikely
  core::Node n{};
  TEST_ASSERT_FALSE(core::nextTowardBestCube({{0, 0}, 1, 0}, c, &n));
}

void test_likely_far_beats_unlikely_near() {
  core::CubeBelief c = emptyCubes();
  c.east[0][0] = 0.35f;  // cost 1, score 0.35
  c.north[0][1] = 0.95f; // street (0,1)-(0,2): cost 2, score 0.475
  core::Node n{};
  TEST_ASSERT_TRUE(core::nextTowardBestCube({{0, 0}, 1, 0}, c, &n));
  TEST_ASSERT_EQUAL(0, n.c);
  TEST_ASSERT_EQUAL(1, n.r);
}

void test_crossing_needs_two_readings() {
  core::CrossingState s;
  TEST_ASSERT_FALSE(core::crossingStep(8, 8, 0, s));
  TEST_ASSERT_TRUE(core::crossingStep(8, 8, 10, s));
}

void test_crossing_noise_resets_the_count() {
  core::CrossingState s;
  TEST_ASSERT_FALSE(core::crossingStep(8, 8, 0, s));
  TEST_ASSERT_FALSE(core::crossingStep(7, 8, 10, s));   // one sensor off: not a crossing
  TEST_ASSERT_FALSE(core::crossingStep(8, 8, 20, s));
  TEST_ASSERT_TRUE(core::crossingStep(8, 8, 30, s));
}

void test_crossing_counted_once_while_leaving() {
  core::CrossingState s;
  core::crossingStep(8, 8, 0, s);
  TEST_ASSERT_TRUE(core::crossingStep(8, 8, 10, s));
  TEST_ASSERT_FALSE(core::crossingStep(8, 8, 20, s));    // still all black
  TEST_ASSERT_FALSE(core::crossingStep(3, 8, 100, s));   // left the black, but too soon
  core::crossingStep(8, 8, 150, s);                      // would be a new crossing...
  TEST_ASSERT_FALSE(core::crossingStep(8, 8, 160, s));   // ...still ignored: the window has not closed
  TEST_ASSERT_FALSE(core::crossingStep(2, 8, 400, s));   // window closes here
  core::crossingStep(8, 8, 410, s);
  TEST_ASSERT_TRUE(core::crossingStep(8, 8, 420, s));    // a real second crossing
}

void test_leave_crossing_restarts_the_clock() {
  core::CrossingState s;
  core::crossingStep(8, 8, 0, s);
  core::crossingStep(8, 8, 10, s);
  core::leaveCrossing(s, 2000);                          // stopped for 2 s, now driving off
  TEST_ASSERT_FALSE(core::crossingStep(2, 8, 2100, s));  // 100 ms after leaving: too soon
  TEST_ASSERT_FALSE(core::crossingStep(2, 8, 2400, s));  // window closed
  core::crossingStep(8, 8, 2410, s);
  TEST_ASSERT_TRUE(core::crossingStep(8, 8, 2420, s));
}

void test_color_table_each_reference_is_itself() {
  for (int i = 0; i < core::NUM_CORES; ++i) {
    const core::RefCor& x = core::kCoresRef[i];
    TEST_ASSERT_EQUAL(i, core::classificarCor(x.r, x.g, x.b, x.c));
  }
}

void test_color_table_unknown_and_empty() {
  TEST_ASSERT_EQUAL(core::DESCONHECIDA, core::classificarCor(0, 0, 0, 0));
  TEST_ASSERT_EQUAL(core::DESCONHECIDA, core::classificarCor(20, 120, 20, 200));   // a pure green nobody measured
}

void test_color_table_names_keep_the_order() {
  TEST_ASSERT_EQUAL_STRING("BRANCO", core::nomeCor(core::BRANCO));
  TEST_ASSERT_EQUAL_STRING("VERMELHO", core::nomeCor(core::VERMELHO));
  TEST_ASSERT_EQUAL_STRING("DESCONHECIDA", core::nomeCor(core::DESCONHECIDA));
  TEST_ASSERT_EQUAL(0, core::BRANCO);
  TEST_ASSERT_EQUAL(1, core::PRETO);
  TEST_ASSERT_EQUAL(2, core::VERDE);
  TEST_ASSERT_EQUAL(3, core::AMARELO);
  TEST_ASSERT_EQUAL(4, core::AZUL);
  TEST_ASSERT_EQUAL(5, core::VERMELHO);
}

void test_color_filter_needs_two_equal_readings() {
  core::Cor anterior = core::DESCONHECIDA, aceita = core::BRANCO;
  aceita = core::filtrarCor(core::VERMELHO, anterior, aceita);
  TEST_ASSERT_EQUAL(core::BRANCO, aceita);                // first red: not believed yet
  aceita = core::filtrarCor(core::VERMELHO, anterior, aceita);
  TEST_ASSERT_EQUAL(core::VERMELHO, aceita);
  aceita = core::filtrarCor(core::AZUL, anterior, aceita);
  TEST_ASSERT_EQUAL(core::VERMELHO, aceita);              // a single blue does not flip it
}

void test_navigator_goes_to_a_likely_cube() {
  core::CubeBelief c = emptyCubes();
  c.east[1][0] = 0.9f;
  const core::Decision d = core::decideAtCrossing({{0, 0}, 1, 0}, c);
  TEST_ASSERT_TRUE(d.toCube);
  TEST_ASSERT_EQUAL(1, d.next.c);
  TEST_ASSERT_EQUAL(core::Turn::Straight, d.turn);
}

void test_navigator_turns_toward_the_cube() {
  core::CubeBelief c = emptyCubes();
  c.north[0][1] = 0.9f;                                     // street (0,1)-(0,2), north of the robot, which faces east
  const core::Decision d = core::decideAtCrossing({{0, 0}, 1, 0}, c);
  TEST_ASSERT_EQUAL(core::Turn::Left, d.turn);
}

void test_navigator_falls_back_to_the_least_unlikely() {
  core::CubeBelief c = emptyCubes();
  c.east[3][0] = 0.1f;                                      // below the 0.3 bar, still the only hint
  const core::Decision d = core::decideAtCrossing({{0, 0}, 1, 0}, c);
  TEST_ASSERT_FALSE(d.toCube);
  TEST_ASSERT_EQUAL(1, d.next.c);
}

void test_navigator_explores_an_empty_map() {
  const core::Decision d = core::decideAtCrossing({{5, 2}, 1, 0}, emptyCubes());   // facing east at the east edge
  TEST_ASSERT_FALSE(d.toCube);
  TEST_ASSERT_TRUE(core::inMap(d.next.c, d.next.r));
  const core::Node here{5, 2};
  TEST_ASSERT_FALSE(d.next == here);
}

void test_yaw_bias_is_the_still_mean() {
  core::GyroBias b;
  core::addStillSample(b, 0.01f);
  core::addStillSample(b, 0.03f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, b.bias());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, core::GyroBias{}.bias());
}

void test_yaw_integrates_a_quarter_turn() {
  // 90 deg/s for 1 s in 100 steps, with a 0.02 rad/s bias that must cancel out
  const float rate = 90.0f / core::kRadToDeg;
  float yaw = 0.0f;
  for (int i = 0; i < 100; ++i) yaw = core::integrateYaw(yaw, rate + 0.02f, 0.02f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, yaw);
  TEST_ASSERT_TRUE(core::turnReached(yaw, 90.0f - 0.1f));
  TEST_ASSERT_FALSE(core::turnReached(yaw, 91.0f));
}

void test_yaw_turn_reached_either_direction() {
  TEST_ASSERT_TRUE(core::turnReached(-90.0f, 90.0f));
  TEST_ASSERT_FALSE(core::turnReached(-89.0f, 90.0f));
}

void test_timed_turn_ignores_the_gyro() {
  const core::TurnSpec t{90.0f, false, 90.0f, 2.0f};   // 1 s at 90 deg/s
  TEST_ASSERT_EQUAL_UINT32(1000, core::timedTurnMs(t));
  TEST_ASSERT_FALSE(core::turnFinished(t, 999, 90.0f));   // yaw already there, but no MPU is trusted
  TEST_ASSERT_TRUE(core::turnFinished(t, 1000, 0.0f));
}

void test_gyro_turn_finishes_on_the_angle() {
  const core::TurnSpec t{90.0f, true, 90.0f, 2.0f};
  TEST_ASSERT_FALSE(core::turnFinished(t, 500, 60.0f));
  TEST_ASSERT_TRUE(core::turnFinished(t, 500, 90.0f));
  TEST_ASSERT_TRUE(core::turnFinished(t, 500, -91.0f));
}

void test_gyro_turn_times_out_if_the_gyro_is_dead() {
  const core::TurnSpec t{90.0f, true, 90.0f, 2.0f};    // limit 2 s
  TEST_ASSERT_FALSE(core::turnFinished(t, 1999, 0.0f));
  TEST_ASSERT_TRUE(core::turnFinished(t, 2000, 0.0f));
}

void test_loc_gate_gives_column_and_heading_not_row() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  TEST_ASSERT_TRUE(b.atGate);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, b.p[0][3][0]);   // 5 rows, equally likely
  core::onCrossing(b);                                    // first junction is column 0
  TEST_ASSERT_FALSE(b.atGate);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, b.p[0][0][0]);
}

void test_loc_counts_columns_along_the_street() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  for (int i = 0; i < 4; ++i) core::onCrossing(b);       // gate + 3 more: column 3
  const core::Estimate e = core::estimate(b);
  TEST_ASSERT_EQUAL(3, e.pose.at.c);
  TEST_ASSERT_EQUAL(1, e.pose.dc);
}

void test_loc_running_past_the_last_column_means_we_miscounted() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  for (int i = 0; i < 6; ++i) core::onCrossing(b);       // columns 0..5
  const bool ok = core::onCrossing(b);                    // a 7th crossing cannot exist
  TEST_ASSERT_FALSE(ok);                                  // collapsed: we are lost, start from nothing
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f / 120.0f, b.p[2][2][2]);
}

void test_loc_turn_rotates_the_heading() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  core::onCrossing(b);
  core::onTurn(b, 1);                                     // left: now facing +r
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, b.p[0][1][1]);
  core::onTurn(b, -1);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, b.p[0][1][0]);
}

void test_loc_line_end_pins_down_the_row() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  core::onCrossing(b);                                    // column 0, row unknown
  core::onTurn(b, 1);                                     // facing +r
  core::onLineEnd(b);                                     // the street ended: only the top row faces outside going +r
  const core::Estimate e = core::estimate(b);
  TEST_ASSERT_EQUAL(4, e.pose.at.r);
  TEST_ASSERT_TRUE(e.confidence > 0.85f);   // 0.95 at the edge against 0.03 in the middle: ~0.89, not certainty
}

void test_loc_walking_up_then_the_end_confirms_the_count() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  core::onCrossing(b);
  core::onTurn(b, 1);                                     // facing +r from some row
  core::onCrossing(b);                                    // one junction up: row >= 1
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, b.p[0][0][1]);    // cannot still be on row 0
  core::onLineEnd(b);                                     // ...and the end now means row 4 (it was on row 3 before)
  TEST_ASSERT_EQUAL(4, core::estimate(b).pose.at.r);
}

void test_loc_red_is_only_beyond_a_vertical_edge() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  core::onCrossing(b);
  core::onTurn(b, 1);
  core::onCrossing(b);
  core::onForbidden(b, core::Forbidden::Red);
  const core::Estimate e = core::estimate(b);
  TEST_ASSERT_EQUAL(4, e.pose.at.r);
  TEST_ASSERT_EQUAL(1, e.pose.dr);
}

void test_loc_other_blue_pins_the_far_column() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  for (int i = 0; i < 6; ++i) core::onCrossing(b);       // column 5 (if the count is right)
  core::onForbidden(b, core::Forbidden::OtherBlue);
  TEST_ASSERT_TRUE(core::columnMass(b, 5) > 0.9f);       // the column is firm...
  TEST_ASSERT_TRUE(core::estimate(b).confidence < 0.3f); // ...the row is still any of 5 (it never showed)
  TEST_ASSERT_EQUAL(5, core::estimate(b).pose.at.c);
}

void test_loc_a_missed_crossing_is_not_fatal() {
  core::PoseBelief b;
  core::onOwnBlue(b);
  for (int i = 0; i < 5; ++i) core::onCrossing(b);       // counted column 4
  core::onForbidden(b, core::Forbidden::OtherBlue);      // but the other blue says we are at the edge: column 5 (one crossing was missed)
  TEST_ASSERT_EQUAL(5, core::estimate(b).pose.at.c);
}

void test_plan_trusts_a_known_pose() {
  core::PoseBelief loc;
  core::onOwnBlue(loc);
  core::onCrossing(loc);
  core::onTurn(loc, 1);
  core::onCrossing(loc);
  core::onLineEnd(loc);                                   // row 4 and firm enough
  core::onTurn(loc, 1);                                   // facing -c now, at the top edge: both coordinates known
  core::CubeBelief cubes = emptyCubes();
  cubes.east[0][4] = 0.9f;                                // the street just east of (0,4)
  const core::Plan p = core::planAtCrossing(loc, cubes, 0.85f);
  TEST_ASSERT_FALSE(p.anchoring);
  TEST_ASSERT_TRUE(p.move.toCube);
}

void test_plan_anchors_when_the_row_is_unknown() {
  core::PoseBelief loc;
  core::onOwnBlue(loc);
  core::onCrossing(loc);                                  // column 0, row any of 5
  core::CubeBelief cubes = emptyCubes();
  cubes.east[0][2] = 0.9f;                                // a tempting cube, but we do not know which row we are on
  const core::Plan p = core::planAtCrossing(loc, cubes);
  TEST_ASSERT_TRUE(p.anchoring);
  TEST_ASSERT_FALSE(p.move.toCube);
  TEST_ASSERT_EQUAL(core::Turn::Left, p.move.turn);       // facing +c, the +r edge is on the left
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_turn_toward_headings);
  RUN_TEST(test_move_to_updates_heading);
  RUN_TEST(test_no_cube_no_step);
  RUN_TEST(test_nearest_cube_on_street);
  RUN_TEST(test_street_cube_is_walked_from_its_near_end);
  RUN_TEST(test_cube_on_the_next_street_steps_over_it);
  RUN_TEST(test_crossing_needs_two_readings);
  RUN_TEST(test_crossing_noise_resets_the_count);
  RUN_TEST(test_crossing_counted_once_while_leaving);
  RUN_TEST(test_leave_crossing_restarts_the_clock);
  RUN_TEST(test_color_table_each_reference_is_itself);
  RUN_TEST(test_color_table_unknown_and_empty);
  RUN_TEST(test_color_table_names_keep_the_order);
  RUN_TEST(test_color_filter_needs_two_equal_readings);
  RUN_TEST(test_navigator_goes_to_a_likely_cube);
  RUN_TEST(test_navigator_turns_toward_the_cube);
  RUN_TEST(test_navigator_falls_back_to_the_least_unlikely);
  RUN_TEST(test_navigator_explores_an_empty_map);
  RUN_TEST(test_yaw_bias_is_the_still_mean);
  RUN_TEST(test_yaw_integrates_a_quarter_turn);
  RUN_TEST(test_yaw_turn_reached_either_direction);
  RUN_TEST(test_timed_turn_ignores_the_gyro);
  RUN_TEST(test_gyro_turn_finishes_on_the_angle);
  RUN_TEST(test_gyro_turn_times_out_if_the_gyro_is_dead);
  RUN_TEST(test_loc_gate_gives_column_and_heading_not_row);
  RUN_TEST(test_loc_counts_columns_along_the_street);
  RUN_TEST(test_loc_running_past_the_last_column_means_we_miscounted);
  RUN_TEST(test_loc_turn_rotates_the_heading);
  RUN_TEST(test_loc_line_end_pins_down_the_row);
  RUN_TEST(test_loc_walking_up_then_the_end_confirms_the_count);
  RUN_TEST(test_loc_red_is_only_beyond_a_vertical_edge);
  RUN_TEST(test_loc_other_blue_pins_the_far_column);
  RUN_TEST(test_loc_a_missed_crossing_is_not_fatal);
  RUN_TEST(test_plan_trusts_a_known_pose);
  RUN_TEST(test_plan_anchors_when_the_row_is_unknown);
  RUN_TEST(test_forget_returns_to_prior);
  RUN_TEST(test_observation_moves_belief_the_right_way);
  RUN_TEST(test_observe_street_picks_the_right_slot);
  RUN_TEST(test_unlikely_places_are_not_worth_a_trip);
  RUN_TEST(test_likely_far_beats_unlikely_near);
  RUN_TEST(test_spin_duration);
  RUN_TEST(test_spin_refused);
  RUN_TEST(test_line_centered);
  RUN_TEST(test_line_left_is_negative);
  RUN_TEST(test_line_between_two_sensors);
  RUN_TEST(test_line_lost_and_junction);
  RUN_TEST(test_pid_sign_and_clamp);
  RUN_TEST(test_wrap_delta);
  RUN_TEST(test_spin_counts);
  RUN_TEST(test_color_classes);
  RUN_TEST(test_shoulder_mapping);
  RUN_TEST(test_shoulder_clamps_to_servo_travel);
  RUN_TEST(test_spin_duration);
  RUN_TEST(test_spin_refused);
  return UNITY_END();
}
