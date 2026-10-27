#include <unity.h>

#include "core/arm.h"
#include "core/color.h"
#include "core/line.h"
#include "core/odo.h"
#include "core/pid.h"
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

int main() {
  UNITY_BEGIN();
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
  return UNITY_END();
}
