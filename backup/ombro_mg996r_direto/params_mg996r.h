#pragma once
#include <stdint.h>
// Every number here is an ASSUMPTION until measured on the real robot.
namespace params {
// STS3215 bus IDs (wheels only; the shoulder is a TowerPro MG996R on PWM, direct on the pivot)
constexpr uint8_t kIdLeft = 1;
constexpr uint8_t kIdRight = 2;
// Signs include the mounting AND the gear reversal (STS -> centre gear -> wheel gear turns the wheel the other way); flip if the robot drives backwards.
#if defined(CAR_DC)
constexpr int kLeftSign = +1;
constexpr int kRightSign = +1;
#else
constexpr int kLeftSign = +1;
constexpr int kRightSign = -1;
#endif
constexpr bool kStsEchoes = false;  // true if TX and RX share one wire without an adapter

// Shoulder (MG996R direct on the bar pivot, no reduction): servo angle = center + sign * ratio * arm_angle.
// Arm angles: pick -9.8, mid -16, place +25.5 deg (braco/params.json) -> servo 80, 74, 115.5 deg. Sign +1 is an ASSUMPTION (positive servo angle raises the arm): flip if it moves backwards.
// Never power the MG996R from the ESP32 board: stall is ~2.5 A, use a separate 5-6 V supply with common GND.
constexpr float kShoulderRatio = 1.0f;
constexpr int kShoulderSign = +1;
constexpr float kShoulderCenterDeg = 90.0f;

// STS car: servo steps/s at full command; the lateral gear train sits between servo and wheel (see kGearRatio).
constexpr int kStsMaxSteps = 3400;
// DC car (yellow TT, no encoders): duty below this does not move the car; speed at full duty and spin rate must be measured (advance by time, turns by gyro).
constexpr float kDcMinDuty = 0.25f;
constexpr float kDcFullSpeedMmS = 400.0f;

// Geometry (mm). Wheel 66 mm; kTrackMm and kGearRatio (centre gear -> wheel gear, looked 1:1 in the photos) still to measure.
constexpr float kWheelDiaMm = 66.0f;
constexpr float kTrackMm = 100.0f;
constexpr float kGearRatio = 1.0f;  // servo revs per wheel rev
constexpr float kAxleToSensorMm = 60.0f;  // QRE8D ahead of the wheel axle

// QRE8D polarity: set true if a channel reads HIGH over the black line.
constexpr bool kLineIsHigh = false;

// Line follower
constexpr float kBaseSpeed = 0.45f;  // fraction of full command
constexpr float kKp = 0.9f;
constexpr float kKi = 0.0f;
constexpr float kKd = 0.08f;
constexpr float kSearchSpeedFrac = 0.4f;
constexpr char kRoute[] = "SLRU";  // per junction: S straight, L left, R right, U u-turn (cycles)
}  // namespace params
