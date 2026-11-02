#pragma once
#include <stdint.h>
// Every number here is an ASSUMPTION until measured on the real robot.
namespace params {
// STS3215 bus IDs: one servo per lateral (2 wheels each) and one on the tower. Factory id is 1: set the others one servo at a time (bringup 'i').
constexpr uint8_t kIdLeft = 1;
constexpr uint8_t kIdRight = 2;
constexpr uint8_t kIdShoulder = 3;
// Signs include the mounting AND the gear reversal (STS -> centre gear -> wheel gear turns the wheel the other way); flip if the robot drives backwards.
#if defined(CAR_DC)
constexpr int kLeftSign = +1;
constexpr int kRightSign = +1;
#else
constexpr int kLeftSign = +1;
constexpr int kRightSign = -1;
#endif

// Shoulder: servo angle = center + sign * ratio * arm_angle. Assemble with the servo at kShoulderCenterDeg and the bars horizontal (alpha = 0): that is the tooth mesh phase.
// Arm angles: pick -9.8, mid -16, place +25.5 deg (cad/braco/params.json) -> servo 148, 173, 7 deg.
constexpr float kShoulderRatio = 4.0f;
constexpr int kShoulderSign = -1;
constexpr float kShoulderCenterDeg = 109.0f;
// The shoulder is an STS3215 in position mode (id kIdShoulder): the 12-bit position is the feedback, so moves are deltas from the CURRENT position (user rule), never absolute.
// ASSUMPTION until the bench: position increasing raises the arm. Flip kShoulderUpSign if the first small move goes the wrong way.
constexpr int kShoulderUpSign = +1;
constexpr uint16_t kShoulderSpeed = 400;  // steps/s, gentle for the first tests
constexpr uint8_t kShoulderAcc = 20;
// HARD LIMIT (user rule: never more than 90 deg at the shoulder): the largest single rotation accepted.
constexpr float kShoulderMaxMoveDeg = 90.0f;

// STS car: servo steps/s at full command; the lateral gear train sits between servo and wheel (see kGearRatio).
constexpr int kStsMaxSteps = 3400;
// STS input-voltage window in EEPROM (x0.1 V). Factory max is 8.0 V: on a 12 V supply every reply carries the voltage-error bit and the LED flashes (2026-10-31).
constexpr uint8_t kStsVoltMax = 140;
constexpr uint8_t kStsVoltMin = 40;
// DC car (yellow TT, no encoders): duty below this does not move the car; speed at full duty and spin rate must be measured (advance by time, turns by gyro).
constexpr float kDcMinDuty = 0.25f;
constexpr float kDcFullSpeedMmS = 400.0f;

// PCA9548 I2C multiplexer (A0-A2 to GND = 0x70 ASSUMED) and what sits on each channel.
constexpr uint8_t kMuxAddr = 0x70;
constexpr uint8_t kMuxFrontRgb = 0;
constexpr uint8_t kMuxFloorRgb = 1;
constexpr uint8_t kMuxImu = 2;

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
constexpr float kPidDerivTau = 0.1f;  // s, low-pass on the PID derivative (the line bar is digital: raw D spikes on every sensor step)
constexpr float kSearchSpeedFrac = 0.4f;
// In-place turns: spin rate at kBaseSpeed*0.5 is ASSUMED (measure it). It sets the open-loop turn time and the time limit of encoder/gyro turns.
constexpr float kTurnDegPerSec = 180.0f;
constexpr float kTurnTimeoutFactor = 3.0f;  // a closed-loop turn gives up after this many times the open-loop duration
constexpr char kRoute[] = "SLRU";  // per junction: S straight, L left, R right, U u-turn (cycles)
}  // namespace params
