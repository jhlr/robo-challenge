// Bench: turns the shoulder STS (direct drive, 1 servo tick = 360/4096 deg of arm) by a delta from its CURRENT position.
// Serial: 'u' = 45 deg up, 'd' = 45 deg down, 's' = stop holding (torque off). Moves in kChunkTicks steps and aborts if current gets high or the servo stops following.
#include <Arduino.h>

#include <SCServo.h>

#include "params.h"
#include "pins.h"

SMS_STS sms_sts;
constexpr float kTicksPerDeg = 4096.0f / 360.0f;
constexpr int kChunkTicks = 64;       // ~5.6 deg per step
constexpr int kMaxCurrent = 400;      // raw units (~6.5 mA each): abort above this
constexpr float kDeltaDeg = 45.0f;

void turn(float deg) {  // deg > 0 = arm up (kShoulderUpSign decides the tick direction)
  const int id = params::kIdShoulder;
  int pos = sms_sts.ReadPos(id);
  if (pos < 0) { Serial.println("no position read, not moving"); return; }
  const int total = (int)(deg * kTicksPerDeg + (deg > 0 ? 0.5f : -0.5f)) * params::kShoulderUpSign;
  const int start = pos;
  const int goal = start + total;
  if (goal < 0 || goal > 4095) { Serial.printf("goal %d outside 0..4095, not moving\n", goal); return; }
  Serial.printf("start %d -> goal %d (%d ticks)\n", start, goal, total);
  const int dir = total > 0 ? 1 : -1;
  int target = start;
  while (target != goal) {
    target += dir * kChunkTicks;
    if ((dir > 0 && target > goal) || (dir < 0 && target < goal)) target = goal;
    sms_sts.WritePosEx(id, target, 300, 20);
    delay(400);
    const int p = sms_sts.ReadPos(id), cur = sms_sts.ReadCurrent(id);
    Serial.printf("target %d pos %d current %d\n", target, p, cur);
    if (p < 0 || abs(p - target) > 40 || abs(cur) > kMaxCurrent) {
      Serial.println("ABORT: not following or high current; holding where it is");
      if (p >= 0) sms_sts.WritePosEx(id, p, 300, 20);
      return;
    }
  }
  Serial.printf("done at %d\n", sms_sts.ReadPos(id));
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  sms_sts.pSerial = &Serial1;
  delay(1500);
  Serial.printf("id %d mode=%d pos=%d\n", params::kIdShoulder, sms_sts.ReadMode(params::kIdShoulder), sms_sts.ReadPos(params::kIdShoulder));
  Serial.println("'u' = 45 deg up, 'd' = 45 deg down");
}

void loop() {
  if (!Serial.available()) return;
  const int c = Serial.read();
  if (c == 'u') turn(kDeltaDeg);
  else if (c == 'd') turn(-kDeltaDeg);
}
