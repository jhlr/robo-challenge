// Bench: moves each STS that answers (ids 1..3) a little, one at a time, to prove the motors work. Prints mode/position/voltage/temperature/current at boot; moves only when it receives 'm' on the serial.
// Wheel mode (mode 1): spins slowly ~0.8 s forward, stops, spins the same time back (net travel ~0). Position mode (0): turns +kStepTicks from the CURRENT position, then back by the same delta.
#include <Arduino.h>

#include <SCServo.h>

#include "pins.h"

SMS_STS sms_sts;
constexpr int kWheelSteps = 400;   // servo steps/s: ~0.1 rev/s
constexpr int kStepTicks = 100;    // ~8.8 deg in position mode

bool answers(int id) {  // a duplicated id answers only some of the time: try a few times
  for (int k = 0; k < 8; k++) {
    if (sms_sts.Ping(id) == id) return true;
    delay(20);
  }
  return false;
}

void status(int id, const char* when) {
  const int pos = sms_sts.ReadPos(id);
  Serial.printf("[%s] id %d: mode=%d pos=%d volt=%.1fV temp=%dC current=%d\n", when, id, sms_sts.ReadMode(id), pos, sms_sts.ReadVoltage(id) / 10.0f,
                sms_sts.ReadTemper(id), sms_sts.ReadCurrent(id));
}

void moveOne(int id) {
  const int mode = sms_sts.ReadMode(id);
  const int p0 = sms_sts.ReadPos(id);
  if (p0 < 0) { Serial.printf("id %d: no position read, not moving\n", id); return; }
  if (mode == 1) {
    sms_sts.WriteSpe(id, kWheelSteps, 20); delay(800);
    sms_sts.WriteSpe(id, 0, 20); delay(600);
    status(id, "after forward");
    sms_sts.WriteSpe(id, -kWheelSteps, 20); delay(800);
    sms_sts.WriteSpe(id, 0, 20); delay(600);
    status(id, "after back");
  } else if (mode == 0) {
    const int dir = (p0 + kStepTicks <= 4095) ? 1 : -1;
    sms_sts.WritePosEx(id, p0 + dir * kStepTicks, 300, 20); delay(1500);
    const int p1 = sms_sts.ReadPos(id);
    status(id, "after step");
    if (p1 >= 0) sms_sts.WritePosEx(id, p1 - dir * kStepTicks, 300, 20);
    delay(1500);
    status(id, "after back");
  } else {
    Serial.printf("id %d: unexpected mode %d, not moving\n", id, mode);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  sms_sts.pSerial = &Serial1;
  delay(1500);
  for (int id = 1; id <= 3; id++) {
    if (answers(id)) status(id, "boot"); else Serial.printf("[boot] id %d: no answer\n", id);
  }
  Serial.println("send 'm' to move");
}

void loop() {
  if (!Serial.available() || Serial.read() != 'm') return;
  for (int id = 1; id <= 3; id++) {
    if (answers(id)) moveOne(id); else Serial.printf("id %d: no answer, skipped\n", id);
    delay(500);
  }
  Serial.println("done");
}
