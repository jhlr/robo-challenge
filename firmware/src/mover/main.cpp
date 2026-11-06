// Bench: moves ONE STS (kTestId) a little to prove the motor works. Prints mode/position/voltage/temperature/current at boot; moves only when it receives 'm' on the serial.
// Wheel mode (mode 1): spins slowly ~0.8 s forward, stops, spins the same time back (net travel ~0). Position mode (0): turns +kStepTicks from the CURRENT position, then back by the same delta.
#include <Arduino.h>

#include <SCServo.h>

#include "pins.h"

SMS_STS sms_sts;
constexpr int kTestId = 2;
constexpr int kWheelSteps = 400;   // servo steps/s: ~0.1 rev/s
constexpr int kStepTicks = 100;    // ~8.8 deg in position mode

void status(const char* when) {
  const int pos = sms_sts.ReadPos(kTestId);
  Serial.printf("[%s] id %d: mode=%d pos=%d volt=%.1fV temp=%dC current=%d\n", when, kTestId, sms_sts.ReadMode(kTestId), pos,
                sms_sts.ReadVoltage(kTestId) / 10.0f, sms_sts.ReadTemper(kTestId), sms_sts.ReadCurrent(kTestId));
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  sms_sts.pSerial = &Serial1;
  delay(1500);
  status("boot");
  Serial.println("send 'm' to move");
}

void loop() {
  if (!Serial.available()) return;
  if (Serial.read() != 'm') return;
  const int mode = sms_sts.ReadMode(kTestId);
  const int p0 = sms_sts.ReadPos(kTestId);
  if (p0 < 0) { Serial.println("no position read: not moving"); return; }
  if (mode == 1) {
    sms_sts.WriteSpe(kTestId, kWheelSteps, 20); delay(800);
    sms_sts.WriteSpe(kTestId, 0, 20); delay(600);
    status("after forward");
    sms_sts.WriteSpe(kTestId, -kWheelSteps, 20); delay(800);
    sms_sts.WriteSpe(kTestId, 0, 20); delay(600);
    status("after back");
  } else if (mode == 0) {
    const int dir = (p0 + kStepTicks <= 4095) ? 1 : -1;
    sms_sts.WritePosEx(kTestId, p0 + dir * kStepTicks, 300, 20); delay(1500);
    const int p1 = sms_sts.ReadPos(kTestId);
    status("after step");
    if (p1 >= 0) sms_sts.WritePosEx(kTestId, p1 - dir * kStepTicks, 300, 20);
    delay(1500);
    status("after back");
  } else {
    Serial.printf("unexpected mode %d: not moving\n", mode);
  }
}
