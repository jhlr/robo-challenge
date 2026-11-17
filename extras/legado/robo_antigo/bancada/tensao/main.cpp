// Bench: configures ONE new STS that is alone on the bus. 'v' writes the input-voltage window (params::kStsVoltMin..kStsVoltMax) with hal::ensureStsVoltageWindow, so the LED stops flashing on the 12 V supply (factory max is 8.0 V).
// 'i' moves its id from the factory 1 to params::kIdShoulder (3), so it no longer collides with the left wheel. Both refuse to run unless exactly one servo answers (ids 1..20), and each writes EEPROM once per run.
#include <Arduino.h>

#include <SCServo.h>

#include "hal/drive.h"
#include "params.h"
#include "pins.h"

SMS_STS sms_sts;

int onlyServo() {  // id of the single servo on the bus, or -1 (none, or more than one: nothing is written)
  int found = -1, n = 0;
  for (int id = 1; id <= 20; id++) {
    for (int k = 0; k < 3; k++) {
      if (sms_sts.Ping(id) == id) { found = id; n++; break; }
    }
  }
  if (n != 1) Serial.printf("%d servos answered (need exactly 1): not writing\n", n);
  return n == 1 ? found : -1;
}

void report(const char* when) {
  Serial.printf("[%s]\n", when);
  for (int id = 1; id <= 20; id++)
    if (sms_sts.Ping(id) == id) Serial.printf("  id %d: volt=%.1fV temp=%dC mode=%d\n", id, sms_sts.ReadVoltage(id) / 10.0f, sms_sts.ReadTemper(id), sms_sts.ReadMode(id));
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  sms_sts.pSerial = &Serial1;
  delay(1500);
  report("boot");
  Serial.println("'v' = voltage window, 'i' = id -> 3");
}

void loop() {
  if (!Serial.available()) return;
  const char c = Serial.read();
  if (c == 'v') {
    const int id = onlyServo();
    if (id < 0) return;
    Serial.printf("voltage window on id %d: %s\n", id, hal::ensureStsVoltageWindow(sms_sts, id) ? "OK" : "FAILED");
  } else if (c == 'i') {
    const int id = onlyServo();
    if (id < 0) return;
    if (id == params::kIdShoulder) { Serial.println("already the shoulder id"); return; }
    sms_sts.unLockEprom(id);
    sms_sts.writeByte(id, SMS_STS_ID, params::kIdShoulder);
    sms_sts.LockEprom(params::kIdShoulder);
    delay(300);
    report("after id change");
  }
}
