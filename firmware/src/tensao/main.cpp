// Bench: writes the STS input-voltage window (params::kStsVoltMin..kStsVoltMax, EEPROM regs 15/14) to EVERY servo on the bus with one broadcast (id 254), then reads it back from ids 1..3.
// Use when a new STS (factory max 8.0 V) flashes its LED on the 12 V supply and its id is unknown or duplicated. Runs only on 'v' over the serial (one EEPROM write per servo per run).
#include <Arduino.h>

#include <SCServo.h>

#include "params.h"
#include "pins.h"

SMS_STS sms_sts;
constexpr uint8_t kBroadcast = 254, kRegMaxVolt = 14, kRegMinVolt = 15;

void report(const char* when) {
  Serial.printf("[%s]\n", when);
  for (int id = 1; id <= 3; id++)
    Serial.printf("  id %d: ping=%d maxV=%d minV=%d volt=%.1fV\n", id, sms_sts.Ping(id), sms_sts.readByte(id, kRegMaxVolt), sms_sts.readByte(id, kRegMinVolt), sms_sts.ReadVoltage(id) / 10.0f);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  sms_sts.pSerial = &Serial1;
  delay(1500);
  report("boot");
  Serial.println("send 'v' to write the voltage window to all servos");
}

void loop() {
  if (!Serial.available() || Serial.read() != 'v') return;
  sms_sts.unLockEprom(kBroadcast);
  sms_sts.writeByte(kBroadcast, kRegMaxVolt, params::kStsVoltMax);
  sms_sts.writeByte(kBroadcast, kRegMinVolt, params::kStsVoltMin);
  sms_sts.LockEprom(kBroadcast);
  delay(300);
  report("after broadcast");
}
