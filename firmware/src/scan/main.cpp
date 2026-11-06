// Bench: pings every STS id 0..20 (20 tries each) and prints how many answered, then repeats. Finds wrong/duplicate ids and a flaky bus. Moves nothing.
#include <Arduino.h>

#include <SCServo.h>

#include "pins.h"

SMS_STS sms_sts;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  sms_sts.pSerial = &Serial1;
  delay(1000);
}

void loop() {
  Serial.println("--- scan ids 0..20, 20 pings each ---");
  for (int id = 0; id <= 20; id++) {
    int ok = 0;
    for (int k = 0; k < 20; k++) {
      if (sms_sts.Ping(id) == id) ok++;
      delay(5);
    }
    if (ok > 0) Serial.printf("id %d: %d/20\n", id, ok);
  }
  Serial.println("--- done ---");
  delay(500);
}
