// Bench test from the professor's sketch: ping STS ids 1..3 at 1 Mbaud on RX 16 / TX 17 (the pins the adapter is wired to).
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
  for (int i = 1; i <= 3; i++) {
    const int id = sms_sts.Ping(i);
    if (id > 0) {
      Serial.printf("Servo ID:%d\n", id);
      delay(100);
    } else {
      Serial.printf("Ping servo ID %d error!\n", i);
      delay(500);
    }
  }
}
