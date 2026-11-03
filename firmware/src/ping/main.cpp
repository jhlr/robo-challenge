// Bench test from the professor's sketch: ping STS ids 1..3 on the bus (RX 16 / TX 17, 1 Mbaud). Prints the id that answers or an error.
#include <Arduino.h>

#include <SCServo.h>

#include "pins.h"

SMS_STS sms_sts;

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, pins::kStsRx, pins::kStsTx);
  sms_sts.pSerial = &Serial2;
  delay(1000);
}

void loop() {
  bool any = false;
  for (int id = 1; id <= 3; id++) {
    if (sms_sts.Ping(id) != -1) {
      Serial.printf("Servo ID:%d\n", id);
      any = true;
    }
  }
  if (!any) Serial.println("Ping servo ID error!");
  delay(1000);
}
