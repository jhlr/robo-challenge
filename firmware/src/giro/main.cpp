// Bench test: both wheels forward forever (car in the air!). Re-sent every 100 ms so a servo reset or a lost frame does not stop it.
#include <Arduino.h>

#include <SCServo.h>

#include "hal/drive.h"
#include "params.h"

namespace {
SMS_STS st;
hal::StsDrive drive(st);
constexpr float kSpeed = 0.25f;
}  // namespace

void setup() {
  Serial.begin(115200);
  hal::beginStsBus(st);
  drive.begin();
  Serial.printf("ping esq=%d dir=%d\n", st.Ping(params::kIdLeft), st.Ping(params::kIdRight));
}

void loop() {
  drive.setSpeeds(kSpeed, kSpeed);
  delay(100);
}
