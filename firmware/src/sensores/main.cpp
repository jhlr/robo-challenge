// Bench test: HC-SR04, front RGB, floor RGB and MPU6050 streamed at 5 Hz. A sensor that failed to start is retried every line, so a loose wire can be fixed while watching.
// Boot also scans the I2C bus raw (no mux, then each PCA9548 channel) to show what is really answering. Send any key to rescan.
#include <Arduino.h>
#include <Wire.h>

#include "core/color.h"
#include "hal/sensors.h"
#include "params.h"
#include "pins.h"

namespace {
hal::Ultrasonic sonar;
hal::ColorSensor frontColor, floorColor;
hal::Imu imu;
bool frontOk = false, floorOk = false, imuOk = false;

void scanBus() {
  Serial.println(F("--- scan I2C ---"));
  Serial.print(F("direto (sem mux):"));
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
  const bool muxThere = (Wire.beginTransmission(params::kMuxAddr), Wire.endTransmission() == 0);
  Serial.printf("mux 0x%02X: %s\n", params::kMuxAddr, muxThere ? "responde" : "NAO responde");
  if (muxThere) {
    for (uint8_t ch = 0; ch < 8; ++ch) {
      hal::muxSelect(ch);
      Serial.printf("canal %u:", ch);
      for (uint8_t a = 1; a < 127; ++a) {
        if (a == params::kMuxAddr) continue;
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
      }
      Serial.println();
    }
  }
  Serial.println(F("----------------"));
}

void printColor(const char* label, hal::ColorSensor& sensor, bool& ok, uint8_t channel) {
  if (!ok) ok = sensor.begin(channel);
  hal::ColorSample c;
  if (!ok || !sensor.read(c)) {
    Serial.printf(" | %s: FALHA", label);
    ok = false;
    return;
  }
  const core::Hsv h = core::toHsv(c.r, c.g, c.b);
  Serial.printf(" | %s r%u g%u b%u c%u hue %.0f -> %s", label, c.r, c.g, c.b, c.c, h.h, core::name(c.floor));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  sonar.begin();
  hal::beginBuses();
  scanBus();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    scanBus();
  }
  const uint16_t mm = sonar.readMm();
  if (mm == 0xFFFF) Serial.print(F("sonar: sem eco")); else Serial.printf("sonar: %u mm", mm);
  printColor("frente", frontColor, frontOk, params::kMuxFrontRgb);
  printColor("chao", floorColor, floorOk, params::kMuxFloorRgb);
  if (!imuOk) imuOk = imu.begin();
  hal::ImuSample m;
  if (imuOk && imu.read(m)) Serial.printf(" | imu a %.2f %.2f %.2f g %.2f %.2f %.2f", m.ax, m.ay, m.az, m.gx, m.gy, m.gz);
  else { Serial.print(F(" | imu: FALHA")); imuOk = false; }
  Serial.println();
  delay(200);
}
