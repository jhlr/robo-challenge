// Bench test: HC-SR04, front RGB (Wire) and floor RGB (Wire1) streamed at 5 Hz. A sensor that failed to start is retried every line, so a loose wire can be fixed while watching.
// Boot also scans each I2C bus raw to show what is really answering (each TCS34725 should show 0x29 on its own bus). Send any key to rescan.
#include <Arduino.h>
#include <Wire.h>

#include "core/color.h"
#include "hal/sensors.h"
#include "params.h"
#include "pins.h"

namespace {
hal::Ultrasonic sonar;
hal::ColorSensor frontColor, floorColor;
bool frontOk = false, floorOk = false;

// Prints every address that answers on `bus`. A run of consecutive addresses is the lines floating or stuck, not devices.
void scanBus(TwoWire& bus, const char* name, int sda, int scl) {
  Serial.printf("%s (SDA GPIO%d, SCL GPIO%d):", name, sda, scl);
  for (uint8_t a = 1; a < 127; ++a) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
}

void scanAll() {
  Serial.println(F("--- scan I2C ---"));
  scanBus(Wire, "Wire  (frente)", pins::kI2cSda, pins::kI2cScl);
  if (pins::kI2c2Sda >= 0) scanBus(Wire1, "Wire1 (chao)  ", pins::kI2c2Sda, pins::kI2c2Scl);
  Serial.println(F("----------------"));
}

void printColor(const char* label, hal::ColorSensor& sensor, bool& ok, TwoWire& bus) {
  if (!ok) ok = sensor.begin(bus);
  hal::ColorSample c;
  if (!ok || !sensor.read(c)) {
    Serial.printf(" | %s: FALHA", label);
    ok = false;
    return;
  }
  const core::Hsv h = core::toHsv(c.r, c.g, c.b);
  Serial.printf(" | %s r%u g%u b%u c%u hue %.0f -> %s", label, c.r, c.g, c.b, c.c, h.h, core::name(c.floor));
}

void printIdle(const char* name, int sda, int scl) {
  Serial.printf("%s SDA(GPIO%d)=%d SCL(GPIO%d)=%d  ", name, sda, digitalRead(sda), scl, digitalRead(scl));
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  sonar.begin();
  // line levels before starting I2C: with a pull-up and a free bus both read 1; 0 = a line held to GND (short, stuck slave or damaged pin)
  for (const int pin : {pins::kI2cSda, pins::kI2cScl}) pinMode(pin, INPUT);
  if (pins::kI2c2Sda >= 0) for (const int pin : {pins::kI2c2Sda, pins::kI2c2Scl}) pinMode(pin, INPUT);
  delay(50);
  Serial.print(F("linhas I2C em repouso (esperado 1 e 1): "));
  printIdle("Wire", pins::kI2cSda, pins::kI2cScl);
  if (pins::kI2c2Sda >= 0) printIdle("Wire1", pins::kI2c2Sda, pins::kI2c2Scl);
  Serial.println();
  hal::beginBuses();
  scanAll();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    scanAll();
  }
  const uint16_t mm = sonar.readMm();
  if (mm == 0xFFFF) Serial.print(F("sonar: sem eco")); else Serial.printf("sonar: %u mm", mm);
  printColor("frente", frontColor, frontOk, Wire);
  if (pins::kI2c2Sda >= 0) printColor("chao", floorColor, floorOk, Wire1);
  Serial.println();
  delay(200);
}
