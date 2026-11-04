#include "sensors.h"

#include <Wire.h>

#include "pins.h"
#include "params.h"

namespace hal {

// ---- QRE8D line array ----
void LineArray::begin() {
  for (int i = 0; i < pins::kLineCount; ++i) pinMode(pins::kLine[i], INPUT);
}

void LineArray::read(float* out) {
  for (int i = 0; i < pins::kLineCount; ++i) {
    const bool high = digitalRead(pins::kLine[i]) == HIGH;
    out[i] = (high == params::kLineIsHigh) ? 1.0f : 0.0f;
  }
}

// ---- HC-SR04 ----
void Ultrasonic::begin() {
  pinMode(pins::kTrig, OUTPUT);
  digitalWrite(pins::kTrig, LOW);
  pinMode(pins::kEcho, INPUT);
}

uint16_t Ultrasonic::readMm(uint32_t timeoutUs) {
  digitalWrite(pins::kTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pins::kTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pins::kTrig, LOW);
  const unsigned long echoUs = pulseIn(pins::kEcho, HIGH, timeoutUs);   // waits for ECHO to rise, then times how long it stays HIGH; 0 = no echo within timeoutUs
  if (echoUs == 0) return 0xFFFF;
  return static_cast<uint16_t>(echoUs * params::kSoundMmPerUs / 2.0f + 0.5f);   // out and back: half the path
}

// ---- TCS34725 ----
bool ColorSensor::begin(TwoWire& bus) {
  bus_ = &bus;
  ok_ = tcs_.begin(TCS34725_ADDRESS, bus_);
  return ok_;
}

bool ColorSensor::read(ColorSample& out) {
  if (!ok_) return false;
  tcs_.getRawData(&out.r, &out.g, &out.b, &out.c);
  out.floor = core::classify(out.r, out.g, out.b, out.c);
  return true;
}

// ---- I2C buses ----
void beginBuses() {
  Wire.begin(pins::kI2cSda, pins::kI2cScl);
  Wire.setClock(100000);   // standard mode: the sensors are read at a few Hz and the bench wiring is jumper wires
  if (pins::kI2c2Sda >= 0) {
    Wire1.begin(pins::kI2c2Sda, pins::kI2c2Scl);
    Wire1.setClock(100000);
  }
}

// ---- servo (ESP32Servo) ----
void Servo::begin(int pin) {
  servo_.setPeriodHertz(50);
  servo_.attach(pin, spec_.minUs, spec_.maxUs);
}

void Servo::writeUs(float us) { servo_.writeMicroseconds(static_cast<int>(us)); }

void Servo::write(float degrees) {
  writeUs(spec_.minUs + constrain(degrees, 0.0f, spec_.rangeDeg) * spec_.usPerDeg());
}

// ---- buzzer / LED ----
void beginIndicators() {
  pinMode(pins::kBuzzer, OUTPUT);
  digitalWrite(pins::kBuzzer, LOW);
  if (pins::kLed >= 0) {
    pinMode(pins::kLed, OUTPUT);
    digitalWrite(pins::kLed, LOW);
  }
}

void beep(uint16_t ms) {  // active buzzer
  digitalWrite(pins::kBuzzer, HIGH);
  delay(ms);
  digitalWrite(pins::kBuzzer, LOW);
}

void led(bool on) {
  if (pins::kLed >= 0) digitalWrite(pins::kLed, on ? HIGH : LOW);
}

}  // namespace hal
