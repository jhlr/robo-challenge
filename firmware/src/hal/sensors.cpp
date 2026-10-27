#include "sensors.h"

#include <Adafruit_MPU6050.h>
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
  pinMode(pins::kEcho, INPUT);
  digitalWrite(pins::kTrig, LOW);
}

uint16_t Ultrasonic::readMm() {
  digitalWrite(pins::kTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pins::kTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pins::kTrig, LOW);
  const unsigned long us = pulseIn(pins::kEcho, HIGH, 25000);
  return us == 0 ? 0xFFFF : static_cast<uint16_t>(us * 0.1715f);
}

// ---- TCS34725 ----
bool ColorSensor::begin(TwoWire& bus) {
  ok_ = tcs_.begin(TCS34725_ADDRESS, &bus);
  return ok_;
}

bool ColorSensor::read(ColorSample& out) {
  if (!ok_) return false;
  tcs_.getRawData(&out.r, &out.g, &out.b, &out.c);
  out.floor = core::classify(out.r, out.g, out.b, out.c);
  return true;
}

// ---- I2C buses ----
// Wire: IMU + front RGB. Wire1: floor RGB. Both TCS34725 sit at 0x29, hence one per bus.
void beginBuses() {
  Wire.begin(pins::kI2cSda, pins::kI2cScl);
  Wire.setClock(400000);
  Wire1.begin(pins::kFloorSda, pins::kFloorScl);
  Wire1.setClock(400000);
}

// ---- MPU6050 on Wire ----
namespace {
Adafruit_MPU6050 mpu;
}  // namespace

bool Imu::begin() {
  ok_ = mpu.begin(0x68, &Wire);  // ASSUMPTION: MPU6050 at 0x68
  return ok_;
}

bool Imu::read(ImuSample& out) {
  if (!ok_) return false;
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  out = {a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z};
  return true;
}

// ---- SG90 (LEDC, 16-bit) ----
void Sg90::begin(int pin) {
  pin_ = pin;
  ledcAttach(pin_, 50, 16);
}

void Sg90::write(float degrees) {
  const float us = 500.0f + constrain(degrees, 0.0f, 180.0f) * (1900.0f / 180.0f);
  ledcWrite(pin_, static_cast<uint32_t>(us * 65535.0f / 20000.0f));
}

// ---- buzzer / LED ----
void beginIndicators() {
  pinMode(pins::kBuzzer, OUTPUT);
  pinMode(pins::kLed, OUTPUT);
  digitalWrite(pins::kBuzzer, LOW);
  digitalWrite(pins::kLed, LOW);
}

void beep(uint16_t ms) {  // active buzzer
  digitalWrite(pins::kBuzzer, HIGH);
  delay(ms);
  digitalWrite(pins::kBuzzer, LOW);
}

void led(bool on) { digitalWrite(pins::kLed, on ? HIGH : LOW); }

}  // namespace hal
