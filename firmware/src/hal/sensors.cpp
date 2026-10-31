#include "sensors.h"

#include <Adafruit_MPU6050.h>
#include <TCA9548.h>
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
uint16_t Ultrasonic::readMm(uint32_t timeoutUs) {
  const unsigned int us = sonar_.ping(timeoutUs / US_ROUNDTRIP_CM);   // max distance (cm) from the caller's timeout
  return us == 0 ? 0xFFFF : static_cast<uint16_t>(us * 10 / US_ROUNDTRIP_CM);
}

// ---- TCS34725 ----
bool ColorSensor::begin(uint8_t muxChannel) {
  channel_ = muxChannel;
  muxSelect(channel_);
  ok_ = tcs_.begin(TCS34725_ADDRESS, &Wire);
  return ok_;
}

bool ColorSensor::read(ColorSample& out) {
  if (!ok_) return false;
  muxSelect(channel_);
  tcs_.getRawData(&out.r, &out.g, &out.b, &out.c);
  out.floor = core::classify(out.r, out.g, out.b, out.c);
  return true;
}

// ---- I2C bus + PCA9548 ----
namespace {
TCA9548 mux(params::kMuxAddr, &Wire);  // PCA9548 is register-compatible; the library skips the write when the channel is already open
}  // namespace

void muxSelect(uint8_t channel) { mux.selectChannel(channel); }

void beginBuses() {
  Wire.begin(pins::kI2cSda, pins::kI2cScl);
  Wire.setClock(400000);
  mux.begin();
}

// ---- MPU6050 (mux channel params::kMuxImu) ----
namespace {
Adafruit_MPU6050 mpu;
}  // namespace

bool Imu::begin() {
  muxSelect(params::kMuxImu);
  ok_ = mpu.begin(0x68, &Wire);  // ASSUMPTION: MPU6050 at 0x68
  return ok_;
}

bool Imu::read(ImuSample& out) {
  if (!ok_) return false;
  muxSelect(params::kMuxImu);
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  out = {a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z};
  return true;
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
