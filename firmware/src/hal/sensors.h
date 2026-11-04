#pragma once
#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include "core/color.h"
#include "pins.h"
// One thin wrapper per sensor. All hardware access in the firmware goes through here.
namespace hal {

class LineArray {  // QRE8D: 8 digital outputs
 public:
  void begin();
  void read(float* out);  // 1.0 = sees the line, 0.0 = floor
};

// HC-SR04: a 10 us pulse on TRIG, then ECHO stays HIGH for the sound's round trip. distance = echo time x speed of sound / 2 (params::kSoundMmPerUs).
class Ultrasonic {
 public:
  void begin();                                 // TRIG as an output idling LOW, ECHO as an input
  uint16_t readMm(uint32_t timeoutUs = 25000);  // 0xFFFF = no echo. Blocks up to timeoutUs (25 ms is ~4.3 m one way).
};

struct ColorSample { uint16_t r, g, b, c; core::Floor floor; };

// TCS34725, fixed address 0x29: two instances (front, floor) so each lives on its own I2C bus (Wire / Wire1).
class ColorSensor {
 public:
  bool begin(TwoWire& bus);  // call beginBuses() first
  bool read(ColorSample& out);
 private:
  TwoWire* bus_ = nullptr;
  Adafruit_TCS34725 tcs_{TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X};
  bool ok_ = false;
};

// Hobby servo on LEDC, 50 Hz, 16 bit. Pulse mapping is per model: minUs..maxUs spans 0..rangeDeg.
struct ServoSpec {
  float minUs, maxUs, rangeDeg;
  constexpr float usPerDeg() const { return (maxUs - minUs) / rangeDeg; }
};
constexpr ServoSpec kSg90{500.0f, 2400.0f, 180.0f};    // SG90/MG90S

class Servo {
 public:
  explicit Servo(const ServoSpec& spec = kSg90) : spec_(spec) {}
  void begin(int pin);
  void write(float degrees);  // 0..spec.rangeDeg
  void writeUs(float us);     // raw pulse width; the only call a continuous-rotation servo uses
 private:
  ServoSpec spec_;
  ::Servo servo_;  // ESP32Servo
};
void beginBuses();  // Wire on pins::kI2cSda/kI2cScl (floor RGB) and Wire1 on pins::kI2c2Sda/kI2c2Scl (front RGB)
void beginIndicators();
void beep(uint16_t ms);
void led(bool on);

}  // namespace hal
