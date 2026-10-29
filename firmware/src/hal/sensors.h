#pragma once
#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>

#include "core/color.h"
// One thin wrapper per sensor. All hardware access in the firmware goes through here.
namespace hal {

class LineArray {  // QRE8D: 8 digital outputs
 public:
  void begin();
  void read(float* out);  // 1.0 = sees the line, 0.0 = floor
};

class Ultrasonic {
 public:
  void begin();
  uint16_t readMm();  // 0xFFFF = no echo. Blocks up to 25 ms.
};

struct ColorSample { uint16_t r, g, b, c; core::Floor floor; };

// TCS34725 on a given I2C bus. Two instances (front, floor) because the address is fixed at 0x29.
class ColorSensor {
 public:
  bool begin(TwoWire& bus);  // call beginBuses() first
  bool read(ColorSample& out);
 private:
  Adafruit_TCS34725 tcs_{TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X};
  bool ok_ = false;
};

struct ImuSample { float ax, ay, az, gx, gy, gz; };  // m/s^2 and rad/s

class Imu {
 public:
  bool begin();
  bool read(ImuSample& out);
 private:
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
  int pin_ = -1;
};
void beginBuses();  // Wire (IMU, front RGB) and Wire1 (floor RGB)
void beginIndicators();
void beep(uint16_t ms);
void led(bool on);

}  // namespace hal
