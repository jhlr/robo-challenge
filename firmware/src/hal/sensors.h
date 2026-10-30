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
  uint16_t readMm(uint32_t timeoutUs = 25000);  // 0xFFFF = no echo. Blocks up to timeoutUs.
};

struct ColorSample { uint16_t r, g, b, c; core::Floor floor; };

// PCA9548: only one channel talks at a time. Every sensor selects its channel before touching the bus; selecting the open channel costs nothing.
void muxSelect(uint8_t channel);

// TCS34725 behind a mux channel. Two instances (front, floor) because the address is fixed at 0x29.
class ColorSensor {
 public:
  bool begin(uint8_t muxChannel);  // call beginBuses() first
  bool read(ColorSample& out);
 private:
  uint8_t channel_ = 0;
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
void beginBuses();  // Wire on pins::kI2cSda/kI2cScl, facing the PCA9548
void beginIndicators();
void beep(uint16_t ms);
void led(bool on);

}  // namespace hal
