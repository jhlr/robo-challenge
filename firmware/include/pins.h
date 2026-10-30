#pragma once
// ESP32 DevKit pin map. 3.3 V logic everywhere.
// QRE8D line array: 8 digital outputs, so no ADC needed. Power it from 3.3 V (its outputs must not exceed 3.3 V).
namespace pins {
#if defined(CAR_DC)
// Yellow-motor car (TB6612FNG): no STS bus, so UART2 pins 16/17 become motor outputs: AIN1/AIN2/BIN1/BIN2 with PWMA, PWMB tied to 3V3. GPIO2/15 are strapping pins (fine for H-bridge inputs that idle low).
constexpr int kMotorLA = 16;
constexpr int kMotorLB = 17;
constexpr int kMotorRA = 2;
constexpr int kMotorRB = 15;
constexpr int kMotorStby = 12;  // TB6612FNG STBY. GPIO12 is a strapping pin (must be low at boot): keep a 10 k pull-down to GND on it, the driver stays asleep until begin()
#else
constexpr int kStsRx = 16;  // UART2, half-duplex STS3215 bus via adapter board
constexpr int kStsTx = 17;
#endif

// One I2C bus, behind a PCA9548 8-channel multiplexer (params::kMux*): front RGB, floor RGB and the IMU. The two TCS34725 share address 0x29, hence the mux.
constexpr int kI2cSda = 22;
constexpr int kI2cScl = 23;

constexpr int kTrig = 14;
constexpr int kEcho = 12;  // HC-SR04 ECHO needs a 5 V -> 3.3 V divider. GPIO12 is a strapping pin (must be low at boot): the idle ECHO is low, keep it that way

constexpr int kLine[] = {36, 39, 35, 32, 33, 19, 23, 4};  // QRE8D channels, left to right
constexpr int kLineCount = sizeof(kLine) / sizeof(kLine[0]);

constexpr int kClaw = 18;      // SG90 signal (claw)
constexpr int kBuzzer = 26;
constexpr int kLed = 27;
}  // namespace pins
