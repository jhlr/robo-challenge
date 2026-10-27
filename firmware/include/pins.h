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

constexpr int kI2cSda = 21;  // Wire: IMU + front RGB (TCS34725, fixed addr 0x29)
constexpr int kI2cScl = 22;
constexpr int kFloorSda = 13;  // Wire1: floor RGB (second TCS34725, same addr: needs its own bus)
constexpr int kFloorScl = 14;

constexpr int kTrig = 25;
constexpr int kEcho = 34;  // input-only pin; HC-SR04 ECHO needs a 5 V -> 3.3 V divider

constexpr int kLine[] = {36, 39, 35, 32, 33, 19, 23, 4};  // QRE8D channels, left to right
constexpr int kLineCount = sizeof(kLine) / sizeof(kLine[0]);

constexpr int kClaw = 18;      // SG90 signal (claw)
constexpr int kShoulder = 13;  // shoulder servo signal (MG996R, 4:1 gear reduction). BENCH WIRING (2026-10-27): shares GPIO13 with kFloorSda (floor RGB off while testing). Original pin: 5 (strapping pin, glitches at boot)
constexpr int kBuzzer = 26;
constexpr int kLed = 27;
}  // namespace pins
