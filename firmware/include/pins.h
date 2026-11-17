#pragma once
// ESP32 classic (DevKit, CP2102) pin map, as wired on the bench (user, 2026-11-17). 3.3 V logic everywhere.
// Input-only on the classic ESP32: GPIO 34-39 (no output, no I2C, no pull-up). Strapping: 0, 2, 5, 12, 15.
// QRE8D line array: 8 digital inputs, S1 (left) .. S8 (right), in the order listed. Power it from 3.3 V (its outputs must not exceed 3.3 V).
namespace pins {
#if defined(CAR_DC)
// NOT REMAPPED for the classic ESP32: the DC car config still uses the S3 map (see below). Re-map before using CAR_DC.
constexpr int kMotorLA = 16;
constexpr int kMotorLB = 17;
constexpr int kMotorRA = 2;
constexpr int kMotorRB = 15;
constexpr int kMotorStby = 12;  // TB6612FNG STBY. GPIO12 is a strapping pin (must be low at boot): keep a 10 k pull-down to GND on it, the driver stays asleep until begin()
#else
// STS3215 bus on UART2 (half-duplex via the adapter board): RX 16, TX 17.
constexpr int kStsRx = 16;
constexpr int kStsTx = 17;
#endif

// Two I2C buses, no multiplexer: the two TCS34725 share address 0x29, so each one has a bus of its own.
// Wire (FLOOR RGB): SDA 32, SCL 4.   Wire1 (FRONT RGB): SDA 25, SCL 33.
// GPIO 35 (old floor SCL) is input-only on the classic ESP32, so it cannot drive SCL: moved to GPIO 4.
constexpr int kI2cSda = 32;
constexpr int kI2cScl = 4;
constexpr int kI2c2Sda = 25;
constexpr int kI2c2Scl = 33;

// Sonar: TRIG 14, ECHO 12 (ECHO needs a 5 V -> 3.3 V divider). GPIO 12 is a strapping pin: the HC-SR04 ECHO idles low, keep it that way at boot.
constexpr int kTrig = 14;
constexpr int kEcho = 12;

// QRE8D channels, S1 (left) .. S8 (right), as wired (user, 2026-11-17).
constexpr int kLine[] = {26, 27, 18, 19, 21, 22, 23, 34};
constexpr int kLineCount = sizeof(kLine) / sizeof(kLine[0]);

// SG90 signal (claw): GPIO13, free, not a strapping pin.
constexpr int kClaw = 13;
// Buzzer: GPIO5 (strapping, but silent at boot). Free after the line array.
constexpr int kBuzzer = 5;
// Indicator LED: the DevKit's on-board LED on GPIO2 (strapping, the LED does not affect boot).
constexpr int kLed = 2;
}  // namespace pins
