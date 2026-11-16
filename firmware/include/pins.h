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

// Two I2C buses, no multiplexer : the two TCS34725 share address 0x29, so each one has a bus of its own, as wired on the bench.
// Wire (FLOOR RGB): SDA 21, SCL 22.   Wire1 (FRONT RGB): SDA 18, SCL 19.   As wired on the bench.
constexpr int kI2cSda = 21;
constexpr int kI2cScl = 22;
#if defined(CAR_DC)
constexpr int kI2c2Sda = -1;   // the DC car keeps the sonar on 18/19: the front RGB is unavailable there
constexpr int kI2c2Scl = -1;
constexpr int kTrig = 18;
constexpr int kEcho = 19;      // HC-SR04 ECHO needs a 5 V -> 3.3 V divider
#else
constexpr int kI2c2Sda = 18;
constexpr int kI2c2Scl = 19;
// The sonar moved off 18/19 (now Wire1) to the two free GPIOs: TRIG 15, ECHO 23, as wired on the bench (ECHO needs a 5 V -> 3.3 V divider).
constexpr int kTrig = 15;
constexpr int kEcho = 23;
#endif

// QRE8D channels, S1 (left) .. S8 (right), as wired (user, 2026-11-09). S7 = GPIO35 and S8 = GPIO34 are INPUT-ONLY: the RC reading needs to drive the pin to charge the capacitor, so those two never respond
// (2500 us on any floor). GPIO13 and GPIO4 are free: move the S7/S8 wires there, put 13, 4 at the end of this list and clear BarraRC::mortos in robo/main.cpp to get all 8.
constexpr int kLine[] = {14, 27, 26, 25, 33, 32, 35, 34};
constexpr int kLineCount = sizeof(kLine) / sizeof(kLine[0]);

// SG90 signal (claw). Unassigned: it used to say 22, which is the SCL of the floor RGB. Jaw is a stub for now (hal/claw.h); set the GPIO here when it is wired (free: none ideal, see docs).
constexpr int kClaw = -1;
// Indicators must not share a pin with the line array (26/27 used to be here and turned S4/S5 into outputs). GPIO5 is free; the STS car uses the DevKit's on-board LED
// (GPIO2). The DC car drives GPIO2 as a motor input, so it has no LED (-1).
constexpr int kBuzzer = 5;
#if defined(CAR_DC)
constexpr int kLed = -1;
#else
constexpr int kLed = 2;
#endif
}  // namespace pins
