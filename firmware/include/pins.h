#pragma once
// ESP32-S3 (DevKitC-1) pin map, 2026-11-16. 3.3 V logic everywhere. AVOID on the S3: GPIO0, 3, 45, 46 (strapping), 19/20 (USB D-/D+), 26-32 (flash), 33-37 (octal PSRAM on N8R8 modules), 43/44 (UART0 console).
// Free and used below: 1-2, 4-9 (line bar), 10-18, 21, 38-42, 47.
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
constexpr int kStsRx = 18;  // UART2 on the matrix (any GPIO on the S3), half-duplex STS3215 bus via adapter board
constexpr int kStsTx = 17;
#endif

// Two I2C buses, no multiplexer : the two TCS34725 share address 0x29, so each one has a bus of its own, as wired on the bench.
// Wire (FLOOR RGB): SDA 11, SCL 12.   Wire1 (FRONT RGB): SDA 13, SCL 14.   Suggested S3 map (2026-11-16): wire it like this or tell me the new pins.
constexpr int kI2cSda = 11;
constexpr int kI2cScl = 12;
#if defined(CAR_DC)
constexpr int kI2c2Sda = -1;   // the DC car keeps the sonar on 18/19: the front RGB is unavailable there
constexpr int kI2c2Scl = -1;
constexpr int kTrig = 18;
constexpr int kEcho = 19;      // HC-SR04 ECHO needs a 5 V -> 3.3 V divider
#else
constexpr int kI2c2Sda = 13;
constexpr int kI2c2Scl = 14;
// Sonar: TRIG 15, ECHO 16 (ECHO needs a 5 V -> 3.3 V divider).
constexpr int kTrig = 15;
constexpr int kEcho = 16;
#endif

// QRE8D channels, S1 (left) .. S8 (right), as wired (user, 2026-11-16): the board is now an ESP32-S3 and the bar sits on GPIO 1, 2, 4, 5, 6, 7, 8, 9 (S1..S8). All of them can drive the pin, so the RC reading
// charges every capacitor and all 8 sensors respond (params::kLineDeadMask = 0). NOTE: the other pins in this file (STS UART, I2C, sonar, LED) are still the classic ESP32 DevKit map and platformio.ini still
// builds `esp32dev`: they must be re-mapped for the S3 board before flashing (GPIO 6-11 are the flash on a classic ESP32, so this bar map only makes sense on the S3).
constexpr int kLine[] = {1, 2, 4, 5, 6, 7, 8, 9};
constexpr int kLineCount = sizeof(kLine) / sizeof(kLine[0]);

// SG90 signal (claw). Unassigned: it used to say 22, which is the SCL of the floor RGB. Jaw is a stub for now (hal/claw.h); set the GPIO here when it is wired (free: none ideal, see docs).
constexpr int kClaw = -1;   // when wired: GPIO10 (free)
// Indicators must not share a pin with the line array (26/27 used to be here and turned S4/S5 into outputs). GPIO5 is free; the STS car uses the DevKit's on-board LED
// (GPIO2). The DC car drives GPIO2 as a motor input, so it has no LED (-1).
constexpr int kBuzzer = 21;
#if defined(CAR_DC)
constexpr int kLed = -1;
#else
constexpr int kLed = -1;   // the S3 DevKitC onboard LED is an RGB NeoPixel (GPIO48), not a plain digital LED
#endif
}  // namespace pins
