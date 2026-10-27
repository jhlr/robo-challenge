// M1 bring-up: serial menu (115200) to exercise every part one at a time.
#include <Arduino.h>
#include "core/arm.h"
#include "hal/drive.h"
#include "hal/sensors.h"
#include "params.h"
#include "pins.h"

namespace {
#if defined(CAR_DC)
hal::DcDrive drive;
#else
sts::Bus bus;
hal::StsDrive drive(bus);
#endif
hal::LineArray line8;
hal::Ultrasonic sonar;
hal::ColorSensor frontColor, floorColor;
hal::Imu imu;
hal::Sg90 claw, shoulder;
bool streaming = false;
String line;

void help() {
  Serial.println(F(
      "h            help\n"
#if !defined(CAR_DC)
      "p            ping STS wheel ids 1-2\n"
#endif
      "v <l> <r>    drive both sides, -1..1 (x stops, e prints encoder odometry on the STS car)\n"
      "x            stop wheels\n"
      "g <deg>      claw SG90 angle 0-180\n"
      "o <deg>      shoulder servo angle 0-180 (4:1 reduction: arm moves deg/4)\n"
      "a <deg>      shoulder by ARM angle in degrees (-16..25.5), uses params::kShoulder*\n"
      "s            dump every sensor once\n"
      "S            toggle 10 Hz sensor stream\n"
      "b            beep + LED blink"));
}

void printColor(const char* label, hal::ColorSensor& sensor) {
  hal::ColorSample c;
  if (!sensor.read(c)) {
    Serial.printf(" | %s: n/a", label);
    return;
  }
  const core::Hsv h = core::toHsv(c.r, c.g, c.b);
  Serial.printf(" | %s rgbc %u %u %u %u hue %.0f sat %.2f -> %s", label, c.r, c.g, c.b, c.c, h.h, h.s,
                core::name(c.floor));
}

void dumpSensors() {
  float v[pins::kLineCount];
  line8.read(v);
  Serial.print(F("line:"));
  for (int i = 0; i < pins::kLineCount; ++i) Serial.print(v[i] > 0.5f ? " #" : " .");
  const uint16_t mm = sonar.readMm();
  Serial.print(F(" | sonar mm: "));
  if (mm == 0xFFFF) Serial.print(F("none")); else Serial.print(mm);
  printColor("front", frontColor);
  printColor("floor", floorColor);
  hal::ImuSample m;
  if (imu.read(m)) Serial.printf(" | imu a %.2f %.2f %.2f g %.2f %.2f %.2f", m.ax, m.ay, m.az, m.gx, m.gy, m.gz);
  else Serial.print(F(" | imu: n/a"));
  Serial.println();
}

void handle(const String& cmd) {
#if !defined(CAR_DC)
  int a = 0, b = 0;
#endif
  switch (cmd.charAt(0)) {
    case 'h': help(); break;
#if !defined(CAR_DC)
    case 'p':
      for (uint8_t id = params::kIdLeft; id <= params::kIdRight; ++id) Serial.printf("id %u: %s\n", id, bus.ping(id) ? "OK" : "no reply");
      break;
    case 'm':
      a = cmd.substring(1).toInt();
      Serial.println(bus.setMode(a, sts::Mode::Servo) && bus.setTorque(a, true) ? "ok" : "fail");
      break;
    case 't': {
      sscanf(cmd.c_str() + 1, "%d %d", &a, &b);
      uint16_t pos = 0;
      const bool ok = bus.moveTo(a, constrain(b, 0, 4095), 800, 30);
      delay(400);
      Serial.printf("%s, now at %s\n", ok ? "ok" : "fail", bus.readPosition(a, pos) ? String(pos).c_str() : "?");
      break;
    }
#endif
    case 'v': {
      float l = 0, r = 0;
      sscanf(cmd.c_str() + 1, "%f %f", &l, &r);
      drive.begin();  // wheel mode / PWM attach, travel counters to zero
      drive.setSpeeds(l, r);
      Serial.println("ok (x to stop, e for odometry)");
      break;
    }
    case 'x': drive.setSpeeds(0.0f, 0.0f); Serial.println("stopped"); break;
    case 'e':
      if (drive.hasEncoders()) Serial.printf("forward %.1f mm, spin %.1f deg (since v)\n", drive.forwardMm(), drive.spinDeg());
      else Serial.println("this car has no encoders");
      break;
    case 'g': claw.write(cmd.substring(1).toFloat()); Serial.println("ok"); break;
    case 'o': {
      const float want = cmd.substring(1).toFloat();
      const float deg = constrain(want, 0.0f, params::kShoulderMaxServoDeg);
      shoulder.write(deg);
      Serial.printf("servo %.1f deg%s\n", deg, deg != want ? " (LIMITED to kShoulderMaxServoDeg)" : "");
      break;
    }
    case 'a': {
      const float arm = cmd.substring(1).toFloat();
      bool cut = false;
      const float servo = core::shoulderServoDeg(arm, params::kShoulderCenterDeg, params::kShoulderSign, params::kShoulderRatio, &cut);
      const float safe = constrain(servo, 0.0f, params::kShoulderMaxServoDeg);
      shoulder.write(safe);
      Serial.printf("arm %.1f deg -> servo %.1f deg%s%s\n", arm, safe, cut ? " (CLAMPED to servo travel)" : "", safe != servo ? " (LIMITED to kShoulderMaxServoDeg)" : "");
      break;
    }
    case 's': dumpSensors(); break;
    case 'S': streaming = !streaming; break;
    case 'b': hal::led(true); hal::beep(150); hal::led(false); break;
    default: Serial.println("? (h for help)");
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
#if !defined(CAR_DC)
  bus.begin(Serial2, pins::kStsRx, pins::kStsTx, params::kStsEchoes);
#endif
  line8.begin();
  sonar.begin();
  hal::beginIndicators();
  hal::beginBuses();
  Serial.printf("front rgb: %s | floor rgb: %s | imu: %s\n", frontColor.begin(Wire) ? "ok" : "FAIL",
                floorColor.begin(Wire1) ? "ok" : "FAIL", imu.begin() ? "ok" : "FAIL");
  // Servos attach AFTER the I2C buses: the bench wiring puts the claw signal on GPIO13, which shares a pin with the floor-sensor I2C (Wire1) in the bench wiring.
  claw.begin(pins::kClaw);
  shoulder.begin(pins::kShoulder);
  help();
}

void loop() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line.trim();
      if (line.length()) handle(line);
      line = "";
    } else {
      line += ch;
    }
  }
  static uint32_t last = 0;
  if (streaming && millis() - last >= 100) {
    last = millis();
    dumpSensors();
  }
}
