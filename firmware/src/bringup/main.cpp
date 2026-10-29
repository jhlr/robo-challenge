// M1 bring-up: serial menu (115200) to exercise every part one at a time.
#include <Arduino.h>
#include "core/arm.h"
#include "hal/drive.h"
#include "hal/sensors.h"
#include "hal/shoulder.h"
#include "params.h"
#include "pins.h"

namespace {
#if defined(CAR_DC)
hal::DcDrive drive;
#else
SMS_STS st;
hal::StsDrive drive(st);
#endif
hal::LineArray line8;
hal::Ultrasonic sonar;
hal::ColorSensor frontColor, floorColor;
hal::Imu imu;
hal::Servo claw(hal::kSg90);
bool streaming = false;
String line;

void help() {
  Serial.println(F(
      "h            help\n"
#if !defined(CAR_DC)
      "p            ping STS ids 1-3 (wheels 1-2, shoulder 3)\n"
      "i <old> <new> change a servo id (ONE servo on the bus; factory id 1)\n"
#endif
      "v <l> <r>    drive both sides, -1..1 (x stops, e prints encoder odometry on the STS car)\n"
      "x            stop wheels\n"
      "g <deg>      claw SG90 angle 0-180\n"
      "d <deg>      turn the shoulder STS by REAL degrees from its current position: + arm up, - arm down (|deg| <= kShoulderMaxMoveDeg)\n"
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

// True when `text` (after the command letter) is a plain number like "45" or "-9.8".
bool isNumber(const String& text) {
  const char* start = text.c_str();
  char* end = nullptr;
  strtof(start, &end);
  while (*end == ' ') ++end;
  return end != start && *end == '\0';
}

void handle(const String& cmd) {
  // Never act on a help line echoed back to us (TX looped to RX): "<deg>" placeholders and non-numeric arguments are rejected.
  const char letter = cmd.charAt(0);
  const bool needsNumber = letter == 'g' || letter == 'o' || letter == 'a' || letter == 'd';
  if (cmd.indexOf('<') >= 0 || (needsNumber && !isNumber(cmd.substring(1)))) {
    Serial.println("? this command needs a number, e.g. d 45");
    return;
  }
#if !defined(CAR_DC)
  int a = 0, b = 0;
#endif
  switch (cmd.charAt(0)) {
    case 'h': help(); break;
#if !defined(CAR_DC)
    case 'p':
      for (uint8_t id = params::kIdLeft; id <= params::kIdShoulder; ++id) Serial.printf("id %u: %s\n", id, st.Ping(id) != -1 ? "OK" : "no reply");
      break;
    case 'i':
      if (sscanf(cmd.c_str() + 1, "%d %d", &a, &b) == 2) {  // official ProgramEprom sequence
        st.unLockEprom(a);
        st.writeByte(a, SMS_STS_ID, b);
        st.LockEprom(b);
        Serial.println("done, p to confirm");
      } else {
        Serial.println("usage: i <old> <new>");
      }
      break;
    case 'm':
      a = cmd.substring(1).toInt();
      Serial.println(st.writeByte(a, SMS_STS_MODE, 0) && st.EnableTorque(a, 1) ? "ok" : "fail");
      break;
    case 't': {
      sscanf(cmd.c_str() + 1, "%d %d", &a, &b);
      const bool ok = st.WritePosEx(a, constrain(b, 0, 4095), 800, 30) != 0;
      delay(400);
      Serial.printf("%s, now at %d\n", ok ? "ok" : "fail", st.ReadPos(a));
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
    case 'd': {
#if defined(CAR_DC)
      Serial.println("no STS bus on the DC car");
      break;
#else
      const float deg = cmd.substring(1).toFloat();
      if (shoulderRotate(deg)) Serial.printf("turned %+.1f deg (%s)\n", deg, deg > 0 ? "up" : "down");
      else Serial.printf("REFUSED: %.1f deg is 0, beyond +-%.0f, out of the 0-4095 range, or the servo did not answer\n", deg, params::kShoulderMaxMoveDeg);
      break;
#endif
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
  hal::beginStsBus(st);
#endif
  line8.begin();
  sonar.begin();
  hal::beginIndicators();
  hal::beginBuses();
  Serial.printf("front rgb: %s | floor rgb: %s | imu: %s\n", frontColor.begin(Wire) ? "ok" : "FAIL",
                floorColor.begin(Wire1) ? "ok" : "FAIL", imu.begin() ? "ok" : "FAIL");
  // Servos attach AFTER the I2C buses: the bench wiring puts the claw signal on GPIO13, which shares a pin with the floor-sensor I2C (Wire1) in the bench wiring.
  claw.begin(pins::kClaw);
#if !defined(CAR_DC)
  Serial.printf("shoulder STS id %u: %s\n", params::kIdShoulder, shoulderBegin(st) ? "ok" : "no reply");
#endif
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
