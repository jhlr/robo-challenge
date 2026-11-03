// M2: line follower with junction handling (advance -> settle/read floor -> turn per kRoute). Same app for both cars (see platformio.ini):
// STS car measures advance and turns with the servo encoders; the DC car has none (advance by time, turns by integrating the IMU gyro).
#include <Arduino.h>

#include "core/line.h"
#include "core/pid.h"
#include "core/turn.h"
#include "hal/drive.h"
#include "hal/sensors.h"
#include "params.h"
#include "pins.h"

namespace {
enum class State { Follow, Advance, Settle, Turn };

#if defined(CAR_DC)
hal::DcDrive drive;
#else
SMS_STS st;
hal::StsDrive drive(st);
#endif
hal::LineArray line8;
hal::ColorSensor floorColor;
core::Pid pid(params::kKp, params::kKi, params::kKd, 1.0f);

State state = State::Follow;
uint32_t stateSince = 0;
uint32_t holdoffUntil = 0;  // ignore junctions right after leaving one
size_t routeIdx = 0;
char action = 'S';
float lastPos = 0.0f;

// Motion in progress: distance (mm) or angle (deg) to cover, measured by encoders when the drive has them.
float motionTarget = 0.0f;
uint32_t motionStartMs = 0;
int turnDir = +1;  // +1 spin right, -1 spin left

void enter(State s) {
  state = s;
  stateSince = millis();
}

void beginAdvance(float mm) {
  motionTarget = mm;
  motionStartMs = millis();
  drive.resetTravel();
}

bool advanceDone() {
  if (drive.hasEncoders()) return drive.forwardMm() >= motionTarget;
  return (millis() - motionStartMs) * 1e-3f * drive.mmPerSecond(params::kBaseSpeed * 0.5f) >= motionTarget;
}

void beginTurn(float deg) {
  motionTarget = deg;
  motionStartMs = millis();
  drive.resetTravel();
}

// Closed loop on the encoders with a time limit; the DC car has none, so its turn is open loop (timed). core/turn.h owns the rule.
bool turnDone() {
  const bool closedLoop = drive.hasEncoders();
  const float yaw = closedLoop ? drive.spinDeg() : 0.0f;
  const core::TurnSpec spec{motionTarget, closedLoop, params::kTurnDegPerSec, params::kTurnTimeoutFactor};
  const uint32_t elapsed = millis() - motionStartMs;
  const bool done = core::turnFinished(spec, elapsed, yaw);
  if (done && closedLoop && !core::turnReached(yaw, motionTarget)) Serial.println("WARNING: turn ended by timeout, not by angle");
  return done;
}
}  // namespace

void setup() {
  Serial.begin(115200);
#if !defined(CAR_DC)
  hal::beginStsBus(st);
#endif
  line8.begin();
  hal::beginBuses();
  floorColor.begin(Wire1);
  hal::beginIndicators();
  drive.begin();
  if (!drive.hasEncoders()) Serial.println("WARNING: no encoders: turns are timed (open loop)");
  delay(1000);
  pid.dTau = params::kPidDerivTau;
  Serial.println("follow: running");
  enter(State::Follow);
}

void loop() {
  static uint32_t lastTick = millis();   // not 0: the first dt would be the whole uptime
  const uint32_t now = millis();
  if (now - lastTick < 10) return;  // 100 Hz
  const float dt = (now - lastTick) * 1e-3f;
  lastTick = now;

  float v[pins::kLineCount];
  line8.read(v);
  const core::LineState ln = core::readLine(v, pins::kLineCount);

  switch (state) {
    case State::Follow: {
      if (ln.junction && now > holdoffUntil) {
        action = params::kRoute[routeIdx % (sizeof(params::kRoute) - 1)];
        ++routeIdx;
        beginAdvance(params::kAxleToSensorMm);
        drive.setSpeeds(params::kBaseSpeed * 0.5f, params::kBaseSpeed * 0.5f);
        enter(State::Advance);
        break;
      }
      if (ln.lost) {  // spin toward the side the line was last seen
        const float s = params::kBaseSpeed * params::kSearchSpeedFrac;
        drive.setSpeeds(lastPos >= 0 ? s : -s, lastPos >= 0 ? -s : s);
        break;
      }
      lastPos = ln.pos;
      const float corr = pid.step(ln.pos, dt) * params::kBaseSpeed;
      drive.setSpeeds(params::kBaseSpeed + corr, params::kBaseSpeed - corr);  // line right => left faster
      break;
    }
    case State::Advance:
      if (advanceDone()) {
        drive.setSpeeds(0.0f, 0.0f);
        enter(State::Settle);
      }
      break;
    case State::Settle:
      if (now - stateSince >= 300) {
        hal::ColorSample c;
        if (floorColor.read(c)) Serial.printf("junction %u: floor %s, action %c\n", (unsigned)routeIdx, core::name(c.floor), action);
        float deg = 0.0f;
        if (action == 'L') { deg = 90.0f; turnDir = -1; }
        else if (action == 'R') { deg = 90.0f; turnDir = +1; }
        else if (action == 'U') { deg = 180.0f; turnDir = +1; }
        if (deg > 0.0f) {
          beginTurn(deg);
          const float s = params::kBaseSpeed * 0.5f;
          drive.setSpeeds(turnDir * s, -turnDir * s);
          enter(State::Turn);
        } else {
          pid.reset();
          holdoffUntil = now + 400;
          enter(State::Follow);
        }
      }
      break;
    case State::Turn:
      if (turnDone()) {
        drive.setSpeeds(0.0f, 0.0f);
        pid.reset();
        holdoffUntil = now + 400;
        enter(State::Follow);
      }
      break;
  }
}
