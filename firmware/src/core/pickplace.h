#pragma once
// Pure logic (no Arduino.h): the cube pick-and-place sequence as an explicit state machine. One step() runs ONE state against an `IO` (the real car or a test double).
//
// Flow (user, 2026-11-09), starting when the sonar has already seen a cube and the shoulder is at the top:
//   cube color -> back up 9 cm -> open claw -> shoulder to pick -> close claw -> check grip -> shoulder to top -> check nothing in front ->
//   drive to the platform -> check platform position (sonar) -> check platform color -> back up 13 cm -> shoulder to platform height -> open claw -> shoulder to top -> turn 180.
// Walking the map and finding the cube/platform are the navigation's job (GoToPlatform is a hook into it); this machine owns the arm and claw order and every check between them.
//
// IO contract (every call returns true on success):
//   cubeColorOk()            front color sensor shows the expected cube color
//   backUp(mm)               straight back by mm (encoders)
//   clawOpen() / clawClose() move the claw
//   shoulderTo(Level)        small checked steps to a calibrated height; false = blocked, the shoulder holds where it stopped
//   holding()                the claw is gripping something
//   frontClear()             sonar sees no cube in front
//   goToPlatform()           navigation brings the car in front of the platform
//   platformPositionOk()     sonar confirms the platform is where it should be
//   platformColorOk()        floor/front color matches the carried cube
//   turn180()                spin half a turn
namespace core {

enum class Level : uint8_t { Up, Place, Pick };

enum class PpState : uint8_t {
  CheckColor, BackUpPick, OpenForPick, LowerPick, ClosePick, CheckGrip, RaiseCarry, CheckClear,
  GoToPlatform, CheckPlatformPos, CheckPlatformColor, BackUpPlace, LowerPlace, Release, RaiseAfter, Turn180,
  Done, Aborted
};

enum class PpFail : uint8_t { None, WrongColor, BackUp, Claw, Shoulder, NoGrip, FrontBlocked, Platform, PlatformPos, PlatformColor, Turn };

constexpr float kBackUpPickMm = 90.0f;     // user: ~9 cm
constexpr float kBackUpPlaceMm = 130.0f;   // user: 13 cm
constexpr uint8_t kGripTries = 3;          // closings before giving up on a cube

struct PickPlace {
  PpState state = PpState::CheckColor;
  PpFail fail = PpFail::None;
  uint8_t gripTries = 0;
  bool carrying = false;  // true from the first successful grip until the release

  bool running() const { return state != PpState::Done && state != PpState::Aborted; }

  // Runs the current state once. Returns running(). On a failure it records `fail`, and (unless the shoulder itself is the problem) lifts the shoulder to the top before stopping.
  template <class IO>
  bool step(IO& io) {
    switch (state) {
      case PpState::CheckColor: next(io.cubeColorOk(), PpState::BackUpPick, PpFail::WrongColor); break;
      case PpState::BackUpPick: next(io.backUp(kBackUpPickMm), PpState::OpenForPick, PpFail::BackUp); break;
      case PpState::OpenForPick: next(io.clawOpen(), PpState::LowerPick, PpFail::Claw); break;
      case PpState::LowerPick: shoulder(io, Level::Pick, PpState::ClosePick); break;
      case PpState::ClosePick: next(io.clawClose(), PpState::CheckGrip, PpFail::Claw); break;
      case PpState::CheckGrip:
        if (io.holding()) { carrying = true; state = PpState::RaiseCarry; }
        else if (++gripTries < kGripTries) state = PpState::OpenForPick;   // already at the pick height: open and close again
        else abort(io, PpFail::NoGrip);
        break;
      case PpState::RaiseCarry: shoulder(io, Level::Up, PpState::CheckClear); break;
      case PpState::CheckClear: next(io.frontClear(), PpState::GoToPlatform, PpFail::FrontBlocked); break;
      case PpState::GoToPlatform: next(io.goToPlatform(), PpState::CheckPlatformPos, PpFail::Platform); break;
      case PpState::CheckPlatformPos: next(io.platformPositionOk(), PpState::CheckPlatformColor, PpFail::PlatformPos); break;
      case PpState::CheckPlatformColor: next(io.platformColorOk(), PpState::BackUpPlace, PpFail::PlatformColor); break;
      case PpState::BackUpPlace: next(io.backUp(kBackUpPlaceMm), PpState::LowerPlace, PpFail::BackUp); break;
      case PpState::LowerPlace: shoulder(io, Level::Place, PpState::Release); break;
      case PpState::Release:
        if (io.clawOpen()) { carrying = false; state = PpState::RaiseAfter; } else abort(io, PpFail::Claw);
        break;
      case PpState::RaiseAfter: shoulder(io, Level::Up, PpState::Turn180); break;
      case PpState::Turn180: next(io.turn180(), PpState::Done, PpFail::Turn); break;
      case PpState::Done: case PpState::Aborted: break;
    }
    return running();
  }

  template <class IO>
  void run(IO& io) { while (step(io)) {} }

 private:
  void next(bool ok, PpState to, PpFail why) { if (ok) state = to; else { fail = why; state = PpState::Aborted; } }
  template <class IO>
  void abort(IO& io, PpFail why) {
    fail = why;
    state = PpState::Aborted;
    io.shoulderTo(Level::Up);   // best effort, one try: a blocked shoulder just holds
  }
  template <class IO>
  void shoulder(IO& io, Level to, PpState then) {
    if (io.shoulderTo(to)) state = then;
    else { fail = PpFail::Shoulder; state = PpState::Aborted; }   // no second command: re-read the position before moving again
  }
};

}  // namespace core
