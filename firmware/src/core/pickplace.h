#pragma once
// Pure logic (no Arduino.h): the cube pick and place sequences as an explicit state machine. One step() runs ONE state against an `IO` (the real car or a test double).
//
// Two phases, because the navigation (SeguidorRC robo.h) owns everything between them (walking the map, finding the cube, driving to the platform, turning):
//   PICK  (`PickPlace::pick()`)   starts with the cube in front and the shoulder at the top:
//     cube color -> back up 9 cm -> open claw -> shoulder to pick -> close claw -> check grip -> shoulder to top -> check nothing in front.
//     No grip: shoulder to top, check again, reposition, open, lower, close, check again (up to kGripTries closings).
//   PLACE (`PickPlace::place()`)  starts in front of the platform, cube in the claw:
//     platform position (sonar) -> platform color -> back up 13 cm -> shoulder to platform height -> open claw -> shoulder to top.
// Carrying a cube means STORING IT NOW: no state here collects anything else; cubes seen on the way are only remembered by the navigation (robo.h blocks that street temporarily,
// remembers the cube for after the delivery and goes around it).
//
// IO contract (every call returns true on success):
//   cubeColorOk()            front color sensor still shows a cube color
//   backUp(mm)               straight back by mm (encoders)
//   clawOpen() / clawClose() move the claw
//   shoulderTo(Level)        small checked steps to a calibrated height; false = blocked, the shoulder holds where it stopped
//   holding()                with the shoulder up: the cube has left the front (sonar). Sem cubo na garra, ele continua na frente
//   reposition()             after a failed grip, with the shoulder up: re-aim at the cube (sonar) and adjust the distance
//   frontClear()             sonar sees no cube in front
//   platformPositionOk()     sonar confirms the platform is where it should be
//   platformColorOk()        front color matches the carried cube
#include <cstdint>

namespace core {

enum class Level : uint8_t { Up, Place, Pick };

enum class PpState : uint8_t {
  CheckColor, BackUpPick, OpenForPick, LowerPick, ClosePick, CheckGrip, RaiseRetry, CheckGripRaised, Reposition, RaiseCarry, CheckClear,
  CheckPlatformPos, CheckPlatformColor, BackUpPlace, LowerPlace, Release, RaiseAfter,
  Done, Aborted
};

enum class PpFail : uint8_t { None, WrongColor, BackUp, Reposition, Claw, Shoulder, NoGrip, FrontBlocked, PlatformPos, PlatformColor };

constexpr float kBackUpPickMm = 90.0f;     // user: ~9 cm
constexpr float kBackUpPlaceMm = 130.0f;   // user: 13 cm
constexpr uint8_t kGripTries = 3;          // closings before giving up on a cube

struct PickPlace {
  PpState state = PpState::CheckColor;
  PpFail fail = PpFail::None;
  uint8_t gripTries = 0;
  bool carrying = false;  // true from the first successful grip until the release

  static PickPlace pick() { return PickPlace{}; }
  static PickPlace place() { PickPlace p; p.state = PpState::CheckPlatformPos; p.carrying = true; return p; }

  bool running() const { return state != PpState::Done && state != PpState::Aborted; }
  bool done() const { return state == PpState::Done; }

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
        else state = PpState::RaiseRetry;
        break;
      case PpState::RaiseRetry: shoulder(io, Level::Up, PpState::CheckGripRaised); break;
      case PpState::CheckGripRaised:   // the cube may only show it is held once lifted
        if (io.holding()) { carrying = true; state = PpState::CheckClear; }
        else if (++gripTries < kGripTries) state = PpState::Reposition;
        else abort(io, PpFail::NoGrip);
        break;
      case PpState::Reposition: next(io.reposition(), PpState::OpenForPick, PpFail::Reposition); break;
      case PpState::RaiseCarry: shoulder(io, Level::Up, PpState::CheckClear); break;
      case PpState::CheckClear: next(io.frontClear(), PpState::Done, PpFail::FrontBlocked); break;
      case PpState::CheckPlatformPos: next(io.platformPositionOk(), PpState::CheckPlatformColor, PpFail::PlatformPos); break;
      case PpState::CheckPlatformColor: next(io.platformColorOk(), PpState::BackUpPlace, PpFail::PlatformColor); break;
      case PpState::BackUpPlace: next(io.backUp(kBackUpPlaceMm), PpState::LowerPlace, PpFail::BackUp); break;
      case PpState::LowerPlace: shoulder(io, Level::Place, PpState::Release); break;
      case PpState::Release:
        if (io.clawOpen()) { carrying = false; state = PpState::RaiseAfter; } else abort(io, PpFail::Claw);
        break;
      case PpState::RaiseAfter: shoulder(io, Level::Up, PpState::Done); break;
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
