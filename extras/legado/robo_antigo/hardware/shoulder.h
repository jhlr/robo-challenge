#pragma once
#include <SCServo.h>

// Shoulder: STS3215 (params::kIdShoulder) in position mode on the shared bus, direct drive. The only code that commands that servo.
// Every move is a series of small steps (params::kShoulderChunkTicks) from the CURRENT position, each one checked (position follows, current below the limit).

enum class ShoulderLevel : uint8_t { Up, Place, Pick };  // heights in params::kShoulderTicks*

bool shoulderBegin(SMS_STS& st);  // position mode (angle limit opened to 4095) + torque on; does NOT move. False if the servo does not answer.

// Move to a calibrated height. True only when it ARRIVED. False: nothing more was commanded after the first failed check (position unreadable, farther than kShoulderMaxMoveDeg, not following,
// or current above the limit); the servo is told to hold where it stopped. Re-read the position before trusting the arm.
bool shoulderGoTo(ShoulderLevel level);

// Turn by `deg` arm degrees from the CURRENT position (+ = up per params::kShoulderUpSign). Same step-by-step checks; false also when the delta is 0, above the limit or out of range.
bool shoulderRotate(float deg);

int shoulderTicks();  // current position, -1 when unreadable
