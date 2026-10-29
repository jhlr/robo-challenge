#pragma once
#include <SCServo.h>

// Shoulder: STS3215 (params::kIdShoulder) in position mode on the shared bus. The only code that commands that servo.

bool shoulderBegin(SMS_STS& st);  // position mode (angle limit opened to 4095) + torque on; does NOT move. False if the servo does not answer.

// Turn the servo by `servoDeg` real degrees from its CURRENT position (+ = arm up, per params::kShoulderUpSign). Blocks until it arrives or ~3 s.
// Returns false and moves nothing if the delta is 0, above kShoulderMaxMoveDeg, outside the single-turn range, or the position can't be read.
// Arm travel -16..+25.5 deg is 4 x 41.5 = 166 servo deg (z48/z12).
bool shoulderRotate(float servoDeg);
