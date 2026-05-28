#ifndef OPENLOOP_MOTOR_TABLE_H
#define OPENLOOP_MOTOR_TABLE_H

#include <stdint.h>

// ============================================
// Tier-0 Open-Loop Motor Direction Table
// ============================================
//
// Fixed sign vectors for the six cardinal chassis directions, one row
// per direction. The row values are the PER-MOTOR SIGN (+1 / -1 / 0)
// in INTERNAL motor order [L1, R1, L2, R2] — the same order PID and
// FK use internally (permutation to wire order [L1, R1, R2, L2] is
// handled downstream in websocket_server's broadcast, not here).
//
// To execute a tier-0 segment, the executor multiplies the active
// row by a base PWM magnitude and writes directly to setMotorSpeed().
// PID and IK are deliberately bypassed — the point of tier 0 is to
// replicate the pre-ICIPRoB "RC controller + hardcoded direction
// table" baseline for the thesis comparison.
//
// Sign conventions come from the mecanum IK:
//   forward:  +vx          → all wheels forward
//   backward: -vx          → all wheels reverse
//   strafe-L: +vy          → inner wheels reverse, outer forward
//   strafe-R: -vy          → mirror of strafe-L
//   yaw-CCW:  +ω           → left wheels reverse, right forward
//   yaw-CW:   -ω           → mirror of yaw-CCW
//
// Any trajectory segment that doesn't match one of these six
// directions (e.g. diagonal strafe, strafe_circle) is rejected at
// load time — tier 0 deliberately cannot express arbitrary motion.

enum OpenLoopDirection : uint8_t {
    OL_DIR_FWD       = 0,  // +X body (forward)
    OL_DIR_BACK      = 1,  // -X body
    OL_DIR_STRAFE_L  = 2,  // +Y body (left)
    OL_DIR_STRAFE_R  = 3,  // -Y body
    OL_DIR_YAW_CCW   = 4,  // +ω (counter-clockwise)
    OL_DIR_YAW_CW    = 5,  // -ω
    OL_DIR_COUNT     = 6,
    OL_DIR_INVALID   = 0xFF,
};

// Per-direction motor sign vector in INTERNAL [L1, R1, L2, R2] order.
// Multiply each entry by the base PWM magnitude to get the motor
// command for that direction.
extern const int8_t OL_MOTOR_SIGNS[OL_DIR_COUNT][4];

// Human-readable name for logging / debugging.
const char* openloopDirectionName(OpenLoopDirection dir);

#endif // OPENLOOP_MOTOR_TABLE_H
