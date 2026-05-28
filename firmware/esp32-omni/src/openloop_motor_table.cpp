#include "openloop_motor_table.h"

// Motor order: [L1 (rear-left), R1 (rear-right), L2 (front-left), R2 (front-right)]
// Each row: sign vector to apply to the base PWM for that direction.
// Derived from the mecanum IK with unit inputs — see header for the
// sign-convention notes. Treated as an empirical lookup table at
// runtime; no kinematic math happens on this path.
const int8_t OL_MOTOR_SIGNS[OL_DIR_COUNT][4] = {
    // L1  R1  L2  R2
    {  +1, +1, +1, +1 },  // OL_DIR_FWD
    {  -1, -1, -1, -1 },  // OL_DIR_BACK
    {  +1, -1, -1, +1 },  // OL_DIR_STRAFE_L (+Y)
    {  -1, +1, +1, -1 },  // OL_DIR_STRAFE_R (-Y)
    {  -1, +1, -1, +1 },  // OL_DIR_YAW_CCW (+ω)
    {  +1, -1, +1, -1 },  // OL_DIR_YAW_CW  (-ω)
};

const char* openloopDirectionName(OpenLoopDirection dir) {
    switch (dir) {
        case OL_DIR_FWD:      return "fwd";
        case OL_DIR_BACK:     return "back";
        case OL_DIR_STRAFE_L: return "strafe_l";
        case OL_DIR_STRAFE_R: return "strafe_r";
        case OL_DIR_YAW_CCW:  return "yaw_ccw";
        case OL_DIR_YAW_CW:   return "yaw_cw";
        default:              return "invalid";
    }
}
