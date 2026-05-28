#include "mecanum.h"
#include "motors.h"
#include "config.h"

// ============================================
// Mecanum Wheel Kinematics
// ============================================
//
// Wheel layout (top view):
//
//        FRONT
//    L2 ╲    ╱ R2
//        ╲  ╱
//         ╲╱
//         ╱╲
//        ╱  ╲
//    L1 ╱    ╲ R1
//        REAR
//
// Standard mecanum configuration with rollers at 45°
//
// Inverse kinematics (body velocity to wheel speeds):
//   ω_L1 = (1/r) * (vx + vy - (Lx + Ly) * ω)  // rear-left
//   ω_R1 = (1/r) * (vx - vy + (Lx + Ly) * ω)  // rear-right
//   ω_R2 = (1/r) * (vx + vy + (Lx + Ly) * ω)  // front-right
//   ω_L2 = (1/r) * (vx - vy - (Lx + Ly) * ω)  // front-left

// Precomputed constants to avoid runtime division
static const float INV_R       = 1.0f / WHEEL_RADIUS;           // IK: 1/r
static const float FK_SCALE    = WHEEL_RADIUS / 4.0f;           // FK: r/4
static const float FK_ANG      = WHEEL_RADIUS / (4.0f * L_SUM); // FK: r/(4L)

WheelSpeeds mecanumInverseKinematics(float vx, float vy, float omega) {
    WheelSpeeds speeds;

    float Lw = L_SUM * omega;

    speeds.omega_L1 = INV_R * (vx + vy - Lw);  // Left rear  (RL)
    speeds.omega_R1 = INV_R * (vx - vy + Lw);  // Right rear (RR)
    speeds.omega_R2 = INV_R * (vx + vy + Lw);  // Right front (FR)
    speeds.omega_L2 = INV_R * (vx - vy - Lw);  // Left front  (FL)

    return speeds;
}

VelocityCommand mecanumForwardKinematics(float omega_L1, float omega_R1, float omega_R2, float omega_L2) {
    VelocityCommand cmd;

    // Forward kinematics — the algebraic inverse of the IK block above.
    //   vx = (r/4)     * ( ω_L1 + ω_R1 + ω_R2 + ω_L2)
    //   vy = (r/4)     * ( ω_L1 - ω_R1 + ω_R2 - ω_L2)   ← sign fixed 2026-04-15
    //   ω  = (r/(4·L)) * (-ω_L1 + ω_R1 + ω_R2 - ω_L2)
    float sum  = omega_L1 + omega_R1 + omega_R2 + omega_L2;
    float diff = omega_L1 - omega_R1 + omega_R2 - omega_L2;
    float rot  = -omega_L1 + omega_R1 + omega_R2 - omega_L2;

    cmd.vx    = FK_SCALE * sum;
    cmd.vy    = FK_SCALE * diff;
    cmd.omega = FK_ANG   * rot;

    return cmd;
}

int wheelSpeedToPWM(float omega, float maxWheelSpeed) {
    if (maxWheelSpeed <= 0) return 0;

    float ratio = omega / maxWheelSpeed;
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    return (int)(ratio * PWM_MAX_VALUE);
}
