#ifndef MECANUM_H
#define MECANUM_H

#include <Arduino.h>

// ============================================
// Velocity Command Structure
// ============================================

struct VelocityCommand {
    float vx;      // Forward velocity (m/s), positive = forward
    float vy;      // Lateral velocity (m/s), positive = left
    float omega;   // Angular velocity (rad/s), positive = counter-clockwise
};

// ============================================
// Wheel Speeds Structure
// ============================================

struct WheelSpeeds {
    float omega_L1;  // Left rear wheel (rad/s)
    float omega_R1;  // Right rear wheel (rad/s)
    float omega_R2;  // Right front wheel (rad/s)
    float omega_L2;  // Left front wheel (rad/s)
};

// ============================================
// Function Declarations
// ============================================

// Inverse kinematics: body velocity -> wheel speeds (rad/s)
WheelSpeeds mecanumInverseKinematics(float vx, float vy, float omega);

// Forward kinematics: wheel speeds (rad/s) -> body velocity
VelocityCommand mecanumForwardKinematics(float omega_L1, float omega_R1, float omega_R2, float omega_L2);

// Convert wheel speed (rad/s) to PWM value (-255 to 255)
// maxWheelSpeed: maximum wheel angular velocity in rad/s at PWM 255
int wheelSpeedToPWM(float omega, float maxWheelSpeed);

#endif // MECANUM_H
