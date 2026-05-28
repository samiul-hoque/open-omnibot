#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

// ============================================
// Pose Structure
// ============================================

struct Pose {
    float x;           // meters, world frame
    float y;           // meters, world frame
    float theta;       // radians, world frame, CCW positive
    float vx_body;     // body-frame forward velocity (m/s), latest FK
    float vy_body;     // body-frame lateral velocity (m/s), latest FK
    float omega_body;  // body-frame angular velocity (rad/s), latest FK
    uint32_t timestamp;
};

// ============================================
// Function Declarations
// ============================================

// Initialize odometry state (zeros everything)
void initOdometry();

// Reset pose. Sets 300ms suppression window to discard stale encoder deltas.
void resetOdometry(float x = 0, float y = 0, float theta = 0);

// Integrate one odometry step from gain-corrected encoder velocities + IMU.
//
// encoderVelocities: rad/s in *internal* firmware order [L1, R1, L2, R2].
//                    This function handles the index swap for FK internally.
// dt:                time step in seconds (nominally 0.020 at 50 Hz)
// imuYawDeg:         BNO055 yaw in degrees (pass NAN if IMU unavailable)
// gyroZ:             gyro Z-axis angular velocity in rad/s (informational)
void odomUpdate(const float encoderVelocities[4], float dt,
                float imuYawDeg, float gyroZ);

// Thread-safe pose getter (copies under spinlock for cross-core reads)
Pose odomGetPose();

// Complementary filter IMU weight: 0.0 = pure odometry, 1.0 = pure IMU
void odomSetImuWeight(float weight);
float odomGetImuWeight();

#endif // ODOMETRY_H
