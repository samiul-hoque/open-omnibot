#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

// ============================================
// PID Configuration
// ============================================

#define PID_KP_DEFAULT     20.0f
#define PID_KI_DEFAULT     25.0f
#define PID_KD_DEFAULT     0.05f
#define PID_INTEGRAL_MAX   200.0f   // Max contribution of integral term (PWM units)

// ============================================
// PID State Structure
// ============================================

struct PIDState {
    float kp;
    float ki;
    float kd;
    float integral;
    float prevError;
};

// ============================================
// PID Diagnostics (for debug broadcast)
// ============================================

struct PIDDiag {
    float target;       // Target wheel speed (rad/s)
    float actual;       // Measured wheel speed (rad/s)
    float error;        // target - actual
    float p_term;       // Proportional contribution
    float i_term;       // Integral contribution
    float d_term;       // Derivative contribution
    float feedforward;  // Feedforward PWM
    int   pwm_out;      // Final PWM output
};

// ============================================
// Function Declarations
// ============================================

// Initialize PID controllers for all 4 wheels
void initPIDControllers();

// Run one closed-loop update cycle: IK → encoder feedback → feedforward+PID → motors
// Call at 50Hz from the motor update loop
void applyClosedLoopVelocity(float vx, float vy, float omega);

// Reset all PID states (call on stop/timeout)
void resetPIDControllers();

// Get last PID diagnostics for all 4 wheels (for debug broadcast)
void getPIDDiagnostics(PIDDiag diag[4]);

// Enable/disable diagnostic data collection (guards writes on hot path)
void setPIDDiagEnabled(bool enabled);

#endif // PID_CONTROLLER_H
