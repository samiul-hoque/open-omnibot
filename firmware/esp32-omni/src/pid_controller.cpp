#include "pid_controller.h"
#include "config.h"
#include "mecanum.h"
#include "motors.h"
#include "sensors.h"

// ============================================
// Private Variables
// ============================================

#define NUM_WHEELS 4

static PIDState pidStates[NUM_WHEELS];

// Independent velocity tracking (separate from readEncoders broadcast)
static int32_t pidLastCounts[NUM_WHEELS] = {0, 0, 0, 0};
static uint32_t pidLastTime = 0;

// Must match MAX_WHEEL_SPEED in main.cpp. See that file for the 2026-04-18
// re-measurement from a stand hold-test (12.0 → 22.0 to align FF with the
// real motor curve at typical operating speeds).
static const float MAX_WHEEL_SPEED = 22.0f;

// Diagnostics (filled by applyClosedLoopVelocity when enabled, read by getPIDDiagnostics)
static PIDDiag lastDiag[NUM_WHEELS] = {};
static bool pidDiagEnabled = false;

// Motor direction signs — needed to convert encoder velocity to IK frame
// Physical order: [L1, R1, L2, R2] at indices [0, 1, 2, 3]
static const int8_t motorDirs[NUM_WHEELS] = {MOTOR_L1_DIR, MOTOR_R1_DIR, MOTOR_L2_DIR, MOTOR_R2_DIR};

// ============================================
// Private Functions
// ============================================

static float computePID(PIDState* state, float error, float dt) {
    // Proportional
    float p = state->kp * error;

    // Integral with anti-windup
    state->integral += error * dt;
    float integralLimit = PID_INTEGRAL_MAX / state->ki;
    if (state->integral > integralLimit) state->integral = integralLimit;
    if (state->integral < -integralLimit) state->integral = -integralLimit;
    float i = state->ki * state->integral;

    // Derivative
    float d = state->kd * (error - state->prevError) / dt;
    state->prevError = error;

    return p + i + d;
}

// ============================================
// Public Functions
// ============================================

void initPIDControllers() {
    for (int i = 0; i < NUM_WHEELS; i++) {
        pidStates[i].kp = PID_KP_DEFAULT;
        pidStates[i].ki = PID_KI_DEFAULT;
        pidStates[i].kd = PID_KD_DEFAULT;
        pidStates[i].integral = 0;
        pidStates[i].prevError = 0;
        pidLastCounts[i] = getEncoderCount(i);
    }
    pidLastTime = millis();
}

void applyClosedLoopVelocity(float vx, float vy, float omega) {
    // Body-frame command: vx = forward (+X), vy = left (+Y), omega = CCW.
    // Per-motor MOTOR_*_DIR / ENC_*_DIR in config.h bring each wheel's sign
    // into agreement with this convention, so IK is applied directly.

    // Zero target: stop motors but preserve integral (learned motor bias)
    // Integral only resets on timeout via resetPIDControllers()
    if (vx == 0.0f && vy == 0.0f && omega == 0.0f) {
        // Keep encoder tracking fresh to avoid stale delta on next command
        for (int i = 0; i < NUM_WHEELS; i++) {
            pidLastCounts[i] = getEncoderCount(i);
        }
        pidLastTime = millis();
        setWheelSpeeds(0, 0, 0, 0);
        return;
    }

    // Step 1: Compute target wheel speeds via IK
    WheelSpeeds targets = mecanumInverseKinematics(vx, vy, omega);

    // Scale if any wheel exceeds max speed
    float maxReq = max(max(fabsf(targets.omega_L1), fabsf(targets.omega_R1)),
                       max(fabsf(targets.omega_R2), fabsf(targets.omega_L2)));
    float scale = 1.0f;
    if (maxReq > MAX_WHEEL_SPEED && maxReq > 0) {
        scale = MAX_WHEEL_SPEED / maxReq;
    }

    // Target array in physical order: [L1, R1, L2, R2]
    float target[NUM_WHEELS] = {
        targets.omega_L1 * scale,
        targets.omega_R1 * scale,
        targets.omega_L2 * scale,  // index 2 = L2
        targets.omega_R2 * scale   // index 3 = R2
    };

    // Step 2: Measure actual wheel velocities from encoder deltas
    uint32_t now = millis();
    uint32_t dt_ms = now - pidLastTime;
    if (dt_ms == 0) dt_ms = 1;
    float dt = dt_ms / 1000.0f;

    float actual[NUM_WHEELS];
    int32_t counts[NUM_WHEELS];
    float gainsFwd[NUM_WHEELS], gainsRev[NUM_WHEELS];
    getMotorGains(gainsFwd, gainsRev);
    for (int i = 0; i < NUM_WHEELS; i++) {
        counts[i] = getEncoderCount(i);
        // Measurement is the TRUE wheel velocity (no gain applied). Gains
        // compensate motor PWM→speed variance at feedforward below; the
        // PID loop closes on true rad/s so the integrator converges to
        // target for every wheel regardless of motor strength.
        int32_t delta = (counts[i] - pidLastCounts[i]) * (-motorDirs[i]);
        float revs = (float)delta / (float)COUNTS_PER_WHEEL_REV;
        actual[i] = (revs * 2.0f * 3.14159265f) / dt;
    }

    // Step 3: Feedforward + PID correction per wheel
    int pwm[NUM_WHEELS];
    for (int i = 0; i < NUM_WHEELS; i++) {
        // Per-motor gain compensates PWM→speed variance at feedforward.
        // The cal measures mean_counts / motor_counts at fixed PWM, so a
        // slow motor gets gain > 1 → the scaled target yields more FF
        // PWM → motor reaches target with minimal PID correction. Direction
        // is chosen from the target sign so we use the relevant curve.
        float g_ff = (target[i] >= 0) ? gainsFwd[i] : gainsRev[i];
        float ff = (float)wheelSpeedToPWM(target[i] * g_ff, MAX_WHEEL_SPEED);
        float error = target[i] - actual[i];
        float correction = computePID(&pidStates[i], error, dt);

        float total = ff + correction;
        if (total > 255.0f) total = 255.0f;
        if (total < -255.0f) total = -255.0f;
        pwm[i] = (int)total;

        // Store diagnostics only when debug mode is active (avoids
        // unnecessary writes on the hot path when nobody is reading)
        if (pidDiagEnabled) {
            lastDiag[i].target = target[i];
            lastDiag[i].actual = actual[i];
            lastDiag[i].error = error;
            lastDiag[i].p_term = pidStates[i].kp * error;
            lastDiag[i].i_term = pidStates[i].ki * pidStates[i].integral;
            lastDiag[i].d_term = correction - lastDiag[i].p_term - lastDiag[i].i_term;
            lastDiag[i].feedforward = ff;
            lastDiag[i].pwm_out = pwm[i];
        }
    }

    // Step 4: Update state
    for (int i = 0; i < NUM_WHEELS; i++) {
        pidLastCounts[i] = counts[i];
    }
    pidLastTime = now;

    // Step 5: Apply to motors
    // pwm array is [L1, R1, L2, R2], setWheelSpeeds expects (L1, R1, R2, L2)
    setWheelSpeeds(pwm[0], pwm[1], pwm[3], pwm[2]);
}

void resetPIDControllers() {
    for (int i = 0; i < NUM_WHEELS; i++) {
        pidStates[i].integral = 0;
        pidStates[i].prevError = 0;
        pidLastCounts[i] = getEncoderCount(i);
    }
    pidLastTime = millis();
}

void getPIDDiagnostics(PIDDiag diag[4]) {
    memcpy(diag, lastDiag, sizeof(lastDiag));
}

void setPIDDiagEnabled(bool enabled) {
    pidDiagEnabled = enabled;
}
