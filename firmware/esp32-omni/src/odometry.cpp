#include "odometry.h"
#include "mecanum.h"
#include "config.h"
#include <math.h>

// ============================================
// Internal state
// ============================================

static Pose currentPose;
static portMUX_TYPE poseMux = portMUX_INITIALIZER_UNLOCKED;

// Complementary filter state
static float imuWeight = 0.98f;
static float imuYawOffset = 0.0f;
static bool  imuInitialized = false;

// Post-reset suppression (discard stale encoder deltas).
// Accessed from main loop (read) and WebSocket context (write via
// resetOdometry), so protected by poseMux.
static uint32_t suppressUntil = 0;

// ============================================
// Helpers
// ============================================

// NaN-safe angle normalization. Returns 0 if input is NaN/Inf to
// prevent infinite loops and downstream poison.
static inline float normalizeAngle(float a) {
    if (!isfinite(a)) return 0.0f;
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static inline float degToRad(float deg) {
    return deg * (M_PI / 180.0f);
}

// ============================================
// Public API
// ============================================

void initOdometry() {
    portENTER_CRITICAL(&poseMux);
    memset(&currentPose, 0, sizeof(currentPose));
    suppressUntil = 0;
    portEXIT_CRITICAL(&poseMux);
    imuWeight = 0.98f;
    imuYawOffset = 0.0f;
    imuInitialized = false;
}

void resetOdometry(float x, float y, float theta) {
    portENTER_CRITICAL(&poseMux);
    currentPose.x = x;
    currentPose.y = y;
    currentPose.theta = normalizeAngle(theta);
    currentPose.vx_body = 0;
    currentPose.vy_body = 0;
    currentPose.omega_body = 0;
    currentPose.timestamp = millis();
    suppressUntil = millis() + 300;
    portEXIT_CRITICAL(&poseMux);

    imuInitialized = false;
}

void odomUpdate(const float encoderVelocities[4], float dt,
                float imuYawDeg, float gyroZ) {
    // Read shared state in a single critical section: suppressUntil and
    // current heading. Consolidating avoids 3 separate lock pairs.
    uint32_t suppressEnd;
    float theta;
    portENTER_CRITICAL(&poseMux);
    suppressEnd = suppressUntil;
    theta = currentPose.theta;
    portEXIT_CRITICAL(&poseMux);

    // Skip during post-reset suppression window
    if (millis() < suppressEnd) return;

    // Encoder velocities are in internal order [L1=0, R1=1, L2=2, R2=3].
    // FK expects parameter order (L1, R1, R2, L2) — swap indices 2 and 3.
    VelocityCommand fk = mecanumForwardKinematics(
        encoderVelocities[0],   // L1
        encoderVelocities[1],   // R1
        encoderVelocities[3],   // R2 (internal index 3)
        encoderVelocities[2]    // L2 (internal index 2)
    );

    // Guard: if FK returns NaN (shouldn't happen, but belt-and-suspenders)
    if (!isfinite(fk.vx) || !isfinite(fk.vy) || !isfinite(fk.omega)) return;

    // Body-frame displacements
    float dx = fk.vx * dt;
    float dy = fk.vy * dt;
    float dtheta = fk.omega * dt;

    // Pure odometry heading update
    float odomTheta = normalizeAngle(theta + dtheta);
    float fusedTheta = odomTheta;

    // Complementary filter: fuse IMU yaw into heading
    bool imuValid = !isnan(imuYawDeg) && isfinite(imuYawDeg);
    if (imuValid && imuWeight > 0.0f) {
        float imuYawRad = degToRad(imuYawDeg);

        // Re-check after conversion (degToRad can't produce NaN from
        // finite input, but guard anyway)
        if (isfinite(imuYawRad)) {
            if (!imuInitialized) {
                // Capture offset so IMU frame aligns with odometry frame
                imuYawOffset = imuYawRad - odomTheta;
                imuInitialized = true;
            }

            float imuCorrected = normalizeAngle(imuYawRad - imuYawOffset);
            float angleDiff = normalizeAngle(imuCorrected - odomTheta);
            fusedTheta = normalizeAngle(odomTheta + imuWeight * angleDiff);
        }
    }

    // Body-to-world transform using midpoint heading.
    // Use normalizeAngle on the delta to handle the ±π boundary correctly:
    // e.g., theta=3.1 and fusedTheta=-3.1 are ~0 apart, not ~6.2 apart.
    float headingDelta = normalizeAngle(fusedTheta - theta);
    float mid = theta + headingDelta * 0.5f;
    float cosM = cosf(mid);
    float sinM = sinf(mid);
    float worldDx = dx * cosM - dy * sinM;
    float worldDy = dx * sinM + dy * cosM;

    // Accumulate pose under spinlock
    portENTER_CRITICAL(&poseMux);
    currentPose.x += worldDx;
    currentPose.y += worldDy;
    currentPose.theta = fusedTheta;
    currentPose.vx_body = fk.vx;
    currentPose.vy_body = fk.vy;
    currentPose.omega_body = fk.omega;
    currentPose.timestamp = millis();
    portEXIT_CRITICAL(&poseMux);
}

Pose odomGetPose() {
    Pose copy;
    portENTER_CRITICAL(&poseMux);
    copy = currentPose;
    portEXIT_CRITICAL(&poseMux);
    return copy;
}

void odomSetImuWeight(float weight) {
    if (weight < 0.0f) weight = 0.0f;
    if (weight > 1.0f) weight = 1.0f;
    imuWeight = weight;
}

float odomGetImuWeight() {
    return imuWeight;
}
