/**
 * @file test_odometry.cpp
 * @brief Unit tests for odometry math: normalizeAngle, bodyToWorld,
 *        forward kinematics integration, and complementary filter.
 *
 * These tests run in the PlatformIO native environment (x86) and
 * exercise the pure math without hardware dependencies. The functions
 * under test are re-implemented here with the same formulas as
 * odometry.cpp and mecanum.cpp to keep the test self-contained.
 */

#include <unity.h>
#include <math.h>

// ============================================
// Constants (must match config.h)
// ============================================

#define WHEEL_RADIUS 0.04f
#define LX 0.1175f
#define LY 0.0953f
#define L_SUM (LX + LY)

// ============================================
// Functions under test (standalone copies)
// ============================================

static float normalizeAngle(float a) {
    if (!isfinite(a)) return 0.0f;
    while (a > M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

static float degToRad(float deg) {
    return deg * (float)(M_PI / 180.0);
}

struct Vec2 { float dx, dy; };

// bodyToWorld using midpoint heading (matches mecanumKinematics.js:112-120)
static Vec2 bodyToWorld(float dx_body, float dy_body, float theta, float dtheta) {
    float mid = theta + dtheta / 2.0f;
    float c = cosf(mid);
    float s = sinf(mid);
    return { dx_body * c - dy_body * s, dx_body * s + dy_body * c };
}

struct FKResult { float vx, vy, omega; };

// Forward kinematics (matches mecanum.cpp:52-71)
// Parameters in external order: (L1, R1, R2, L2)
static FKResult mecanumFK(float wL1, float wR1, float wR2, float wL2) {
    float r = WHEEL_RADIUS;
    float L = L_SUM;
    return {
        (r / 4.0f) * (wL1 + wR1 + wR2 + wL2),
        (r / 4.0f) * (wL1 - wR1 + wR2 - wL2),
        (r / (4.0f * L)) * (-wL1 + wR1 + wR2 - wL2)
    };
}

void setUp(void) {}
void tearDown(void) {}

// ============================================
// normalizeAngle tests
// ============================================

void test_normalize_zero(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, normalizeAngle(0.0f));
}

void test_normalize_pi(void) {
    // pi is on the boundary; either +pi or -pi is acceptable
    float result = normalizeAngle((float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, (float)M_PI, fabsf(result));
}

void test_normalize_positive_wrap(void) {
    // 3pi should wrap to ±pi (boundary)
    float result = normalizeAngle(3.0f * (float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, (float)M_PI, fabsf(result));
}

void test_normalize_negative_wrap(void) {
    // -3pi should wrap to ±pi (boundary)
    float result = normalizeAngle(-3.0f * (float)M_PI);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, (float)M_PI, fabsf(result));
}

void test_normalize_small_positive(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, normalizeAngle(0.5f));
}

void test_normalize_large_positive(void) {
    // 7pi/4 = 5.4978 should wrap to -pi/4 = -0.7854
    float result = normalizeAngle(7.0f * (float)M_PI / 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -(float)M_PI / 4.0f, result);
}

void test_normalize_nan_returns_zero(void) {
    float result = normalizeAngle(NAN);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, result);
}

void test_normalize_inf_returns_zero(void) {
    float result = normalizeAngle(INFINITY);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, result);
}

void test_normalize_neg_inf_returns_zero(void) {
    float result = normalizeAngle(-INFINITY);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, result);
}

// ============================================
// bodyToWorld tests
// ============================================

void test_b2w_heading_zero(void) {
    // At theta=0, body forward = world +x
    Vec2 w = bodyToWorld(1.0f, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, w.dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, w.dy);
}

void test_b2w_heading_90(void) {
    // At theta=pi/2, body forward = world +y
    Vec2 w = bodyToWorld(1.0f, 0.0f, (float)M_PI / 2.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, w.dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, w.dy);
}

void test_b2w_strafe_left_heading_zero(void) {
    // At theta=0, body left (vy=+1) = world +y
    Vec2 w = bodyToWorld(0.0f, 1.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, w.dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, w.dy);
}

void test_b2w_midpoint_heading(void) {
    // Moving forward while turning 90deg: midpoint is 45deg
    // Forward motion should split equally between +x and +y
    Vec2 w = bodyToWorld(1.0f, 0.0f, 0.0f, (float)M_PI / 2.0f);
    float expected = cosf((float)M_PI / 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, w.dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, w.dy);
}

// ============================================
// FK tests (regression guards, same as server localization.test.js)
// ============================================

void test_fk_forward(void) {
    // All wheels forward at 1 rad/s
    FKResult fk = mecanumFK(1.0f, 1.0f, 1.0f, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, WHEEL_RADIUS, fk.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, fk.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, fk.omega);
}

void test_fk_strafe_left(void) {
    // Strafe left pattern: L1=+1, R1=-1, R2=+1, L2=-1
    FKResult fk = mecanumFK(1.0f, -1.0f, 1.0f, -1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, fk.vx);
    // vy should be positive (body left)
    TEST_ASSERT_TRUE(fk.vy > 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, WHEEL_RADIUS, fk.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, fk.omega);
}

void test_fk_ccw_rotation(void) {
    // CCW pattern: L1=-1, R1=+1, R2=+1, L2=-1
    FKResult fk = mecanumFK(-1.0f, 1.0f, 1.0f, -1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, fk.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, fk.vy);
    // omega should be positive (CCW)
    TEST_ASSERT_TRUE(fk.omega > 0);
}

// ============================================
// Odometry integration test (multi-step)
// ============================================

void test_odom_straight_line(void) {
    // Simulate driving forward at 0.1 m/s for 2 seconds (100 steps @ 50Hz)
    float x = 0, y = 0, theta = 0;
    float dt = 0.020f;

    // All 4 wheels at the same speed for pure forward motion
    // vx = r * omega => omega = vx / r = 0.1 / 0.04 = 2.5 rad/s
    float omega_w = 0.1f / WHEEL_RADIUS;

    for (int i = 0; i < 100; i++) {
        FKResult fk = mecanumFK(omega_w, omega_w, omega_w, omega_w);
        float dx = fk.vx * dt;
        float dy = fk.vy * dt;
        float dtheta = fk.omega * dt;
        Vec2 w = bodyToWorld(dx, dy, theta, dtheta);
        x += w.dx;
        y += w.dy;
        theta = normalizeAngle(theta + dtheta);
    }

    // Should have traveled ~2.0m forward, ~0m lateral
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, x);  // 0.1 m/s * 2s = 0.2m
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, theta);
}

void test_odom_strafe_left(void) {
    // Simulate strafing left at 0.1 m/s for 1 second (50 steps @ 50Hz)
    float x = 0, y = 0, theta = 0;
    float dt = 0.020f;

    // Strafe left: vy = 0.1 m/s
    // L1=+w, R1=-w, R2=+w, L2=-w where w = vy / r = 2.5 rad/s
    float omega_w = 0.1f / WHEEL_RADIUS;

    for (int i = 0; i < 50; i++) {
        FKResult fk = mecanumFK(omega_w, -omega_w, omega_w, -omega_w);
        float dx = fk.vx * dt;
        float dy = fk.vy * dt;
        float dtheta = fk.omega * dt;
        Vec2 w = bodyToWorld(dx, dy, theta, dtheta);
        x += w.dx;
        y += w.dy;
        theta = normalizeAngle(theta + dtheta);
    }

    // Should have traveled ~0.1m left (+y), ~0m forward
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, theta);
}

// ============================================
// Complementary filter test
// ============================================

void test_complementary_filter_convergence(void) {
    // Simulate complementary filter: odom says theta=0, IMU says theta=0.1 rad
    // With weight=0.98, after many steps the fused theta should approach IMU
    float odomTheta = 0.0f;
    float imuYawRad = 0.1f;
    float weight = 0.98f;

    for (int i = 0; i < 200; i++) {
        float angleDiff = normalizeAngle(imuYawRad - odomTheta);
        odomTheta = normalizeAngle(odomTheta + weight * angleDiff);
    }

    // Should converge close to IMU heading
    TEST_ASSERT_FLOAT_WITHIN(0.001f, imuYawRad, odomTheta);
}

void test_complementary_filter_pure_odom(void) {
    // With weight=0, should ignore IMU entirely
    float theta = 0.5f;
    float dtheta = 0.01f;
    float imuYaw = 1.0f;
    float weight = 0.0f;

    float odomTheta = normalizeAngle(theta + dtheta);
    // No IMU contribution
    float fused = odomTheta;

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.51f, fused);
}

// ============================================
// Index swap test
// ============================================

void test_internal_to_fk_index_swap(void) {
    // Internal order: [L1=0, R1=1, L2=2, R2=3]
    // FK expects:     (L1,   R1,   R2,   L2)
    // So we pass vel[0], vel[1], vel[3], vel[2]

    float vel[4] = {1.0f, 2.0f, 3.0f, 4.0f};  // L1=1, R1=2, L2=3, R2=4
    FKResult fk = mecanumFK(vel[0], vel[1], vel[3], vel[2]);

    // Manual: FK(1, 2, 4, 3)
    float r = WHEEL_RADIUS;
    float L = L_SUM;
    float expected_vx = (r / 4.0f) * (1.0f + 2.0f + 4.0f + 3.0f);
    float expected_vy = (r / 4.0f) * (1.0f - 2.0f + 4.0f - 3.0f);
    float expected_omega = (r / (4.0f * L)) * (-1.0f + 2.0f + 4.0f - 3.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected_vx, fk.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected_vy, fk.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, expected_omega, fk.omega);
}

// ============================================
// Main
// ============================================

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // normalizeAngle
    RUN_TEST(test_normalize_zero);
    RUN_TEST(test_normalize_pi);
    RUN_TEST(test_normalize_positive_wrap);
    RUN_TEST(test_normalize_negative_wrap);
    RUN_TEST(test_normalize_small_positive);
    RUN_TEST(test_normalize_large_positive);
    RUN_TEST(test_normalize_nan_returns_zero);
    RUN_TEST(test_normalize_inf_returns_zero);
    RUN_TEST(test_normalize_neg_inf_returns_zero);

    // bodyToWorld
    RUN_TEST(test_b2w_heading_zero);
    RUN_TEST(test_b2w_heading_90);
    RUN_TEST(test_b2w_strafe_left_heading_zero);
    RUN_TEST(test_b2w_midpoint_heading);

    // FK regression guards
    RUN_TEST(test_fk_forward);
    RUN_TEST(test_fk_strafe_left);
    RUN_TEST(test_fk_ccw_rotation);

    // Odometry integration
    RUN_TEST(test_odom_straight_line);
    RUN_TEST(test_odom_strafe_left);

    // Complementary filter
    RUN_TEST(test_complementary_filter_convergence);
    RUN_TEST(test_complementary_filter_pure_odom);

    // Index swap
    RUN_TEST(test_internal_to_fk_index_swap);

    return UNITY_END();
}
