/**
 * @file test_openloop.cpp
 * @brief Native unit tests for tier-0 open-loop segment classification +
 *        duration/distance math. Follows the same pattern as
 *        test_trajectory: reproduces the pure-logic subset locally so
 *        the native env doesn't drag in Arduino / NVS / WebSocket deps.
 */

#include <unity.h>
#include <math.h>
#include <stdint.h>

// ============================================
// Reproduced types and constants
// ============================================

enum SegmentKind : uint8_t {
    SEG_TRANSLATE = 0,
    SEG_YAW = 1,
    SEG_STRAFE_CIRCLE = 2,
    SEG_PAUSE = 3,
};

struct TrajectorySegment {
    SegmentKind kind;
    float durationMs;
    union {
        struct { float vx; float vy; float distance; } translate;
        struct { float w; float angle; } yaw;
        struct { float speed; float radius; } circle;
    };
};

enum OpenLoopDirection : uint8_t {
    OL_DIR_FWD = 0,
    OL_DIR_BACK = 1,
    OL_DIR_STRAFE_L = 2,
    OL_DIR_STRAFE_R = 3,
    OL_DIR_YAW_CCW = 4,
    OL_DIR_YAW_CW = 5,
    OL_DIR_COUNT = 6,
    OL_DIR_INVALID = 0xFF,
};

// Mirror of the table in openloop_motor_table.cpp. Keep in sync if
// the firmware copy is edited.
static const int8_t OL_MOTOR_SIGNS[OL_DIR_COUNT][4] = {
    {  +1, +1, +1, +1 },  // FWD
    {  -1, -1, -1, -1 },  // BACK
    {  +1, -1, -1, +1 },  // STRAFE_L
    {  -1, +1, +1, -1 },  // STRAFE_R
    {  -1, +1, -1, +1 },  // YAW_CCW
    {  +1, -1, +1, -1 },  // YAW_CW
};

// Physical constants from config.h. Duplicated here so the native
// test doesn't need Arduino headers.
static constexpr float WHEEL_RADIUS = 0.04f;
static constexpr float LX = 0.1175f;
static constexpr float LY = 0.0953f;
static constexpr float L_SUM = LX + LY;
static constexpr int COUNTS_PER_WHEEL_REV = 1092;
static constexpr float WHEEL_CIRCUMFERENCE = 2.0f * 3.14159265f * WHEEL_RADIUS;
static constexpr float METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;

// ============================================
// Port of openloopClassifySegment — behavior
// MUST stay byte-identical to the firmware copy.
// ============================================

static constexpr float OL_CARDINAL_EPS = 1e-3f;

static OpenLoopDirection classify(const TrajectorySegment& seg) {
    switch (seg.kind) {
        case SEG_TRANSLATE: {
            const float vx = seg.translate.vx;
            const float vy = seg.translate.vy;
            const bool vxNonzero = fabsf(vx) > OL_CARDINAL_EPS;
            const bool vyNonzero = fabsf(vy) > OL_CARDINAL_EPS;
            if (vxNonzero && vyNonzero) return OL_DIR_INVALID;
            if (vxNonzero) return (vx > 0) ? OL_DIR_FWD : OL_DIR_BACK;
            if (vyNonzero) return (vy > 0) ? OL_DIR_STRAFE_L : OL_DIR_STRAFE_R;
            return OL_DIR_INVALID;
        }
        case SEG_YAW: {
            const float angle = seg.yaw.angle;
            if (fabsf(angle) < OL_CARDINAL_EPS) return OL_DIR_INVALID;
            return (angle > 0) ? OL_DIR_YAW_CCW : OL_DIR_YAW_CW;
        }
        case SEG_STRAFE_CIRCLE:
        case SEG_PAUSE:
        default:
            return OL_DIR_INVALID;
    }
}

// ============================================
// Port of openloopSegmentExpectedCounts
// ============================================

static uint32_t expectedCounts(const TrajectorySegment& seg) {
    if (seg.kind == SEG_PAUSE) return 0;
    const float countsPerMeter = 1.0f / METERS_PER_COUNT;
    if (seg.kind == SEG_TRANSLATE) {
        return (uint32_t)(4.0f * seg.translate.distance * countsPerMeter);
    }
    if (seg.kind == SEG_YAW) {
        const float perWheelMeters = fabsf(seg.yaw.angle) * L_SUM;
        return (uint32_t)(4.0f * perWheelMeters * countsPerMeter);
    }
    return 0;
}

// ============================================
// Tests — classifier
// ============================================

static TrajectorySegment mkTranslate(float vx, float vy, float d) {
    TrajectorySegment s{}; s.kind = SEG_TRANSLATE;
    s.translate.vx = vx; s.translate.vy = vy; s.translate.distance = d;
    return s;
}
static TrajectorySegment mkYaw(float w, float a) {
    TrajectorySegment s{}; s.kind = SEG_YAW;
    s.yaw.w = w; s.yaw.angle = a;
    return s;
}
static TrajectorySegment mkCircle(float speed, float radius) {
    TrajectorySegment s{}; s.kind = SEG_STRAFE_CIRCLE;
    s.circle.speed = speed; s.circle.radius = radius;
    return s;
}
static TrajectorySegment mkPause() {
    TrajectorySegment s{}; s.kind = SEG_PAUSE;
    return s;
}

void test_classify_translate_cardinal() {
    TEST_ASSERT_EQUAL(OL_DIR_FWD,      classify(mkTranslate( 0.2f,  0.0f, 1.0f)));
    TEST_ASSERT_EQUAL(OL_DIR_BACK,     classify(mkTranslate(-0.2f,  0.0f, 1.0f)));
    TEST_ASSERT_EQUAL(OL_DIR_STRAFE_L, classify(mkTranslate( 0.0f,  0.2f, 1.0f)));
    TEST_ASSERT_EQUAL(OL_DIR_STRAFE_R, classify(mkTranslate( 0.0f, -0.2f, 1.0f)));
}

void test_classify_translate_diagonal_rejected() {
    // Diagonal (vx and vy both non-zero) is unsupported in tier 0.
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkTranslate( 0.2f,  0.2f, 1.0f)));
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkTranslate(-0.2f,  0.2f, 1.0f)));
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkTranslate( 0.2f, -0.2f, 1.0f)));
}

void test_classify_translate_zero_rejected() {
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkTranslate(0, 0, 1.0f)));
}

void test_classify_translate_near_zero_treated_as_zero() {
    // A commanded translate at below-epsilon velocity is classified as
    // zero — prevents float-round fuzz from silently picking a cardinal.
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkTranslate(1e-4f, 0, 1.0f)));
}

void test_classify_yaw() {
    TEST_ASSERT_EQUAL(OL_DIR_YAW_CCW, classify(mkYaw(0.5f,  M_PI / 2.0f)));
    TEST_ASSERT_EQUAL(OL_DIR_YAW_CW,  classify(mkYaw(0.5f, -M_PI / 2.0f)));
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkYaw(0.5f,  0.0f)));
}

void test_classify_strafe_circle_rejected() {
    // Tier 0 cannot decompose continuously rotating velocity vectors.
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkCircle(0.4f, 0.5f)));
}

void test_classify_pause_is_invalid() {
    // Pauses are handled separately — classifier says INVALID so a
    // caller doesn't accidentally pick a direction for zero motion.
    TEST_ASSERT_EQUAL(OL_DIR_INVALID, classify(mkPause()));
}

// ============================================
// Tests — expected-counts math
// ============================================

void test_expected_counts_translate() {
    // 1 m translation: each wheel turns 1 m / wheel circumference revolutions.
    // 4 × counts_per_m ≈ 4 / 0.000230 ≈ 17368
    const uint32_t expected = (uint32_t)(4.0f * 1.0f / METERS_PER_COUNT);
    TEST_ASSERT_EQUAL_UINT32(expected, expectedCounts(mkTranslate(0.2f, 0, 1.0f)));
    // Scales linearly with distance.
    const uint32_t expectedHalf = (uint32_t)(4.0f * 0.5f / METERS_PER_COUNT);
    TEST_ASSERT_EQUAL_UINT32(expectedHalf, expectedCounts(mkTranslate(0.2f, 0, 0.5f)));
    // Distance, not speed, drives the expected count — same direction
    // at different speeds yields same count.
    TEST_ASSERT_EQUAL_UINT32(expected, expectedCounts(mkTranslate(0.4f, 0, 1.0f)));
}

void test_expected_counts_yaw_ninety_degrees() {
    // 90° yaw: each wheel traces an arc of L_SUM meters × π/2 rad.
    const float arc = (M_PI / 2.0f) * L_SUM;
    const uint32_t expected = (uint32_t)(4.0f * arc / METERS_PER_COUNT);
    TEST_ASSERT_EQUAL_UINT32(expected, expectedCounts(mkYaw(0.5f,  M_PI / 2.0f)));
    // Sign of angle doesn't matter for count magnitude.
    TEST_ASSERT_EQUAL_UINT32(expected, expectedCounts(mkYaw(0.5f, -M_PI / 2.0f)));
}

void test_expected_counts_pause_is_zero() {
    TEST_ASSERT_EQUAL_UINT32(0, expectedCounts(mkPause()));
}

// ============================================
// Tests — motor sign table
// ============================================

void test_motor_table_forward() {
    // Forward: all four wheels drive positive.
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL(+1, OL_MOTOR_SIGNS[OL_DIR_FWD][i]);
    }
}

void test_motor_table_backward_mirrors_forward() {
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL(-OL_MOTOR_SIGNS[OL_DIR_FWD][i], OL_MOTOR_SIGNS[OL_DIR_BACK][i]);
    }
}

void test_motor_table_strafe_left_mirrors_strafe_right() {
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL(-OL_MOTOR_SIGNS[OL_DIR_STRAFE_L][i],
                           OL_MOTOR_SIGNS[OL_DIR_STRAFE_R][i]);
    }
}

void test_motor_table_yaw_ccw_mirrors_yaw_cw() {
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL(-OL_MOTOR_SIGNS[OL_DIR_YAW_CCW][i],
                           OL_MOTOR_SIGNS[OL_DIR_YAW_CW][i]);
    }
}

void test_motor_table_strafe_left_sign_pattern() {
    // Mecanum strafe-left in internal [L1, R1, L2, R2] order:
    //   L1 forward, R1 reverse, L2 reverse, R2 forward
    // Derived from the IK formula dy = (r/4)(L1 − R1 + R2 − L2)
    TEST_ASSERT_EQUAL(+1, OL_MOTOR_SIGNS[OL_DIR_STRAFE_L][0]);  // L1
    TEST_ASSERT_EQUAL(-1, OL_MOTOR_SIGNS[OL_DIR_STRAFE_L][1]);  // R1
    TEST_ASSERT_EQUAL(-1, OL_MOTOR_SIGNS[OL_DIR_STRAFE_L][2]);  // L2
    TEST_ASSERT_EQUAL(+1, OL_MOTOR_SIGNS[OL_DIR_STRAFE_L][3]);  // R2
}

void test_motor_table_yaw_ccw_sign_pattern() {
    // Yaw CCW: left wheels reverse, right wheels forward.
    //   L1 reverse, R1 forward, L2 reverse, R2 forward
    TEST_ASSERT_EQUAL(-1, OL_MOTOR_SIGNS[OL_DIR_YAW_CCW][0]);  // L1
    TEST_ASSERT_EQUAL(+1, OL_MOTOR_SIGNS[OL_DIR_YAW_CCW][1]);  // R1
    TEST_ASSERT_EQUAL(-1, OL_MOTOR_SIGNS[OL_DIR_YAW_CCW][2]);  // L2
    TEST_ASSERT_EQUAL(+1, OL_MOTOR_SIGNS[OL_DIR_YAW_CCW][3]);  // R2
}

void test_motor_table_rows_are_pure_sign() {
    // Every entry in the table must be +1, -1, or 0 — no magnitudes.
    // The base PWM magnitude is applied at runtime by multiplication;
    // if any sign row sneaks in a non-unit value we'd get wrong
    // amplitudes.
    for (int d = 0; d < OL_DIR_COUNT; d++) {
        for (int i = 0; i < 4; i++) {
            const int8_t v = OL_MOTOR_SIGNS[d][i];
            TEST_ASSERT_TRUE(v == -1 || v == 0 || v == 1);
        }
    }
}

// ============================================
// Runner
// ============================================

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_classify_translate_cardinal);
    RUN_TEST(test_classify_translate_diagonal_rejected);
    RUN_TEST(test_classify_translate_zero_rejected);
    RUN_TEST(test_classify_translate_near_zero_treated_as_zero);
    RUN_TEST(test_classify_yaw);
    RUN_TEST(test_classify_strafe_circle_rejected);
    RUN_TEST(test_classify_pause_is_invalid);
    RUN_TEST(test_expected_counts_translate);
    RUN_TEST(test_expected_counts_yaw_ninety_degrees);
    RUN_TEST(test_expected_counts_pause_is_zero);
    RUN_TEST(test_motor_table_forward);
    RUN_TEST(test_motor_table_backward_mirrors_forward);
    RUN_TEST(test_motor_table_strafe_left_mirrors_strafe_right);
    RUN_TEST(test_motor_table_yaw_ccw_mirrors_yaw_cw);
    RUN_TEST(test_motor_table_strafe_left_sign_pattern);
    RUN_TEST(test_motor_table_yaw_ccw_sign_pattern);
    RUN_TEST(test_motor_table_rows_are_pure_sign);
    return UNITY_END();
}
