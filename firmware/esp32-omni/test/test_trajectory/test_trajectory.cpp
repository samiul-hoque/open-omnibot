/**
 * @file test_trajectory.cpp
 * @brief Unit tests for trajectory segment velocity sampling and duration
 *        computation. Runs in PlatformIO native environment (x86).
 */

#include <unity.h>
#include <math.h>
#include <string.h>

// ============================================
// Reproduce types and helpers (no hardware deps)
// ============================================

#define MAX_TRAJECTORY_SEGMENTS 16

enum SegmentKind : uint8_t {
    SEG_TRANSLATE = 0,
    SEG_YAW = 1,
    SEG_STRAFE_CIRCLE = 2,
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

struct VelocityCommand { float vx, vy, omega; };

static float computeSegmentDuration(const TrajectorySegment& seg) {
    switch (seg.kind) {
        case SEG_TRANSLATE: {
            float speed = sqrtf(seg.translate.vx * seg.translate.vx +
                                seg.translate.vy * seg.translate.vy);
            if (speed < 1e-6f) return 0;
            return (seg.translate.distance / speed) * 1000.0f;
        }
        case SEG_YAW: {
            float absW = fabsf(seg.yaw.w);
            if (absW < 1e-6f) return 0;
            return (fabsf(seg.yaw.angle) / absW) * 1000.0f;
        }
        case SEG_STRAFE_CIRCLE: {
            float speed = seg.circle.speed;
            if (speed < 1e-6f) return 0;
            return (2.0f * M_PI * seg.circle.radius / speed) * 1000.0f;
        }
        default:
            return 0;
    }
}

static VelocityCommand segmentVelocityAt(const TrajectorySegment& seg, float elapsedMs) {
    VelocityCommand cmd = {0, 0, 0};
    switch (seg.kind) {
        case SEG_TRANSLATE:
            cmd.vx = seg.translate.vx;
            cmd.vy = seg.translate.vy;
            cmd.omega = 0;
            break;
        case SEG_YAW: {
            float sign = (seg.yaw.angle >= 0) ? 1.0f : -1.0f;
            cmd.omega = sign * fabsf(seg.yaw.w);
            break;
        }
        case SEG_STRAFE_CIRCLE: {
            float omega = seg.circle.speed / seg.circle.radius;
            float theta = omega * (elapsedMs / 1000.0f);
            cmd.vx = seg.circle.speed * sinf(theta);
            cmd.vy = seg.circle.speed * cosf(theta);
            break;
        }
    }
    return cmd;
}

void setUp(void) {}
void tearDown(void) {}

// ============================================
// Duration computation tests
// ============================================

void test_translate_duration(void) {
    TrajectorySegment seg;
    seg.kind = SEG_TRANSLATE;
    seg.translate.vx = 0.2f;
    seg.translate.vy = 0.0f;
    seg.translate.distance = 2.0f;
    float d = computeSegmentDuration(seg);
    // 2.0m / 0.2 m/s = 10s = 10000ms
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 10000.0f, d);
}

void test_translate_diagonal_duration(void) {
    TrajectorySegment seg;
    seg.kind = SEG_TRANSLATE;
    seg.translate.vx = 0.1f;
    seg.translate.vy = 0.1f;
    seg.translate.distance = 1.0f;
    float d = computeSegmentDuration(seg);
    // speed = sqrt(0.01+0.01) = 0.1414 m/s, duration = 1.0/0.1414 = 7071ms
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 7071.0f, d);
}

void test_yaw_duration(void) {
    TrajectorySegment seg;
    seg.kind = SEG_YAW;
    seg.yaw.w = 0.5f;
    seg.yaw.angle = (float)M_PI / 2.0f;
    float d = computeSegmentDuration(seg);
    // (pi/2) / 0.5 = pi seconds = 3141.6ms
    TEST_ASSERT_FLOAT_WITHIN(1.0f, (float)(M_PI * 1000.0), d);
}

void test_yaw_negative_angle_duration(void) {
    TrajectorySegment seg;
    seg.kind = SEG_YAW;
    seg.yaw.w = 0.5f;
    seg.yaw.angle = -(float)M_PI / 2.0f;
    float d = computeSegmentDuration(seg);
    // Same duration regardless of sign
    TEST_ASSERT_FLOAT_WITHIN(1.0f, (float)(M_PI * 1000.0), d);
}

void test_circle_duration(void) {
    TrajectorySegment seg;
    seg.kind = SEG_STRAFE_CIRCLE;
    seg.circle.speed = 0.2f;
    seg.circle.radius = 0.5f;
    float d = computeSegmentDuration(seg);
    // circumference = 2*pi*0.5 = pi m, time = pi/0.2 = 5*pi s ≈ 15708ms
    float expected = 2.0f * (float)M_PI * 0.5f / 0.2f * 1000.0f;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected, d);
}

void test_zero_speed_returns_zero_duration(void) {
    TrajectorySegment seg;
    seg.kind = SEG_TRANSLATE;
    seg.translate.vx = 0;
    seg.translate.vy = 0;
    seg.translate.distance = 1.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, computeSegmentDuration(seg));
}

// ============================================
// Velocity sampling tests
// ============================================

void test_translate_velocity(void) {
    TrajectorySegment seg;
    seg.kind = SEG_TRANSLATE;
    seg.translate.vx = 0.3f;
    seg.translate.vy = -0.1f;
    seg.translate.distance = 1.0f;
    seg.durationMs = computeSegmentDuration(seg);

    // Velocity is constant throughout
    VelocityCommand v0 = segmentVelocityAt(seg, 0.0f);
    VelocityCommand v1 = segmentVelocityAt(seg, 500.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, v0.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.1f, v0.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, v0.omega);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, v0.vx, v1.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, v0.vy, v1.vy);
}

void test_yaw_positive_velocity(void) {
    TrajectorySegment seg;
    seg.kind = SEG_YAW;
    seg.yaw.w = 0.5f;
    seg.yaw.angle = 1.5708f;  // +pi/2
    seg.durationMs = computeSegmentDuration(seg);

    VelocityCommand v = segmentVelocityAt(seg, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, v.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, v.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, v.omega);  // positive
}

void test_yaw_negative_velocity(void) {
    TrajectorySegment seg;
    seg.kind = SEG_YAW;
    seg.yaw.w = 0.5f;
    seg.yaw.angle = -1.5708f;  // -pi/2
    seg.durationMs = computeSegmentDuration(seg);

    VelocityCommand v = segmentVelocityAt(seg, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.5f, v.omega);  // negative
}

void test_circle_velocity_at_start(void) {
    TrajectorySegment seg;
    seg.kind = SEG_STRAFE_CIRCLE;
    seg.circle.speed = 0.2f;
    seg.circle.radius = 0.5f;
    seg.durationMs = computeSegmentDuration(seg);

    // At t=0: theta=0, vx=speed*sin(0)=0, vy=speed*cos(0)=speed
    VelocityCommand v = segmentVelocityAt(seg, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, v.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, v.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, v.omega);
}

void test_circle_velocity_at_quarter(void) {
    TrajectorySegment seg;
    seg.kind = SEG_STRAFE_CIRCLE;
    seg.circle.speed = 0.2f;
    seg.circle.radius = 0.5f;
    seg.durationMs = computeSegmentDuration(seg);

    // Quarter revolution: omega = speed/radius = 0.4 rad/s
    // t = (pi/2) / omega = pi/0.8 s = 3927ms
    // At quarter: theta=pi/2, vx=0.2*sin(pi/2)=0.2, vy=0.2*cos(pi/2)=0
    float tQuarter = ((float)M_PI / 2.0f) / (0.2f / 0.5f) * 1000.0f;
    VelocityCommand v = segmentVelocityAt(seg, tQuarter);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.2f, v.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, v.vy);
}

void test_circle_velocity_at_half(void) {
    TrajectorySegment seg;
    seg.kind = SEG_STRAFE_CIRCLE;
    seg.circle.speed = 0.2f;
    seg.circle.radius = 0.5f;
    seg.durationMs = computeSegmentDuration(seg);

    // Half revolution: theta=pi, vx=0.2*sin(pi)≈0, vy=0.2*cos(pi)=-0.2
    float tHalf = (float)M_PI / (0.2f / 0.5f) * 1000.0f;
    VelocityCommand v = segmentVelocityAt(seg, tHalf);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, v.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.2f, v.vy);
}

// ============================================
// Square trajectory (multi-segment) test
// ============================================

void test_square_rotate_total_duration(void) {
    // Unit test for computeSegmentDuration on a 4-side + 4-yaw fixture —
    // independent of the server-side trajectory catalog. Uses 1 m sides
    // and pi/2 yaws purely for a round expected-value calculation.
    float speed = 0.2f;
    float yawSpeed = 0.5f;

    TrajectorySegment segs[8];
    float total = 0;
    for (int i = 0; i < 4; i++) {
        segs[i*2].kind = SEG_TRANSLATE;
        segs[i*2].translate.vx = speed;
        segs[i*2].translate.vy = 0;
        segs[i*2].translate.distance = 1.0f;
        segs[i*2].durationMs = computeSegmentDuration(segs[i*2]);
        total += segs[i*2].durationMs;

        segs[i*2+1].kind = SEG_YAW;
        segs[i*2+1].yaw.w = yawSpeed;
        segs[i*2+1].yaw.angle = (float)M_PI / 2.0f;
        segs[i*2+1].durationMs = computeSegmentDuration(segs[i*2+1]);
        total += segs[i*2+1].durationMs;
    }

    // 4 * (1.0/0.2)*1000 + 4 * (pi/2)/0.5*1000
    // = 4*5000 + 4*3141.6 = 20000 + 12566 = 32566ms
    float expected = 4.0f * (1.0f/speed * 1000.0f) +
                     4.0f * (((float)M_PI/2.0f)/yawSpeed * 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, expected, total);
}

// ============================================
// Main
// ============================================

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // Duration computation
    RUN_TEST(test_translate_duration);
    RUN_TEST(test_translate_diagonal_duration);
    RUN_TEST(test_yaw_duration);
    RUN_TEST(test_yaw_negative_angle_duration);
    RUN_TEST(test_circle_duration);
    RUN_TEST(test_zero_speed_returns_zero_duration);

    // Velocity sampling
    RUN_TEST(test_translate_velocity);
    RUN_TEST(test_yaw_positive_velocity);
    RUN_TEST(test_yaw_negative_velocity);
    RUN_TEST(test_circle_velocity_at_start);
    RUN_TEST(test_circle_velocity_at_quarter);
    RUN_TEST(test_circle_velocity_at_half);

    // Multi-segment
    RUN_TEST(test_square_rotate_total_duration);

    return UNITY_END();
}
