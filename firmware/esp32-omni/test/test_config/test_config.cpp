/**
 * @file test_config.cpp
 * @brief Unit tests for configuration constants and calculations
 */

#include <unity.h>
#include <stdint.h>
#include <math.h>

// Replicate config values for native testing
#define PCNT_H_LIM  32767
#define PCNT_L_LIM -32768
#define PCNT_OVERFLOW_THRESHOLD (PCNT_H_LIM / 2)

#define ENCODER_PPR 13
#define GEAR_RATIO 42
#define COUNTS_PER_MOTOR_REV (ENCODER_PPR * 2)
#define COUNTS_PER_WHEEL_REV (COUNTS_PER_MOTOR_REV * GEAR_RATIO)

#define WHEEL_RADIUS 0.04f
#define WHEEL_CIRCUMFERENCE (2.0f * 3.14159265f * WHEEL_RADIUS)
#define METERS_PER_COUNT (WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV)

void setUp(void) {}
void tearDown(void) {}

// ============================================
// PCNT Overflow Threshold Tests
// ============================================

void test_overflow_threshold_is_positive(void) {
    TEST_ASSERT_GREATER_THAN(0, PCNT_OVERFLOW_THRESHOLD);
}

void test_overflow_threshold_is_half_of_max(void) {
    int expected = PCNT_H_LIM / 2;
    TEST_ASSERT_EQUAL_INT(expected, PCNT_OVERFLOW_THRESHOLD);
}

void test_overflow_threshold_value(void) {
    // 32767 / 2 = 16383
    TEST_ASSERT_EQUAL_INT(16383, PCNT_OVERFLOW_THRESHOLD);
}

void test_overflow_threshold_less_than_max(void) {
    TEST_ASSERT_LESS_THAN(PCNT_H_LIM, PCNT_OVERFLOW_THRESHOLD);
}

void test_normal_motion_under_threshold(void) {
    // At 50Hz sample rate, max expected encoder change is ~500 counts
    // This should be well under the threshold
    int16_t max_expected_motion = 500;
    TEST_ASSERT_TRUE(max_expected_motion < PCNT_OVERFLOW_THRESHOLD);
}

// ============================================
// Encoder Configuration Tests
// ============================================

void test_counts_per_wheel_rev_calculation(void) {
    // 13 PPR * 2 (quadrature) * 42 (gear ratio) = 1092
    int expected = 13 * 2 * 42;
    TEST_ASSERT_EQUAL_INT(expected, COUNTS_PER_WHEEL_REV);
}

void test_counts_per_wheel_rev_value(void) {
    TEST_ASSERT_EQUAL_INT(1092, COUNTS_PER_WHEEL_REV);
}

void test_counts_per_wheel_rev_is_reasonable(void) {
    // Should be > 100 for reasonable resolution
    TEST_ASSERT_GREATER_THAN(100, COUNTS_PER_WHEEL_REV);
    // Should be < 10000 for typical hobby encoders
    TEST_ASSERT_LESS_THAN(10000, COUNTS_PER_WHEEL_REV);
}

void test_meters_per_count_is_submillimeter(void) {
    // Each count should represent a small distance (sub-millimeter)
    TEST_ASSERT_TRUE(METERS_PER_COUNT < 0.001f);  // Less than 1mm
}

void test_meters_per_count_is_reasonable(void) {
    // Should be more than 0.1mm for practical resolution
    TEST_ASSERT_TRUE(METERS_PER_COUNT > 0.0001f);
}

// ============================================
// Physical Parameter Tests
// ============================================

void test_wheel_circumference_calculation(void) {
    // 80mm diameter wheel -> ~251mm circumference
    float expected = 2.0f * 3.14159265f * 0.04f;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, WHEEL_CIRCUMFERENCE);
}

void test_wheel_circumference_approximate_value(void) {
    // Should be approximately 0.251m
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.251f, WHEEL_CIRCUMFERENCE);
}

void test_wheel_radius_is_reasonable(void) {
    // Wheel radius should be between 20mm and 200mm for a small robot
    TEST_ASSERT_TRUE(WHEEL_RADIUS >= 0.02f);
    TEST_ASSERT_TRUE(WHEEL_RADIUS <= 0.2f);
}

void test_wheel_radius_value(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.04f, WHEEL_RADIUS);
}

// ============================================
// Test Runner
// ============================================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // PCNT overflow tests
    RUN_TEST(test_overflow_threshold_is_positive);
    RUN_TEST(test_overflow_threshold_is_half_of_max);
    RUN_TEST(test_overflow_threshold_value);
    RUN_TEST(test_overflow_threshold_less_than_max);
    RUN_TEST(test_normal_motion_under_threshold);

    // Encoder configuration tests
    RUN_TEST(test_counts_per_wheel_rev_calculation);
    RUN_TEST(test_counts_per_wheel_rev_value);
    RUN_TEST(test_counts_per_wheel_rev_is_reasonable);
    RUN_TEST(test_meters_per_count_is_submillimeter);
    RUN_TEST(test_meters_per_count_is_reasonable);

    // Physical parameter tests
    RUN_TEST(test_wheel_circumference_calculation);
    RUN_TEST(test_wheel_circumference_approximate_value);
    RUN_TEST(test_wheel_radius_is_reasonable);
    RUN_TEST(test_wheel_radius_value);

    return UNITY_END();
}
