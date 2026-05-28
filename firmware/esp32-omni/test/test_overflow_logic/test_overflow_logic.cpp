/**
 * @file test_overflow_logic.cpp
 * @brief Unit tests for encoder overflow detection logic
 *
 * Tests the core counting and threshold behavior. Note that int16_t
 * subtraction naturally handles counter wrap-around for small motion,
 * so the threshold check is mainly for very high-speed scenarios.
 */

#include <unity.h>
#include <stdint.h>

#define PCNT_H_LIM  32767
#define PCNT_L_LIM -32768
#define PCNT_OVERFLOW_THRESHOLD (PCNT_H_LIM / 2)

struct EncoderState {
    int32_t overflowCount;
    int16_t lastRawCount;
};

int32_t simulateEncoderCount(EncoderState* state, int16_t rawCount, int8_t direction) {
    int16_t diff = rawCount - state->lastRawCount;

    if (diff > PCNT_OVERFLOW_THRESHOLD) {
        state->overflowCount -= 65536;
    } else if (diff < -PCNT_OVERFLOW_THRESHOLD) {
        state->overflowCount += 65536;
    }

    state->lastRawCount = rawCount;
    return (state->overflowCount + rawCount) * direction;
}

void setUp(void) {}
void tearDown(void) {}

// ============================================
// Basic Counting Tests
// ============================================

void test_basic_count_forward(void) {
    EncoderState state = {0, 0};
    int32_t result = simulateEncoderCount(&state, 100, 1);
    TEST_ASSERT_EQUAL_INT32(100, result);
}

void test_basic_count_backward(void) {
    EncoderState state = {0, 100};
    int32_t result = simulateEncoderCount(&state, 50, 1);
    TEST_ASSERT_EQUAL_INT32(50, result);
}

void test_direction_inversion(void) {
    EncoderState state = {0, 0};
    int32_t result = simulateEncoderCount(&state, 100, -1);
    TEST_ASSERT_EQUAL_INT32(-100, result);
}

void test_incremental_counting(void) {
    EncoderState state = {0, 0};
    for (int i = 1; i <= 10; i++) {
        int32_t result = simulateEncoderCount(&state, i * 100, 1);
        TEST_ASSERT_EQUAL_INT32(i * 100, result);
    }
}

// ============================================
// No False Positive Tests
// ============================================

void test_no_false_overflow_on_normal_motion(void) {
    EncoderState state = {0, 0};
    for (int16_t i = 0; i < 1000; i += 10) {
        simulateEncoderCount(&state, i, 1);
    }
    TEST_ASSERT_EQUAL_INT32(0, state.overflowCount);
}

void test_no_false_overflow_on_backward_motion(void) {
    EncoderState state = {0, 1000};
    for (int16_t i = 1000; i > 0; i -= 10) {
        simulateEncoderCount(&state, i, 1);
    }
    TEST_ASSERT_EQUAL_INT32(0, state.overflowCount);
}

void test_threshold_boundary_no_overflow(void) {
    EncoderState state = {0, 0};
    // Jump just under threshold should NOT trigger overflow
    int16_t justUnderThreshold = PCNT_OVERFLOW_THRESHOLD - 1;
    int32_t result = simulateEncoderCount(&state, justUnderThreshold, 1);
    TEST_ASSERT_EQUAL_INT32(justUnderThreshold, result);
    TEST_ASSERT_EQUAL_INT32(0, state.overflowCount);
}

void test_negative_threshold_boundary_no_overflow(void) {
    EncoderState state = {0, 0};
    // Negative jump just under threshold should NOT trigger
    int16_t justUnderThreshold = -(PCNT_OVERFLOW_THRESHOLD - 1);
    int32_t result = simulateEncoderCount(&state, justUnderThreshold, 1);
    TEST_ASSERT_EQUAL_INT32(justUnderThreshold, result);
    TEST_ASSERT_EQUAL_INT32(0, state.overflowCount);
}

void test_zero_crossing_no_overflow(void) {
    EncoderState state = {0, 100};
    int32_t result = simulateEncoderCount(&state, -100, 1);
    TEST_ASSERT_EQUAL_INT32(-100, result);
    TEST_ASSERT_EQUAL_INT32(0, state.overflowCount);
}

// ============================================
// Motion Pattern Tests
// ============================================

void test_forward_then_backward(void) {
    EncoderState state = {0, 0};
    simulateEncoderCount(&state, 1000, 1);
    int32_t result = simulateEncoderCount(&state, -500, 1);
    TEST_ASSERT_EQUAL_INT32(-500, result);
}

void test_oscillating_motion(void) {
    EncoderState state = {0, 0};
    simulateEncoderCount(&state, 100, 1);
    simulateEncoderCount(&state, -100, 1);
    simulateEncoderCount(&state, 100, 1);
    int32_t result = simulateEncoderCount(&state, 0, 1);
    TEST_ASSERT_EQUAL_INT32(0, result);
    TEST_ASSERT_EQUAL_INT32(0, state.overflowCount);
}

void test_large_oscillation(void) {
    EncoderState state = {0, 0};
    // Large but sub-threshold oscillations
    simulateEncoderCount(&state, 10000, 1);
    simulateEncoderCount(&state, -10000, 1);
    simulateEncoderCount(&state, 10000, 1);
    int32_t result = simulateEncoderCount(&state, 0, 1);
    TEST_ASSERT_EQUAL_INT32(0, result);
    TEST_ASSERT_EQUAL_INT32(0, state.overflowCount);
}

// ============================================
// Threshold Value Tests
// ============================================

void test_threshold_value_is_correct(void) {
    // Threshold should be half of max counter value
    TEST_ASSERT_EQUAL_INT(16383, PCNT_OVERFLOW_THRESHOLD);
}

void test_threshold_is_positive(void) {
    TEST_ASSERT_GREATER_THAN(0, PCNT_OVERFLOW_THRESHOLD);
}

void test_threshold_less_than_max(void) {
    TEST_ASSERT_LESS_THAN(PCNT_H_LIM, PCNT_OVERFLOW_THRESHOLD);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Basic counting
    RUN_TEST(test_basic_count_forward);
    RUN_TEST(test_basic_count_backward);
    RUN_TEST(test_direction_inversion);
    RUN_TEST(test_incremental_counting);

    // No false positives
    RUN_TEST(test_no_false_overflow_on_normal_motion);
    RUN_TEST(test_no_false_overflow_on_backward_motion);
    RUN_TEST(test_threshold_boundary_no_overflow);
    RUN_TEST(test_negative_threshold_boundary_no_overflow);
    RUN_TEST(test_zero_crossing_no_overflow);

    // Motion patterns
    RUN_TEST(test_forward_then_backward);
    RUN_TEST(test_oscillating_motion);
    RUN_TEST(test_large_oscillation);

    // Threshold values
    RUN_TEST(test_threshold_value_is_correct);
    RUN_TEST(test_threshold_is_positive);
    RUN_TEST(test_threshold_less_than_max);

    return UNITY_END();
}
