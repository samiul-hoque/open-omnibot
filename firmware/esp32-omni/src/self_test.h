#ifndef SELF_TEST_H
#define SELF_TEST_H

#include <Arduino.h>
#include <ArduinoJson.h>  // for JsonDocument parameter in startSelfTest

// ============================================
// Self-Test Types
// ============================================

enum SelfTestType : uint8_t {
    TEST_MOTOR_DIR = 0,        // Each motor +50 PWM → encoder delta > 0
    TEST_ENCODER_SANITY = 1,   // PWM 80 for 1s → delta in expected range
    TEST_MOTOR_SYMMETRY = 2,   // All motors same PWM → velocities within 25%
    TEST_IMU_CHECK = 3,        // BNO055 alive, gyro responsive
    TEST_PID_TRACKING = 4,     // Command 0.1 m/s → tracking error < threshold
    TEST_COUNT = 5,            // Sentinel: number of test types
};

// ============================================
// Public API
// ============================================

// Start self-test suite. Parses optional "tests" array from JSON.
// If omitted or empty, runs all tests in order.
bool startSelfTest(const JsonDocument& doc);

// Call at 50Hz from the motor update loop
void updateSelfTest();

// Check if any test is running
bool isSelfTestRunning();

// Force-abort (on disconnect, stop, etc.)
void abortSelfTest();

#endif // SELF_TEST_H
