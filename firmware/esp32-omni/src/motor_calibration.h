#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H

#include <Arduino.h>

// Calibration test parameters
#define MOTOR_CAL_NUM_LEVELS 3
#define MOTOR_CAL_STEP_DURATION_MS 2000
#define MOTOR_CAL_SETTLE_MS 300
#define MOTOR_CAL_RAMP_MS 400        // discard initial ramp-up from measurement

// Start the automated motor calibration routine
void startMotorCalibration();

// Check if calibration is currently running
bool isMotorCalibrationRunning();

// Call from main loop at 20ms interval to drive the state machine
void updateMotorCalibration();

// Force-abort the calibration state machine (e.g. on safety timeout
// or client disconnect). Resets calState to CAL_IDLE and stops motors.
// Safe to call when not running (no-op).
void abortMotorCalibration();

#endif // MOTOR_CALIBRATION_H
