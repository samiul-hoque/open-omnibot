#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// Firmware Version
// ============================================
#define FIRMWARE_VERSION "1.0.0"

// ============================================
// WiFi Configuration
// ============================================
// Override these in config.local.h (gitignored)
#define WIFI_SSID "your-ssid"
#define WIFI_PASSWORD "your-password"

// Fallback AP mode if WiFi connection fails
#define AP_SSID "Omni-Robot"
#define AP_PASSWORD "omn1R0b0t!AP"

// Local overrides (create this file with your WiFi credentials)
#if __has_include("config.local.h")
#include "config.local.h"
#endif

// ============================================
// I2C Addresses
// ============================================
#define MCP23017_ADDRESS 0x20
#define BNO055_ADDRESS 0x29

// ============================================
// Robot Physical Parameters
// ============================================
// Wheel radius in meters (80mm diameter)
#define WHEEL_RADIUS 0.04f

// Distance from robot center to wheel center (meters)
// Lx = half of track width (left-right): 235mm / 2 = 117.5mm
// Ly = half of wheelbase (front-back): 190.6mm / 2 = 95.3mm
#define LX 0.1175f
#define LY 0.0953f

// Combined L value for mecanum kinematics (Lx + Ly)
#define L_SUM (LX + LY)

// ============================================
// Encoder Configuration
// ============================================
#define ENCODER_PPR 13              // Pulses per revolution (motor shaft)
#define GEAR_RATIO 42               // Gear reduction ratio
#define COUNTS_PER_MOTOR_REV (ENCODER_PPR * 2)  // 2x quadrature = 26
#define COUNTS_PER_WHEEL_REV (COUNTS_PER_MOTOR_REV * GEAR_RATIO)  // 1092

// Derived values
#define WHEEL_CIRCUMFERENCE (2.0f * 3.14159265f * WHEEL_RADIUS)  // ~0.251m
#define METERS_PER_COUNT (WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV)  // ~0.00023m

// PCNT limits
#define PCNT_H_LIM  32767
#define PCNT_L_LIM -32768

// Encoder overflow detection threshold (half of counter range)
#define PCNT_OVERFLOW_THRESHOLD (PCNT_H_LIM / 2)

// ============================================
// Motor Pin Definitions
// ============================================
// Internal motor/encoder index order (used by every array in sensors.cpp,
// motors.cpp, pid_controller.cpp, motor_calibration.cpp):
//   0 = L1 (Left Rear)
//   1 = R1 (Right Rear)
//   2 = L2 (Left Front)
//   3 = R2 (Right Front)
// The WebSocket broadcast permutes positions 2/3 on output to match the
// external wire order [L1, R1, R2, L2] — see websocket_server.cpp.

// PWM pins (ESP32 GPIO)
// NOTE: L2 (front-left) and R2 (front-right) GPIO assignments look
// "swapped" vs the obvious 26↔L2 / 27↔R2 mapping because the front-motor
// PWM harness was wired this way on the chassis. This matches the
// encoder pin flip just below — both motor and encoder sides now point
// at the same physical wheel for each index.
#define MOTOR_L1_PWM 14
#define MOTOR_R1_PWM 25
#define MOTOR_R2_PWM 27
#define MOTOR_L2_PWM 26

// Encoder A pins (ESP32 GPIO)
// NOTE: R2 and L2 encoder pins are intentionally mapped to the GPIOs that
// the harness routes them to — on this chassis the front-left encoder
// wires are physically connected to GPIOs 19/18 and the front-right
// encoder wires are on GPIOs 33/32. Re-plugging the harness to match
// the "obvious" mapping would work equally well; this mapping avoids
// touching the hardware.
#define MOTOR_L1_ENC_A 35
#define MOTOR_R1_ENC_A 36
#define MOTOR_R2_ENC_A 33
#define MOTOR_L2_ENC_A 19

// Encoder B pins (ESP32 GPIO)
#define MOTOR_L1_ENC_B 34
#define MOTOR_R1_ENC_B 39
#define MOTOR_R2_ENC_B 32
#define MOTOR_L2_ENC_B 18

// Direction pins (MCP23017 pins). L2 and R2 are paired with their
// respective PWM GPIOs above — MOTOR_L2_PWM (26) uses MCP pins 3/2 for
// IN1/IN2, MOTOR_R2_PWM (27) uses MCP pins 1/0. (Swapped from the
// naive L2→1/0 / R2→3/2 mapping for the same front-motor-harness
// reason as the PWM pins.)
#define MOTOR_L1_IN1 7
#define MOTOR_L1_IN2 6
#define MOTOR_R1_IN1 5
#define MOTOR_R1_IN2 4
#define MOTOR_R2_IN1 1
#define MOTOR_R2_IN2 0
#define MOTOR_L2_IN1 3
#define MOTOR_L2_IN2 2

// Standby pins (MCP23017 pins)
#define STBY_FRONT 8   // Controls L2, R2
#define STBY_REAR 9    // Controls L1, R1

// ============================================
// PWM Configuration
// ============================================
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_MAX_VALUE 255
#define MOTOR_MAX_PWM_STEP 15  // Max PWM change per 20ms cycle (slew rate limit)

// ============================================
// Timing Configuration
// ============================================
#define SENSOR_UPDATE_INTERVAL_MS 20    // 50Hz sensor streaming
#define VELOCITY_TIMEOUT_MS 500         // Stop if no command received

// Hard sanity caps on incoming velocity commands. Mecanum saturation in
// mecanum.cpp scales wheel speeds, but unbounded inputs (NaN, garbage)
// would still poison the PID. These should comfortably exceed any real
// command — they are a safety clamp, not an operating envelope.
#define MAX_LINEAR_VEL_MPS 2.0f
#define MAX_ANGULAR_VEL_RPS 6.283185f   // ~2π rad/s

// Heading-hold P controller applied when the user commands pure translation
// (omega=0 with non-zero vx/vy) and heading-hold is enabled. Quiescent gyro
// noise on this robot is ~0.001 rad/s std (IMUPLUS mode, verified by
// ground_imu_experiment.mjs quiescent window), so the dead-zone only needs
// to reject the small residual from wheel vibration during motion
// (gz_std ≈ 0.04–0.13 rad/s depending on direction). The LPF stays useful
// for smoothing the dynamic component.
//   LPF_ALPHA  — one-pole IIR on gyro_z before correction. Output =
//                alpha*new + (1-alpha)*prev. Smaller alpha = heavier
//                smoothing. 0.30 balances response against noise.
//   DEADZONE   — if |gz_filtered| < this (rad/s), force correction to 0,
//                so the controller stops hunting when it's already within
//                the noise floor.
//   GAIN       — P-gain. 1.0 was the stability sweet spot at 50 Hz
//                before we added LPF/deadzone; revisit if the loop starts
//                feeling sluggish with those in the path.
//   MAX_CORRECTION — absolute ceiling on injected omega, in rad/s.
// All four are mutable at runtime via
// {"type":"set_heading_hold","enabled":true,"gain":1.0,"deadzone":0.01,"alpha":0.3}.
#define HEADING_HOLD_GAIN_DEFAULT 1.0f
#define HEADING_HOLD_DEADZONE_DEFAULT 0.01f
#define HEADING_HOLD_LPF_ALPHA_DEFAULT 0.30f
#define HEADING_HOLD_MAX_CORRECTION 0.5f

// ============================================
// NTP Configuration
// ============================================
#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.nist.gov"
#define NTP_SYNC_TIMEOUT_MS 5000        // Max wait for initial sync

// ============================================
// Motor Direction Corrections
// ============================================
// +1 / -1 per motor so that a positive PWM command produces body-frame
// forward motion at that wheel, and a forward wheel rotation produces
// an increasing encoder count. These values were determined empirically
// by commanding +50 PWM on each motor in isolation and observing
// (a) the direction the top of the wheel moved relative to the chassis,
// (b) the sign of the resulting encoder counts. A mismatch between
// MOTOR_*_DIR and ENC_*_DIR would leave the closed-loop PID running on
// wrong-sign feedback, so both need to be correct per wheel.
#define MOTOR_L1_DIR -1
#define MOTOR_R1_DIR -1
#define MOTOR_R2_DIR -1
#define MOTOR_L2_DIR -1

// Encoder direction corrections (if count direction is wrong).
// All four set to -1 after 2026-04-18 per-motor stand test: with the
// current MOTOR_*_DIR=-1 (chassis harness convention, set in 975c2d5),
// encoder readback must be negated on every wheel to agree with the
// commanded PWM sign. Leaving the rear at +1 gave rear encoders the
// opposite sign of rear PWM, which made PID wind up to saturation on
// the rear pair and broke on-ground drive.
#define ENC_L1_DIR -1
#define ENC_R1_DIR -1
#define ENC_R2_DIR -1
#define ENC_L2_DIR -1

#endif // CONFIG_H
