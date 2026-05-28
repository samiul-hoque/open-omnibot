/*
 * Omni-2 Robot WebSocket Firmware
 *
 * This firmware provides a WebSocket interface for controlling a 4-wheel
 * mecanum robot. It streams sensor data and accepts velocity commands.
 *
 * WebSocket endpoint: ws://<robot-ip>/ws
 *
 * Message Protocol:
 *
 * Robot -> Server (sensor stream, ~20Hz):
 * {
 *   "type": "sensors",
 *   "t": 123456,                    // timestamp (ms)
 *   "enc": [100, -100, -100, 100],  // encoder counts [L1, R1, R2, L2]
 *   "vel": [1.2, -1.2, -1.2, 1.2],  // wheel velocities (rad/s)
 *   "imu": {...},
 *   "cal": {...},
 *   "pose": {"x": 0.0, "y": 0.0, "th": 0.0}
 * }
 *
 * Server -> Robot (velocity command):
 * {"type": "cmd", "vx": 0.2, "vy": 0.0, "w": 0.1}
 *
 * Server -> Robot (stop):
 * {"type": "stop"}
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ArduinoOTA.h>

#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "mecanum.h"
#include "websocket_server.h"
#include "pid_controller.h"
#include "motor_calibration.h"
#include "odometry.h"
#include "trajectory.h"
#include "openloop_executor.h"
#include "self_test.h"

// ============================================
// Global Objects
// ============================================

Adafruit_MCP23X17 mcp;

// Timing
uint32_t lastSensorBroadcast = 0;
uint32_t lastMotorUpdate = 0;

// rad/s at PWM 255 (feedforward scaling).
//
// Re-measured 2026-04-18 after the motor-gain fix: held the PID at
// target 2.5 rad/s on a free-wheel stand for 30 s (_hold_test.mjs),
// observed steady-state PWM ≈ 29 across all four wheels with
// integrator fully converged. Linear back-extrapolation:
//   MAX = target × 255 / pwm_ss = 2.5 × 255 / 29 = 22 rad/s
// The old 12.0 f value was measured near PWM 255 at the top of the
// curve, which made FF overshoot dramatically at typical operating
// targets (2-3 rad/s) — FF demanded PWM 53 when the motor only needed
// ~29 PWM. Integrator had to claw back ~24 PWM, slow to converge
// (>10 s), producing chronic +5-7% steady-state bias on 3-5 s
// trajectory segments. 22.0 f gives FF near the true operating point
// so the integrator only makes small per-motor corrections.
// Must match the copy in pid_controller.cpp.
const float MAX_WHEEL_SPEED = 22.0f;

// ============================================
// Heading-hold helper
// ============================================
//
// Applies the gyro_z-nulling P controller to `omega` when the robot is
// translating with omega==0 (i.e. commanded straight-line motion,
// whether from teleop or from a trajectory translate/strafe_circle
// segment). Trajectory YAW segments carry a non-zero omega so the
// correction is intentionally skipped — we don't want to fight an
// in-place rotation command. Returns the (possibly modified) omega.
//
// The LPF state is file-scoped static so it persists across calls;
// it's reset to zero whenever the robot is not translating so a
// pause-and-resume doesn't carry stale filter state into the next
// motion. Teleop and trajectory paths share the same filter because
// only one is ever active per tick.
static float hhGzFiltered = 0.0f;

// Public reset hook (declared in websocket_server.h). Called from
// trajectory.cpp at trajStart/trajResume so the filter doesn't carry
// pre-trajectory bias into the first few translate ticks.
void resetHeadingHoldFilter() {
    hhGzFiltered = 0.0f;
}

static float applyHeadingHoldCorrection(float vx, float vy, float omega,
                                        const CachedSensors& sensors) {
    const bool translating = (vx != 0.0f || vy != 0.0f);
    if (!translating) {
        hhGzFiltered = 0.0f;
        return omega;
    }
    if (!isHeadingHoldEnabled() || omega != 0.0f || !sensors.imuValid) {
        return omega;
    }
    const float alpha = getHeadingHoldAlpha();
    hhGzFiltered = alpha * sensors.imu.gyro_z + (1.0f - alpha) * hhGzFiltered;
    if (fabsf(hhGzFiltered) <= getHeadingHoldDeadzone()) return 0.0f;
    // Negate: positive gyro_z (CCW drift) needs negative omega (CW)
    // correction to counteract.
    float correction = -getHeadingHoldGain() * hhGzFiltered;
    if (correction >  HEADING_HOLD_MAX_CORRECTION) correction =  HEADING_HOLD_MAX_CORRECTION;
    if (correction < -HEADING_HOLD_MAX_CORRECTION) correction = -HEADING_HOLD_MAX_CORRECTION;
    return correction;
}

// ============================================
// Setup
// ============================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n========================================");
    Serial.println("   Omni-2 Robot - WebSocket Firmware");
    Serial.println("========================================\n");

    // Initialize I2C
    Wire.begin();
    Serial.println("I2C initialized");

    // Initialize MCP23017
    if (!mcp.begin_I2C(MCP23017_ADDRESS)) {
        Serial.println("ERROR: MCP23017 not found!");
        while (1) { delay(100); }
    }
    Serial.println("MCP23017 initialized");

    // Initialize subsystems
    if (!initMotors(&mcp)) {
        Serial.println("ERROR: Motor initialization failed!");
        while (1) { delay(100); }
    }

    initEncoders();
    initPIDControllers();
    initIMU();  // OK if this fails, we continue without IMU
    autoLoadIMUCalibration();  // Restore saved BNO055 offsets from NVS
    autoLoadMotorCalibration();  // Restore saved motor gain factors from NVS
    openloopLoadCal();             // Restore tier-0 open-loop cal (if any) from NVS
    initOdometry();

    // Initialize WiFi and WebSocket
    initWebSocket();

    // Print robot parameters
    Serial.println("\n--- Robot Parameters ---");
    Serial.printf("Wheel radius: %.3f m\n", WHEEL_RADIUS);
    Serial.printf("Lx + Ly: %.3f m\n", L_SUM);
    Serial.printf("Counts per wheel rev: %d\n", COUNTS_PER_WHEEL_REV);
    Serial.printf("Max wheel speed: %.2f rad/s\n", MAX_WHEEL_SPEED);
    Serial.printf("Sensor update rate: %d ms\n", SENSOR_UPDATE_INTERVAL_MS);
    Serial.printf("Velocity timeout: %d ms\n", VELOCITY_TIMEOUT_MS);
    Serial.println("------------------------\n");

    Serial.println("System ready! Waiting for WebSocket connection...\n");
}

// ============================================
// Main Loop
// ============================================

void loop() {
    ArduinoOTA.handle();

    uint32_t now = millis();

    // Cleanup stale WebSocket clients
    handleWebSocket();

    // Broadcast sensor data at 20Hz — uses cached sensor snapshot from
    // the most recent 50Hz motor update (no redundant I2C/PCNT reads).
    if (now - lastSensorBroadcast >= 50) {
        lastSensorBroadcast = now;
        broadcastSensorData();
    }

    // 50Hz motor update + sensor cache refresh
    if (now - lastMotorUpdate >= 20) {
        lastMotorUpdate = now;

        // Single sensor read per cycle: refreshes encoder counts + IMU
        // data into a shared cache used by PID, odometry, heading-hold,
        // and broadcast. Eliminates redundant I2C and PCNT reads.
        updateSensorCache();
        const CachedSensors& sensors = getSensorCache();

        // Trajectory executor takes priority when running or paused — it
        // owns the velocity command and feeds directly into the PID.
        // External cmd messages are ignored. Heading-hold IS applied on
        // translate / strafe_circle segments (omega==0 commanded) via
        // applyHeadingHoldCorrection — without it, translate segments
        // drift several degrees per metre from chassis asymmetry. Yaw
        // segments carry a non-zero omega so the helper leaves them
        // alone.
        //
        // PAUSED holds the robot still at a waypoint until the server
        // sends traj_resume. We actively command zero velocity through
        // the PID (zero-target short-circuits to coasted stop in
        // applyClosedLoopVelocity) so teleop cannot leak in.
        if (trajGetState() == TRAJ_RUNNING || trajGetState() == TRAJ_PAUSED) {
            // Always call trajTick() first — it drives the state
            // machine (segment advancement, pause/resume transitions,
            // progress broadcasts). Its returned VelocityCommand is
            // used only by the closed-loop path; open-loop ignores
            // the command and drives motors directly from a fixed
            // direction table (see openloop_executor.cpp).
            VelocityCommand tcmd = trajTick();
            TrajectoryState ts = trajGetState();
            const bool openLoop = (trajGetMode() == TRAJ_MODE_OPEN_LOOP);

            if (ts == TRAJ_RUNNING) {
                if (openLoop) {
                    openloopDrive();
                } else {
                    float tomega = applyHeadingHoldCorrection(
                        tcmd.vx, tcmd.vy, tcmd.omega, sensors);
                    applyClosedLoopVelocity(tcmd.vx, tcmd.vy, tomega);
                }
            } else if (ts == TRAJ_PAUSED) {
                if (openLoop) {
                    openloopDrive();  // idles motors + holds baseline
                } else {
                    applyClosedLoopVelocity(0, 0, 0);
                }
            } else {
                // Completed or aborted mid-tick. Safe stop for either mode.
                stopAllMotors();
                if (!openLoop) resetPIDControllers();
            }
        }
        // Run motor calibration state machine if active
        else if (isMotorCalibrationRunning()) {
            updateMotorCalibration();
        }
        // Run self-test state machine if active
        else if (isSelfTestRunning()) {
            updateSelfTest();
        } else if (!isCalibrationMode()) {
            // PID loop — skipped during manual calibration (motor_test) or auto-calibration
            float vx, vy, omega;
            getLastVelocityCommand(&vx, &vy, &omega);

            // Check for command timeout (safety feature)
            if (!isVelocityCommandValid()) {
                if (vx != 0 || vy != 0 || omega != 0) {
                    wsLog("Velocity command timeout - stopping motors");
                    resetPIDControllers();
                }
                vx = 0;
                vy = 0;
                omega = 0;
            }

            // Rising-edge integrator reset
            static bool prevCommandNonZero = false;
            const bool currCommandNonZero = (vx != 0 || vy != 0 || omega != 0);
            if (currCommandNonZero && !prevCommandNonZero) {
                resetPIDControllers();
            }
            prevCommandNonZero = currCommandNonZero;

            // IMU heading-hold: shared with the trajectory branch above.
            omega = applyHeadingHoldCorrection(vx, vy, omega, sensors);

            // Apply closed-loop velocity command (feedforward + PID)
            applyClosedLoopVelocity(vx, vy, omega);
        }

        // Odometry integration at 50Hz — uses cached encoder/IMU data.
        // Runs regardless of whether PID ran this cycle (motor cal/test
        // skip PID but we still want to track pose). Uses the measured
        // elapsed time so a slipped cycle (e.g. from OTA/WS work) doesn't
        // under-integrate body motion against the velocity samples.
        {
            static uint32_t lastOdomMs = 0;
            float odomDt = 0.020f;
            if (lastOdomMs != 0) {
                uint32_t elapsed = now - lastOdomMs;
                // Clamp to a sane band: 5 ms (faster than scheduler) up to
                // 250 ms (loop stall). Out-of-range falls back to nominal.
                if (elapsed >= 5 && elapsed <= 250) {
                    odomDt = elapsed / 1000.0f;
                }
            }
            lastOdomMs = now;

            EncoderData odomEnc = readOdomEncoders();
            float imuYaw = sensors.imuValid ? sensors.imu.yaw : NAN;
            float gyroZ  = sensors.imuValid ? sensors.imu.gyro_z : 0.0f;
            odomUpdate(odomEnc.velocities, odomDt, imuYaw, gyroZ);
        }
    }

    // Small delay to prevent watchdog issues
    delay(1);
}
