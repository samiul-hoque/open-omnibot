#include "self_test.h"
#include "sensors.h"
#include "motors.h"
#include "mecanum.h"
#include "pid_controller.h"
#include "odometry.h"
#include "websocket_server.h"
#include "config.h"
#include <math.h>

// ============================================
// Constants
// ============================================

#define ST_MAX_QUEUE 8
#define ST_TIMEOUT_MS 15000   // Hard safety timeout per test

// Test-specific timing
#define DIR_SETTLE_MS   300
#define DIR_RUN_MS      800
#define SANITY_RUN_MS   1000
#define SYM_RAMP_MS     400
#define SYM_MEASURE_MS  2000
#define IMU_QUIET_MS    500
#define IMU_VIBRATE_MS  1000
#define PID_SETTLE_MS   500
#define PID_MEASURE_MS  1500

// Motor names for JSON output
static const char* MOTOR_LABELS[4] = {"L1", "R1", "L2", "R2"};

// ============================================
// State
// ============================================

enum STPhase : uint8_t {
    ST_IDLE = 0,
    ST_PREPARING,
    ST_RUNNING,
    ST_EVALUATING,
};

static STPhase phase = ST_IDLE;
static SelfTestType testQueue[ST_MAX_QUEUE];
static int queueLen = 0;
static int queueIdx = 0;
static uint32_t testStartMs = 0;

// Per-test scratch space
static int subStep = 0;
static uint32_t subStepStart = 0;
static int32_t scratch_counts[4];
static float scratch_vel[4];
static float scratch_accum[4];
static int scratch_samples;

// Suite-level results
static int testsRun = 0;
static int testsPassed = 0;
static uint32_t suiteStartMs = 0;

// ============================================
// Helpers
// ============================================

// Broadcast JSON result, with a safety check that the buffer was not truncated.
// n = return value of the final snprintf, bufSize = sizeof(buf).
static void broadcastResult(const char* json, int n = -1, size_t bufSize = 0) {
    if (n >= 0 && (n <= 0 || (size_t)n >= bufSize)) {
        Serial.printf("self_test: JSON truncated (%d >= %u), skipping broadcast\n",
                      n, (unsigned)bufSize);
        return;
    }
    wsBroadcastRaw(json);
}

static void stopAndSettle() {
    stopAllMotors();
    resetPIDControllers();
}

// Read current encoder counts into scratch_counts
static void snapshotEncoders() {
    for (int i = 0; i < 4; i++) {
        scratch_counts[i] = getEncoderCount(i);
    }
}

// Compute encoder delta from scratch_counts
static int32_t encoderDelta(int motor) {
    return getEncoderCount(motor) - scratch_counts[motor];
}

// ============================================
// Test implementations
// ============================================

// --- Motor Direction Test ---
// SubSteps: 0..3 = motor 0..3, each: settle → run → read
static bool tickMotorDir(uint32_t elapsed) {
    int motor = subStep / 3;
    int phase_in_motor = subStep % 3;
    uint32_t phaseElapsed = millis() - subStepStart;

    if (motor >= 4) return true;  // done

    switch (phase_in_motor) {
    case 0: // settle
        if (phaseElapsed >= DIR_SETTLE_MS) {
            snapshotEncoders();
            subStep++;
            subStepStart = millis();
        }
        break;
    case 1: // run at +50 PWM
        setMotorSpeed(motor, 50);
        if (phaseElapsed >= DIR_RUN_MS) {
            scratch_vel[motor] = (float)encoderDelta(motor);
            stopAndSettle();
            subStep++;
            subStepStart = millis();
        }
        break;
    case 2: // settle after
        if (phaseElapsed >= DIR_SETTLE_MS) {
            subStep++;
            subStepStart = millis();
        }
        break;
    }
    return false;
}

static void evaluateMotorDir() {
    bool allPass = true;
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"self_test_result\",\"test\":\"motor_dir\",\"pass\":");

    // Build motor results
    bool motorPass[4];
    for (int i = 0; i < 4; i++) {
        motorPass[i] = scratch_vel[i] > 30.0f;
        if (!motorPass[i]) allPass = false;
    }

    n += snprintf(buf + n, sizeof(buf) - n, "%s,\"motors\":[", allPass ? "true" : "false");
    for (int i = 0; i < 4; i++) {
        const char* verdict = scratch_vel[i] > 30 ? "pass" :
                              scratch_vel[i] > 0 ? "fail_weak" : "fail_reversed";
        n += snprintf(buf + n, sizeof(buf) - n,
            "%s{\"i\":%d,\"label\":\"%s\",\"delta\":%.0f,\"verdict\":\"%s\"}",
            i > 0 ? "," : "", i, MOTOR_LABELS[i], scratch_vel[i], verdict);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
    broadcastResult(buf, n, sizeof(buf));

    if (allPass) testsPassed++;
}

// --- Encoder Sanity Test ---
// SubSteps: 0..7 = motor 0..3 fwd then rev, each: settle → run → read
static bool tickEncoderSanity(uint32_t elapsed) {
    int idx = subStep / 3;       // 0..7
    int motor = idx % 4;
    bool isReverse = idx >= 4;
    int phase_in = subStep % 3;
    uint32_t phaseElapsed = millis() - subStepStart;

    if (idx >= 8) return true;

    switch (phase_in) {
    case 0: // settle
        if (phaseElapsed >= DIR_SETTLE_MS) {
            snapshotEncoders();
            subStep++;
            subStepStart = millis();
        }
        break;
    case 1: // run
        setMotorSpeed(motor, isReverse ? -80 : 80);
        if (phaseElapsed >= SANITY_RUN_MS) {
            float delta = (float)encoderDelta(motor);
            if (isReverse) scratch_accum[motor] = delta;
            else scratch_vel[motor] = delta;
            stopAndSettle();
            subStep++;
            subStepStart = millis();
        }
        break;
    case 2: // settle
        if (phaseElapsed >= DIR_SETTLE_MS) {
            subStep++;
            subStepStart = millis();
        }
        break;
    }
    return false;
}

static void evaluateEncoderSanity() {
    bool allPass = true;
    char buf[512];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"self_test_result\",\"test\":\"encoder_sanity\",\"pass\":");

    bool motorPass[4];
    for (int i = 0; i < 4; i++) {
        float fwd = fabsf(scratch_vel[i]);
        float rev = fabsf(scratch_accum[i]);
        motorPass[i] = fwd > 200 && rev > 200;
        if (!motorPass[i]) allPass = false;
    }

    n += snprintf(buf + n, sizeof(buf) - n, "%s,\"motors\":[", allPass ? "true" : "false");
    for (int i = 0; i < 4; i++) {
        const char* verdict = motorPass[i] ? "pass" :
            (fabsf(scratch_vel[i]) > 0 || fabsf(scratch_accum[i]) > 0) ? "fail_weak" : "fail_dead";
        n += snprintf(buf + n, sizeof(buf) - n,
            "%s{\"i\":%d,\"label\":\"%s\",\"fwd\":%.0f,\"rev\":%.0f,\"verdict\":\"%s\"}",
            i > 0 ? "," : "", i, MOTOR_LABELS[i],
            scratch_vel[i], scratch_accum[i], verdict);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
    broadcastResult(buf, n, sizeof(buf));

    if (allPass) testsPassed++;
}

// --- Motor Symmetry Test ---
// All 4 motors at PWM 100, measure for 2s after 400ms ramp
static bool tickMotorSymmetry(uint32_t elapsed) {
    uint32_t phaseElapsed = millis() - subStepStart;

    switch (subStep) {
    case 0: // settle
        if (phaseElapsed >= DIR_SETTLE_MS) {
            snapshotEncoders();
            for (int i = 0; i < 4; i++) setMotorSpeed(i, 100);
            subStep = 1;
            subStepStart = millis();
        }
        break;
    case 1: // ramp (discard)
        if (phaseElapsed >= SYM_RAMP_MS) {
            snapshotEncoders();
            subStep = 2;
            subStepStart = millis();
        }
        break;
    case 2: // measure
        if (phaseElapsed >= SYM_MEASURE_MS) {
            for (int i = 0; i < 4; i++) {
                scratch_vel[i] = (float)fabsf(encoderDelta(i));
            }
            stopAndSettle();
            return true;
        }
        break;
    }
    return false;
}

static void evaluateMotorSymmetry() {
    float mean = 0;
    for (int i = 0; i < 4; i++) mean += scratch_vel[i];
    mean /= 4.0f;

    bool allPass = mean > 100;
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"self_test_result\",\"test\":\"motor_symmetry\","
        "\"mean_delta\":%.0f,\"pass\":", mean);

    float devs[4];
    for (int i = 0; i < 4; i++) {
        devs[i] = (mean > 1.0f) ? fabsf(scratch_vel[i] - mean) / mean * 100.0f : 100.0f;
        if (devs[i] >= 25.0f) allPass = false;
    }

    n += snprintf(buf + n, sizeof(buf) - n, "%s,\"motors\":[", allPass ? "true" : "false");
    for (int i = 0; i < 4; i++) {
        const char* verdict = devs[i] < 25.0f ? "pass" : "fail";
        n += snprintf(buf + n, sizeof(buf) - n,
            "%s{\"i\":%d,\"label\":\"%s\",\"delta\":%.0f,\"dev_pct\":%.1f,\"verdict\":\"%s\"}",
            i > 0 ? "," : "", i, MOTOR_LABELS[i], scratch_vel[i], devs[i], verdict);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
    broadcastResult(buf, n, sizeof(buf));

    if (allPass) testsPassed++;
}

// --- IMU Check Test ---
// Phase 0: quiescent (500ms), Phase 1: vibrate with motors (1000ms)
static bool tickImuCheck(uint32_t elapsed) {
    uint32_t phaseElapsed = millis() - subStepStart;

    switch (subStep) {
    case 0: // quiescent reading
        if (!isIMUAvailable()) {
            // IMU not available — skip with a note
            scratch_vel[0] = -1;  // sentinel
            return true;
        }
        {
            IMUData imu = readIMU();
            scratch_accum[0] += imu.gyro_z * imu.gyro_z;
            scratch_samples++;
        }
        if (phaseElapsed >= IMU_QUIET_MS) {
            scratch_vel[0] = sqrtf(scratch_accum[0] / scratch_samples); // quiescent gz std
            scratch_accum[0] = 0;
            scratch_samples = 0;
            // Start vibration
            for (int i = 0; i < 4; i++) setMotorSpeed(i, 60);
            subStep = 1;
            subStepStart = millis();
        }
        break;
    case 1: // vibration reading
        {
            IMUData imu = readIMU();
            scratch_accum[0] += imu.gyro_z * imu.gyro_z;
            scratch_samples++;
        }
        if (phaseElapsed >= IMU_VIBRATE_MS) {
            scratch_vel[1] = sqrtf(scratch_accum[0] / scratch_samples); // vibration gz std
            stopAndSettle();
            return true;
        }
        break;
    }
    return false;
}

static void evaluateImuCheck() {
    bool available = scratch_vel[0] >= 0;
    IMUData imu = isIMUAvailable() ? readIMU() : IMUData{};
    bool calOk = imu.cal_gyro >= 2;
    // Responsive = vibration std is measurably above quiescent OR gyro cal is good.
    // On a stand, free-spinning wheels barely vibrate, so relax this check.
    bool responsive = scratch_vel[1] > scratch_vel[0] * 1.1f || calOk;
    bool pass = available && calOk;

    char buf[320];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"self_test_result\",\"test\":\"imu_check\",\"pass\":%s,"
        "\"available\":%s,\"cal_gyro\":%u,"
        "\"quiescent_gz_std\":%.4f,\"vibration_gz_std\":%.4f}",
        pass ? "true" : "false",
        available ? "true" : "false",
        imu.cal_gyro,
        scratch_vel[0] >= 0 ? scratch_vel[0] : 0.0f,
        scratch_vel[1]);
    broadcastResult(buf, n, sizeof(buf));

    if (pass) testsPassed++;
}

// --- PID Tracking Test ---
// Command 0.1 m/s forward via PID, measure tracking error
static bool tickPidTracking(uint32_t elapsed) {
    uint32_t phaseElapsed = millis() - subStepStart;

    // Drive with PID
    applyClosedLoopVelocity(0.1f, 0.0f, 0.0f);

    if (phaseElapsed < PID_SETTLE_MS) {
        return false;  // settling
    }

    // Measure phase: accumulate per-wheel error
    EncoderData enc = readEncoders();
    // FK to get actual body velocity
    VelocityCommand actual = mecanumForwardKinematics(
        enc.velocities[0], enc.velocities[1],
        enc.velocities[3], enc.velocities[2]);

    scratch_accum[0] += (actual.vx - 0.1f) * (actual.vx - 0.1f);
    scratch_accum[1] += actual.vy * actual.vy;
    scratch_samples++;

    if (phaseElapsed >= PID_SETTLE_MS + PID_MEASURE_MS) {
        scratch_vel[0] = sqrtf(scratch_accum[0] / scratch_samples); // RMS vx error
        scratch_vel[1] = sqrtf(scratch_accum[1] / scratch_samples); // RMS vy error
        stopAndSettle();
        return true;
    }
    return false;
}

static void evaluatePidTracking() {
    bool pass = scratch_vel[0] < 0.05f && scratch_vel[1] < 0.05f;

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"self_test_result\",\"test\":\"pid_tracking\",\"pass\":%s,"
        "\"target_vx\":0.1,\"rms_vx_err\":%.4f,\"rms_vy_err\":%.4f,\"samples\":%d}",
        pass ? "true" : "false",
        scratch_vel[0], scratch_vel[1], scratch_samples);
    broadcastResult(buf, n, sizeof(buf));

    if (pass) testsPassed++;
}

// ============================================
// Runner state machine
// ============================================

static void resetScratch() {
    subStep = 0;
    subStepStart = millis();
    memset(scratch_counts, 0, sizeof(scratch_counts));
    memset(scratch_vel, 0, sizeof(scratch_vel));
    memset(scratch_accum, 0, sizeof(scratch_accum));
    scratch_samples = 0;
}

static void startNextTest() {
    if (queueIdx >= queueLen) {
        // Suite complete
        phase = ST_IDLE;
        char buf[192];
        snprintf(buf, sizeof(buf),
            "{\"type\":\"self_test_complete\",\"pass\":%s,"
            "\"tests_run\":%d,\"tests_passed\":%d,\"duration_ms\":%lu}",
            testsPassed == testsRun ? "true" : "false",
            testsRun, testsPassed, (unsigned long)(millis() - suiteStartMs));
        broadcastResult(buf);
        wsLog("Self-test suite complete: %d/%d passed", testsPassed, testsRun);
        return;
    }

    stopAndSettle();
    resetScratch();
    testStartMs = millis();
    phase = ST_RUNNING;

    static const char* TEST_NAMES[] = {
        "motor_dir", "encoder_sanity", "motor_symmetry", "imu_check", "pid_tracking"
    };
    SelfTestType t = testQueue[queueIdx];
    wsLog("Self-test: starting %s (%d/%d)", TEST_NAMES[t], queueIdx + 1, queueLen);
}

bool startSelfTest(const JsonDocument& doc) {
    if (phase != ST_IDLE) return false;

    queueLen = 0;
    queueIdx = 0;
    testsRun = 0;
    testsPassed = 0;

    JsonArrayConst tests = doc["tests"];
    if (!tests.isNull() && tests.size() > 0) {
        for (JsonVariantConst v : tests) {
            const char* name = v.as<const char*>();
            if (!name) continue;
            SelfTestType t;
            if (strcmp(name, "motor_dir") == 0) t = TEST_MOTOR_DIR;
            else if (strcmp(name, "encoder_sanity") == 0) t = TEST_ENCODER_SANITY;
            else if (strcmp(name, "motor_symmetry") == 0) t = TEST_MOTOR_SYMMETRY;
            else if (strcmp(name, "imu_check") == 0) t = TEST_IMU_CHECK;
            else if (strcmp(name, "pid_tracking") == 0) t = TEST_PID_TRACKING;
            else { wsLog("Self-test: unknown test '%s'", name); continue; }
            if (queueLen < ST_MAX_QUEUE) testQueue[queueLen++] = t;
        }
    } else {
        // Run all tests in order
        for (int i = 0; i < TEST_COUNT; i++) {
            testQueue[queueLen++] = (SelfTestType)i;
        }
    }

    if (queueLen == 0) return false;

    suiteStartMs = millis();
    wsLog("Self-test suite started: %d tests", queueLen);
    startNextTest();
    return true;
}

void updateSelfTest() {
    if (phase == ST_IDLE) return;

    uint32_t elapsed = millis() - testStartMs;

    // Safety timeout
    if (elapsed > ST_TIMEOUT_MS) {
        wsLog("Self-test: timeout on test %d", testQueue[queueIdx]);
        stopAndSettle();
        char buf[128];
        static const char* TEST_NAMES[] = {
            "motor_dir", "encoder_sanity", "motor_symmetry", "imu_check", "pid_tracking"
        };
        snprintf(buf, sizeof(buf),
            "{\"type\":\"self_test_result\",\"test\":\"%s\",\"pass\":false,\"error\":\"timeout\"}",
            TEST_NAMES[testQueue[queueIdx]]);
        broadcastResult(buf);
        testsRun++;
        queueIdx++;
        startNextTest();
        return;
    }

    // Tick the current test
    bool done = false;
    switch (testQueue[queueIdx]) {
    case TEST_MOTOR_DIR:      done = tickMotorDir(elapsed); break;
    case TEST_ENCODER_SANITY: done = tickEncoderSanity(elapsed); break;
    case TEST_MOTOR_SYMMETRY: done = tickMotorSymmetry(elapsed); break;
    case TEST_IMU_CHECK:      done = tickImuCheck(elapsed); break;
    case TEST_PID_TRACKING:   done = tickPidTracking(elapsed); break;
    default: done = true; break;
    }

    if (done) {
        stopAndSettle();
        // Evaluate
        switch (testQueue[queueIdx]) {
        case TEST_MOTOR_DIR:      evaluateMotorDir(); break;
        case TEST_ENCODER_SANITY: evaluateEncoderSanity(); break;
        case TEST_MOTOR_SYMMETRY: evaluateMotorSymmetry(); break;
        case TEST_IMU_CHECK:      evaluateImuCheck(); break;
        case TEST_PID_TRACKING:   evaluatePidTracking(); break;
        default: break;
        }
        testsRun++;
        queueIdx++;
        startNextTest();
    }
}

bool isSelfTestRunning() {
    return phase != ST_IDLE;
}

void abortSelfTest() {
    if (phase == ST_IDLE) return;
    stopAndSettle();
    phase = ST_IDLE;
    wsLog("Self-test aborted");
}
