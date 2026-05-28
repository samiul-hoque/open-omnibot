#include "motor_calibration.h"
#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "websocket_server.h"

// ============================================
// Calibration State Machine
// ============================================

enum CalState {
  CAL_IDLE,
  CAL_SETTLING,    // waiting between steps for motors to stop
  CAL_RAMPING,     // motors started, waiting for slew rate limiter to reach target
  CAL_MEASURING,   // steady-state measurement period
  CAL_COMPLETE
};

static const int pwmLevels[MOTOR_CAL_NUM_LEVELS] = {100, 160, 220};

// Total steps = NUM_LEVELS * 2 (forward + reverse per level)
#define MOTOR_CAL_TOTAL_STEPS (MOTOR_CAL_NUM_LEVELS * 2)

// Safety timeout: abort calibration if the entire routine exceeds this
// (e.g. a stalled motor prevents progress through the state machine).
#define MOTOR_CAL_TIMEOUT_MS 60000  // 60 seconds

static CalState calState = CAL_IDLE;
static int calStep = 0;           // 0..TOTAL_STEPS-1
static uint32_t calStepStart = 0;
static uint32_t calGlobalStart = 0;  // start time of entire calibration
static int32_t calStartCounts[4];
static int32_t calAccumFwd[4];    // total absolute encoder counts across forward steps
static int32_t calAccumRev[4];    // total absolute encoder counts across reverse steps
static int calTargetPwm = 0;      // current step's target PWM

static void sendProgress(int step, int total, const char* desc) {
  wsLog("Motor cal: step %d/%d — %s", step + 1, total, desc);
}

// Declared in websocket_server.cpp — broadcasts typed JSON to all WS clients
extern void wsBroadcastRaw(const char* json);

static void sendResult(bool success, const float fwd[4], const float rev[4]) {
  char buf[384];
  int n = snprintf(buf, sizeof(buf),
    "{\"type\":\"motor_cal_result\",\"success\":%s,"
    "\"gainsFwd\":[%.4f,%.4f,%.4f,%.4f],"
    "\"gainsRev\":[%.4f,%.4f,%.4f,%.4f]}",
    success ? "true" : "false",
    fwd[0], fwd[1], fwd[2], fwd[3],
    rev[0], rev[1], rev[2], rev[3]);
  if (n > 0 && n < (int)sizeof(buf)) {
    wsBroadcastRaw(buf);
  }
}

// ============================================
// Public Functions
// ============================================

void startMotorCalibration() {
  if (calState != CAL_IDLE) {
    wsLog("Motor cal already running");
    return;
  }

  wsLog("Motor calibration starting...");

  // Reset gains to 1.0 during calibration for unbiased measurement
  float unity[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  setMotorGains(unity, unity);

  // Reset accumulators
  for (int i = 0; i < 4; i++) {
    calAccumFwd[i] = 0;
    calAccumRev[i] = 0;
  }

  calStep = 0;
  calState = CAL_SETTLING;
  calStepStart = millis();
  calGlobalStart = millis();

  // Stop motors and reset encoders
  stopAllMotors();
  resetAllEncoders();

  sendProgress(0, MOTOR_CAL_TOTAL_STEPS, "preparing...");
}

bool isMotorCalibrationRunning() {
  return calState != CAL_IDLE;
}

void abortMotorCalibration() {
  if (calState == CAL_IDLE) return;
  stopAllMotors();
  calState = CAL_IDLE;
  calStep = 0;
  wsLog("Motor cal aborted");
}

void updateMotorCalibration() {
  if (calState == CAL_IDLE) return;

  uint32_t now = millis();
  uint32_t elapsed = now - calStepStart;

  // Global safety timeout — abort if the entire routine takes too long
  if ((now - calGlobalStart) > MOTOR_CAL_TIMEOUT_MS) {
    stopAllMotors();
    calState = CAL_IDLE;
    calStep = 0;
    float unity[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    sendResult(false, unity, unity);
    wsLog("Motor cal ABORTED: safety timeout (%ds)", MOTOR_CAL_TIMEOUT_MS / 1000);
    return;
  }

  switch (calState) {
    case CAL_SETTLING:
      if (elapsed >= MOTOR_CAL_SETTLE_MS) {
        // Determine PWM for this step
        int levelIdx = calStep / 2;
        bool forward = (calStep % 2) == 0;
        calTargetPwm = forward ? pwmLevels[levelIdx] : -pwmLevels[levelIdx];

        // Start all 4 motors (slew rate limiter will ramp up)
        for (int i = 0; i < 4; i++) {
          setMotorSpeed(i, calTargetPwm);
        }

        char desc[48];
        snprintf(desc, sizeof(desc), "PWM %d %s (ramping)",
                 pwmLevels[levelIdx], forward ? "fwd" : "rev");
        sendProgress(calStep, MOTOR_CAL_TOTAL_STEPS, desc);

        calState = CAL_RAMPING;
        calStepStart = now;
      }
      break;

    case CAL_RAMPING:
      // Keep driving motors so slew rate limiter ramps to target
      for (int i = 0; i < 4; i++) {
        setMotorSpeed(i, calTargetPwm);
      }

      if (elapsed >= MOTOR_CAL_RAMP_MS) {
        // Ramp complete — record starting encoder counts for clean measurement
        for (int i = 0; i < 4; i++) {
          calStartCounts[i] = getEncoderCount(i);
        }
        calState = CAL_MEASURING;
        calStepStart = now;
      }
      break;

    case CAL_MEASURING:
      // Keep driving motors at target PWM
      for (int i = 0; i < 4; i++) {
        setMotorSpeed(i, calTargetPwm);
      }

      if (elapsed >= MOTOR_CAL_STEP_DURATION_MS) {
        // Record encoder counts — apply motor direction correction so
        // forward/reverse bucketing uses the same convention as PID and
        // readEncoders() (positive = physical forward after correction).
        // Order matches the canonical internal index order [L1, R1, L2, R2].
        static const int8_t motorDirs[4] = { MOTOR_L1_DIR, MOTOR_R1_DIR, MOTOR_L2_DIR, MOTOR_R2_DIR };
        for (int i = 0; i < 4; i++) {
          int32_t delta = (getEncoderCount(i) - calStartCounts[i]) * (-motorDirs[i]);
          if (delta >= 0) {
            calAccumFwd[i] += delta;
          } else {
            calAccumRev[i] += (-delta);
          }
        }

        // Stop motors
        stopAllMotors();

        calStep++;

        if (calStep >= MOTOR_CAL_TOTAL_STEPS) {
          calState = CAL_COMPLETE;
        } else {
          calState = CAL_SETTLING;
          calStepStart = now;
        }
      }
      break;

    case CAL_COMPLETE: {
      // Compute per-motor per-direction gains normalized to group mean
      float gainsFwd[4], gainsRev[4];

      float fwdSum = 0, revSum = 0;
      for (int i = 0; i < 4; i++) {
        fwdSum += (float)calAccumFwd[i];
        revSum += (float)calAccumRev[i];
      }
      float fwdMean = fwdSum / 4.0f;
      float revMean = revSum / 4.0f;

      // Require every wheel to have moved meaningfully — a passing group
      // mean with one stalled wheel would otherwise divide by zero and
      // poison that wheel's gain with Inf/NaN.
      const int32_t MIN_PER_WHEEL_COUNTS = 50;
      bool allWheelsMoved = true;
      for (int i = 0; i < 4; i++) {
        if (calAccumFwd[i] < MIN_PER_WHEEL_COUNTS || calAccumRev[i] < MIN_PER_WHEEL_COUNTS) {
          allWheelsMoved = false;
          break;
        }
      }
      bool success = allWheelsMoved && (fwdMean > 100.0f) && (revMean > 100.0f);
      if (success) {
        for (int i = 0; i < 4; i++) {
          gainsFwd[i] = fwdMean / (float)calAccumFwd[i];
          gainsRev[i] = revMean / (float)calAccumRev[i];
        }
        setMotorGains(gainsFwd, gainsRev);
        wsLog("Motor cal complete: fwd [%.4f, %.4f, %.4f, %.4f] rev [%.4f, %.4f, %.4f, %.4f]",
              gainsFwd[0], gainsFwd[1], gainsFwd[2], gainsFwd[3],
              gainsRev[0], gainsRev[1], gainsRev[2], gainsRev[3]);
      } else {
        for (int i = 0; i < 4; i++) { gainsFwd[i] = 1.0f; gainsRev[i] = 1.0f; }
        if (!allWheelsMoved) {
          wsLog("Motor cal FAILED: at least one wheel stalled (fwd [%ld,%ld,%ld,%ld] rev [%ld,%ld,%ld,%ld])",
                (long)calAccumFwd[0], (long)calAccumFwd[1], (long)calAccumFwd[2], (long)calAccumFwd[3],
                (long)calAccumRev[0], (long)calAccumRev[1], (long)calAccumRev[2], (long)calAccumRev[3]);
        } else {
          wsLog("Motor cal FAILED: motors did not move");
        }
      }

      sendResult(success, gainsFwd, gainsRev);
      calState = CAL_IDLE;
      break;
    }

    default:
      break;
  }
}
