#include "openloop_executor.h"
#include "motors.h"
#include "sensors.h"
#include "config.h"
#include "websocket_server.h"  // wsLog, wsBroadcastRaw
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

// ============================================
// Tier-0 open-loop segment executor
// ============================================

// Runtime-tuned watchdog margin. Real runs must come in under 1.3× the
// expected encoder counts or the watchdog aborts — see the header for
// the safety rationale.
static constexpr float OL_WATCHDOG_MARGIN = 1.3f;

// In-RAM cal. Populated at boot from NVS via openloopLoadCal().
static OpenLoopCal g_cal = {};

// Per-segment watchdog bookkeeping. Re-seeded on each segment
// transition by openloopResetWatchdog(); openloopDrive() tracks the
// running encoder-count sum against `watchdogThreshold`.
static int32_t  watchdogBaseline[4] = {0, 0, 0, 0};
static uint32_t watchdogThreshold   = 0;    // 1.3× expected counts
static int      watchdogLastSeg     = -1;   // Detect segment change
static bool     watchdogActive      = false;

static Preferences g_olPrefs;
static const char* NVS_NAMESPACE = "ol_cal";
// Single "version" key so future schema changes can be detected; if
// this byte changes on read, we treat NVS as empty and require a
// fresh calibration. Bump when the OpenLoopCal struct layout changes.
static const char* NVS_KEY_VERSION = "v";
static constexpr uint8_t OL_CAL_VERSION = 1;
static const char* NVS_KEY_BASE_PWM  = "pwm";
// Per-direction speed keys — short names to stay well under the
// 15-char NVS key limit.
static const char* NVS_KEY_SPEEDS[OL_DIR_COUNT] = {
    "s_fwd", "s_back", "s_sl", "s_sr", "s_ccw", "s_cw",
};

// ============================================
// Calibration I/O
// ============================================

void openloopLoadCal() {
    g_cal = OpenLoopCal{};  // Zero-init; valid stays false unless NVS is present
    g_olPrefs.begin(NVS_NAMESPACE, true /* readOnly */);
    const uint8_t ver = g_olPrefs.getUChar(NVS_KEY_VERSION, 0);
    if (ver == OL_CAL_VERSION) {
        for (int i = 0; i < OL_DIR_COUNT; i++) {
            g_cal.speed[i] = g_olPrefs.getFloat(NVS_KEY_SPEEDS[i], 0.0f);
        }
        g_cal.basePwm = g_olPrefs.getUChar(NVS_KEY_BASE_PWM, 0);
        // Only mark valid if every direction has a non-zero speed and
        // PWM is non-zero — partial cal is unsafe (unknown duration).
        bool allSet = (g_cal.basePwm > 0);
        for (int i = 0; i < OL_DIR_COUNT && allSet; i++) {
            if (g_cal.speed[i] <= 0.0f || !isfinite(g_cal.speed[i])) allSet = false;
        }
        g_cal.valid = allSet;
    }
    g_olPrefs.end();
    if (g_cal.valid) {
        wsLog("Open-loop cal loaded: pwm=%u fwd=%.3f back=%.3f sL=%.3f sR=%.3f ccw=%.3f cw=%.3f",
              g_cal.basePwm,
              g_cal.speed[OL_DIR_FWD], g_cal.speed[OL_DIR_BACK],
              g_cal.speed[OL_DIR_STRAFE_L], g_cal.speed[OL_DIR_STRAFE_R],
              g_cal.speed[OL_DIR_YAW_CCW], g_cal.speed[OL_DIR_YAW_CW]);
    } else {
        wsLog("Open-loop cal not present — tier-0 trajectories will reject until calibrated");
    }
}

bool openloopSaveCal(const OpenLoopCal& cal) {
    // Refuse to persist a partial/invalid cal — an all-zero fallback on
    // next boot is safer than a half-populated table that "works" for
    // some directions and hangs the robot trying to compute a
    // zero-speed duration for others.
    if (cal.basePwm == 0) return false;
    for (int i = 0; i < OL_DIR_COUNT; i++) {
        if (cal.speed[i] <= 0.0f || !isfinite(cal.speed[i])) return false;
    }
    g_olPrefs.begin(NVS_NAMESPACE, false /* writable */);
    g_olPrefs.putUChar(NVS_KEY_VERSION, OL_CAL_VERSION);
    g_olPrefs.putUChar(NVS_KEY_BASE_PWM, cal.basePwm);
    for (int i = 0; i < OL_DIR_COUNT; i++) {
        g_olPrefs.putFloat(NVS_KEY_SPEEDS[i], cal.speed[i]);
    }
    g_olPrefs.end();
    g_cal = cal;
    g_cal.valid = true;
    wsLog("Open-loop cal saved (pwm=%u)", cal.basePwm);
    return true;
}

const OpenLoopCal& openloopGetCal() {
    return g_cal;
}

// ============================================
// Segment classification
// ============================================

// Approximate-zero test tolerance. Below this the velocity component
// is treated as zero — tier 0 only does cardinal motions, so a
// commanded translate with e.g. vy = 1e-4 m/s is still "forward".
static constexpr float OL_CARDINAL_EPS = 1e-3f;

OpenLoopDirection openloopClassifySegment(const TrajectorySegment& seg) {
    switch (seg.kind) {
        case SEG_TRANSLATE: {
            const float vx = seg.translate.vx;
            const float vy = seg.translate.vy;
            const bool vxNonzero = fabsf(vx) > OL_CARDINAL_EPS;
            const bool vyNonzero = fabsf(vy) > OL_CARDINAL_EPS;
            if (vxNonzero && vyNonzero) return OL_DIR_INVALID;  // diagonal
            if (vxNonzero) return (vx > 0) ? OL_DIR_FWD : OL_DIR_BACK;
            if (vyNonzero) return (vy > 0) ? OL_DIR_STRAFE_L : OL_DIR_STRAFE_R;
            return OL_DIR_INVALID;  // zero-velocity translate
        }
        case SEG_YAW: {
            const float angle = seg.yaw.angle;
            if (fabsf(angle) < OL_CARDINAL_EPS) return OL_DIR_INVALID;
            return (angle > 0) ? OL_DIR_YAW_CCW : OL_DIR_YAW_CW;
        }
        case SEG_STRAFE_CIRCLE:
            // Tier 0 cannot decompose a continuously-rotating velocity
            // vector into cardinal motions. Reject at load time — the
            // thesis narrative documents this as a deliberate
            // limitation of open-loop control without kinematics.
            return OL_DIR_INVALID;
        case SEG_PAUSE:
        default:
            return OL_DIR_INVALID;
    }
}

float openloopSegmentDurationMs(const TrajectorySegment& seg) {
    if (seg.kind == SEG_PAUSE) return 0.0f;

    const OpenLoopDirection dir = openloopClassifySegment(seg);
    if (dir == OL_DIR_INVALID) return -1.0f;
    if (!g_cal.valid) return -1.0f;
    const float calSpeed = g_cal.speed[dir];
    if (calSpeed <= 0.0f) return -1.0f;

    switch (seg.kind) {
        case SEG_TRANSLATE:
            // distance (m) / speed (m/s) * 1000 → ms
            return (seg.translate.distance / calSpeed) * 1000.0f;
        case SEG_YAW:
            // |angle (rad)| / speed (rad/s) * 1000 → ms
            return (fabsf(seg.yaw.angle) / calSpeed) * 1000.0f;
        default:
            return -1.0f;
    }
}

uint32_t openloopSegmentExpectedCounts(const TrajectorySegment& seg) {
    if (seg.kind == SEG_PAUSE) return 0;

    // Translation: each wheel traverses ~distance meters. Counts per
    // meter from config: 1 / METERS_PER_COUNT. Summed across 4 wheels.
    if (seg.kind == SEG_TRANSLATE) {
        const float countsPerMeter = 1.0f / METERS_PER_COUNT;
        return (uint32_t)(4.0f * seg.translate.distance * countsPerMeter);
    }

    // Yaw: each wheel traces an arc of radius L_SUM around the chassis
    // center. Arc length per wheel = angle * L_SUM (radians × meters).
    // Sum across 4 wheels = 4 * angle * L_SUM meters.
    if (seg.kind == SEG_YAW) {
        const float countsPerMeter = 1.0f / METERS_PER_COUNT;
        const float perWheelMeters = fabsf(seg.yaw.angle) * L_SUM;
        return (uint32_t)(4.0f * perWheelMeters * countsPerMeter);
    }

    return 0;
}

// ============================================
// Runtime driver
// ============================================

void openloopResetWatchdog() {
    // Re-seed the baseline from whatever encoders read right now, and
    // recompute the threshold from the segment we've just entered.
    const int seg = trajGetCurrentSeg();
    const TrajectorySegment* s = trajGetSegment(seg);
    for (int i = 0; i < 4; i++) watchdogBaseline[i] = getEncoderCount(i);
    watchdogLastSeg = seg;
    if (s == nullptr || s->kind == SEG_PAUSE) {
        watchdogThreshold = 0;
        watchdogActive = false;
    } else {
        const uint32_t expected = openloopSegmentExpectedCounts(*s);
        watchdogThreshold = (uint32_t)(expected * OL_WATCHDOG_MARGIN);
        watchdogActive = (watchdogThreshold > 0);
    }
}

// Sum of |enc_delta| across the four wheels since the last reset.
// Treating direction as irrelevant — any motion beyond the threshold
// is bad, regardless of sign.
static uint32_t summedEncoderTravel() {
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) {
        const int32_t delta = getEncoderCount(i) - watchdogBaseline[i];
        sum += (uint32_t)(delta >= 0 ? delta : -delta);
    }
    return sum;
}

// Emit a compact JSON error + abort the trajectory. Keeps the message
// shape consistent with trajectory.cpp's other broadcasts.
static void tripWatchdog(uint32_t actual) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"traj_aborted\",\"runId\":\"%s\","
        "\"reason\":\"encoder_watchdog\","
        "\"seg\":%d,\"counts\":%lu,\"threshold\":%lu}",
        trajGetRunId(), trajGetCurrentSeg(),
        (unsigned long)actual, (unsigned long)watchdogThreshold);
    if (n > 0 && n < (int)sizeof(buf)) wsBroadcastRaw(buf);
    wsLog("Tier-0 watchdog: seg %d travelled %lu counts (limit %lu) — aborting",
          trajGetCurrentSeg(), (unsigned long)actual, (unsigned long)watchdogThreshold);
    trajAbort();
    stopAllMotors();
    watchdogActive = false;
}

void openloopDrive() {
    const TrajectoryState st = trajGetState();
    if (st != TRAJ_RUNNING && st != TRAJ_PAUSED) {
        watchdogActive = false;
        return;
    }

    // Detect segment transition. trajectory.cpp is the source of truth
    // for currentSeg; it advances based on elapsedMs vs durationMs at
    // each tick. On any change, reseed the watchdog.
    const int seg = trajGetCurrentSeg();
    if (seg != watchdogLastSeg) {
        openloopResetWatchdog();
    }

    // Paused → hold zero. Watchdog is idle during pauses since the
    // robot is supposed to be still.
    if (st == TRAJ_PAUSED) {
        stopAllMotors();
        return;
    }

    const TrajectorySegment* s = trajGetSegment(seg);
    if (s == nullptr || s->kind == SEG_PAUSE) {
        stopAllMotors();
        return;
    }

    // Drive motors from the hardcoded direction table.
    const OpenLoopDirection dir = openloopClassifySegment(*s);
    if (dir == OL_DIR_INVALID || !g_cal.valid) {
        // Shouldn't happen — load-time validation rejects these — but
        // belt-and-braces if we somehow get here, stop cleanly.
        stopAllMotors();
        return;
    }
    const int8_t* signs = OL_MOTOR_SIGNS[dir];
    const int pwm = (int)g_cal.basePwm;
    for (int i = 0; i < 4; i++) {
        setMotorSpeed(i, signs[i] * pwm);
    }

    // Watchdog: abort if the robot has travelled > 1.3× expected.
    if (watchdogActive) {
        const uint32_t travel = summedEncoderTravel();
        if (travel > watchdogThreshold) {
            tripWatchdog(travel);
        }
    }
}
