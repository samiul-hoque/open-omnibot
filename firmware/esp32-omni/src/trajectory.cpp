#include "trajectory.h"
#include "odometry.h"
#include "pid_controller.h"
#include "websocket_server.h"
#include "openloop_executor.h"
#include <math.h>

// ============================================
// Internal state
// ============================================

static TrajectorySegment segments[MAX_TRAJECTORY_SEGMENTS];
static int segCount = 0;
static char runId[TRAJ_RUNID_MAX] = {0};
static TrajMode trajMode = TRAJ_MODE_CLOSED_LOOP;

static TrajectoryState state = TRAJ_IDLE;
static int currentSeg = 0;
static uint32_t segmentStartMs = 0;
static uint32_t trajectoryStartMs = 0;
static float totalDurationMs = 0;

// Progress broadcast throttle
static uint32_t lastProgressMs = 0;

// Pause bookkeeping: when we transition into a SEG_PAUSE we record the
// enter time and hold off the `traj_paused` broadcast by
// TRAJ_PAUSE_SETTLE_MS so the robot has physically come to rest before
// the server triggers a snapshot. pausedBroadcast guards against
// re-broadcasting every tick while paused.
static uint32_t pauseEnteredMs = 0;
static bool pausedBroadcast = false;

// ============================================
// Helpers
// ============================================

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
        case SEG_PAUSE:
            // Pauses have no time budget — they hold until trajResume().
            return 0;
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
            cmd.vx = 0;
            cmd.vy = 0;
            cmd.omega = sign * fabsf(seg.yaw.w);
            break;
        }

        case SEG_STRAFE_CIRCLE: {
            float omega = seg.circle.speed / seg.circle.radius;
            float theta = omega * (elapsedMs / 1000.0f);
            cmd.vx = seg.circle.speed * sinf(theta);
            cmd.vy = seg.circle.speed * cosf(theta);
            cmd.omega = 0;
            break;
        }

        case SEG_PAUSE:
        default:
            // Pause holds zero. Other unexpected kinds also fall safe.
            break;
    }

    return cmd;
}

// Broadcast trajectory completion with final pose
static void broadcastDone() {
    Pose pose = odomGetPose();
    static char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"traj_done\",\"runId\":\"%s\","
        "\"pose\":{\"x\":%.4f,\"y\":%.4f,\"th\":%.4f}}",
        runId, pose.x, pose.y, pose.theta);
    if (n > 0 && n < (int)sizeof(buf)) {
        wsBroadcastRaw(buf);
    }
}

// Broadcast that the trajectory has paused at a waypoint segment. Sent
// once per pause after the settle window elapses. The server uses
// `seg` to look up which waypoint label this pause corresponds to.
static void broadcastPaused(int segIdx) {
    Pose pose = odomGetPose();
    static char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"traj_paused\",\"runId\":\"%s\","
        "\"seg\":%d,"
        "\"pose\":{\"x\":%.4f,\"y\":%.4f,\"th\":%.4f}}",
        runId, segIdx, pose.x, pose.y, pose.theta);
    if (n > 0 && n < (int)sizeof(buf)) {
        wsBroadcastRaw(buf);
    }
}

// Broadcast progress update
static void broadcastProgress() {
    float elapsed = (float)(millis() - trajectoryStartMs);
    static char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"traj_progress\",\"runId\":\"%s\","
        "\"seg\":%d,\"elapsed\":%.0f,\"total\":%.0f}",
        runId, currentSeg, elapsed, totalDurationMs);
    if (n > 0 && n < (int)sizeof(buf)) {
        wsBroadcastRaw(buf);
    }
}

// ============================================
// Public API
// ============================================

bool trajLoad(const char* id, TrajectorySegment* segs, int count, TrajMode mode) {
    if (count <= 0 || count > MAX_TRAJECTORY_SEGMENTS) return false;
    if (state == TRAJ_RUNNING || state == TRAJ_PAUSED) return false;  // Can't load while active

    segCount = count;
    totalDurationMs = 0;
    trajMode = mode;

    for (int i = 0; i < count; i++) {
        segments[i] = segs[i];
        // Duration source differs between modes:
        //   closed-loop: distance / commanded_speed (conventional)
        //   open-loop:   distance / calibrated_speed for this direction
        //                — also rejects unsupported directions (diagonal
        //                strafe, strafe_circle) and any segment whose
        //                calibrated speed is zero/missing.
        float dur;
        if (mode == TRAJ_MODE_OPEN_LOOP) {
            dur = openloopSegmentDurationMs(segments[i]);
            // SEG_PAUSE returns 0 (valid); anything else returning <0
            // means unsupported direction or missing cal → reject
            // the whole load rather than silently skip.
            if (segments[i].kind != SEG_PAUSE && dur < 0) {
                return false;
            }
        } else {
            dur = computeSegmentDuration(segments[i]);
            if (segments[i].kind != SEG_PAUSE && dur <= 0) {
                return false;
            }
        }
        segments[i].durationMs = dur;
        totalDurationMs += dur;
    }

    strncpy(runId, id, TRAJ_RUNID_MAX - 1);
    runId[TRAJ_RUNID_MAX - 1] = '\0';

    return true;
}

void trajArm() {
    if (segCount == 0) return;
    state = TRAJ_ARMED;
    currentSeg = 0;
    wsLog("Trajectory armed: %s (%.1fs, %d segments)", runId,
          totalDurationMs / 1000.0f, segCount);
}

void trajStart() {
    if (state != TRAJ_ARMED) return;

    state = TRAJ_RUNNING;
    currentSeg = 0;
    trajectoryStartMs = millis();
    segmentStartMs = trajectoryStartMs;
    lastProgressMs = 0;
    pausedBroadcast = false;

    // Wipe heading-hold LPF so we don't consume stale gyro bias from
    // teleop or a previous trajectory into the first translate ticks.
    // Closed-loop only — open-loop doesn't touch the HH filter since
    // it bypasses the commanded-velocity → IK path entirely.
    if (trajMode == TRAJ_MODE_CLOSED_LOOP) {
        resetHeadingHoldFilter();
    } else {
        // Open-loop: seed the encoder watchdog baseline at the current
        // (post-reset) encoder counts. Avoids a spurious trip on the
        // very first tick if encoder counters drift slightly between
        // load_trajectory (which resets them) and traj_start.
        openloopResetWatchdog();
    }

    wsLog("Trajectory started: %s (%s)", runId,
          trajMode == TRAJ_MODE_OPEN_LOOP ? "open-loop" : "closed-loop");
}

void trajResume() {
    if (state != TRAJ_PAUSED) {
        wsLog("trajResume: ignored (state=%d)", (int)state);
        return;
    }

    // Advance past the pause segment.
    currentSeg++;
    if (currentSeg >= segCount) {
        // Pause was the trailing segment — treat as completion.
        state = TRAJ_COMPLETED;
        wsLog("Trajectory completed (trailing pause): %s", runId);
        broadcastDone();
        return;
    }

    // Closed-loop needs PID + heading-hold resets; open-loop bypasses
    // both of those paths entirely.
    if (trajMode == TRAJ_MODE_CLOSED_LOOP) {
        // Wipe PID integrator state so the next segment starts from a
        // clean integrator — otherwise the accumulated error from the
        // last segment would pile onto the new one and produce a
        // surge at resume.
        resetPIDControllers();

        // Wipe heading-hold LPF too: the robot sat still during the
        // pause (PAUSED commands zero), but any residual filter state
        // from the pre-pause motion would bleed into the post-pause
        // translate ticks.
        resetHeadingHoldFilter();
    } else {
        // Open-loop: reseed the encoder watchdog baseline for the new
        // segment. Without this, counts accumulated during the
        // pre-pause segment plus the settle period would trip the
        // watchdog on the first tick after resume.
        openloopResetWatchdog();
    }

    // Re-anchor segment timing to the resume moment so the freshly
    // started segment runs for its full duration regardless of how long
    // the snapshot took.
    segmentStartMs = millis();
    lastProgressMs = 0;
    pausedBroadcast = false;
    state = TRAJ_RUNNING;

    wsLog("Trajectory resumed: %s (seg %d)", runId, currentSeg);
}

void trajAbort() {
    if (state == TRAJ_IDLE) return;

    TrajectoryState prev = state;
    state = TRAJ_IDLE;
    pausedBroadcast = false;

    const char* prevName =
        prev == TRAJ_RUNNING ? "running" :
        prev == TRAJ_PAUSED ? "paused" :
        prev == TRAJ_ARMED ? "armed" : "completed";
    wsLog("Trajectory aborted (was %s): %s", prevName, runId);
}

VelocityCommand trajTick() {
    VelocityCommand zero = {0, 0, 0};

    // Paused path: hold zero, emit `traj_paused` once the settle window
    // has elapsed so the snapshot service captures a still robot.
    if (state == TRAJ_PAUSED) {
        if (!pausedBroadcast &&
            (millis() - pauseEnteredMs) >= TRAJ_PAUSE_SETTLE_MS) {
            pausedBroadcast = true;
            broadcastPaused(currentSeg);
        }
        return zero;
    }

    if (state != TRAJ_RUNNING) return zero;

    uint32_t now = millis();

    // Degenerate case: the very first tick of trajStart lands on a
    // SEG_PAUSE (e.g. a trajectory that starts with a pause — not used
    // today but keeps the invariant clean).
    if (segments[currentSeg].kind == SEG_PAUSE) {
        state = TRAJ_PAUSED;
        pauseEnteredMs = now;
        pausedBroadcast = false;
        return zero;
    }

    float elapsed = (float)(now - segmentStartMs);

    // Advance segments if current one is complete. On entering a
    // SEG_PAUSE, flip state and return — the pause broadcast is
    // deferred until the settle window elapses on subsequent ticks.
    while (elapsed >= segments[currentSeg].durationMs) {
        elapsed -= segments[currentSeg].durationMs;
        segmentStartMs += (uint32_t)segments[currentSeg].durationMs;
        currentSeg++;

        if (currentSeg >= segCount) {
            state = TRAJ_COMPLETED;
            wsLog("Trajectory completed: %s", runId);
            broadcastDone();
            return zero;
        }

        if (segments[currentSeg].kind == SEG_PAUSE) {
            state = TRAJ_PAUSED;
            pauseEnteredMs = now;
            pausedBroadcast = false;
            return zero;
        }

        // Mode-specific per-segment reset. Closed-loop wipes the PID
        // integrator (see the bias-carry-over rationale below);
        // open-loop reseeds the encoder watchdog baseline since the
        // expected-count threshold changes with the new segment.
        if (trajMode == TRAJ_MODE_CLOSED_LOOP) {
            // Wipe PID integrator state when crossing into a new motion
            // segment. Back-to-back yaw→translate or translate→yaw
            // transitions reverse 2 of 4 wheels, and the prior
            // segment's steady-state bias becomes asymmetric
            // feedforward noise on the new segment (left side carries a
            // negative integrator, right side carries a positive one →
            // CCW yaw impulse for ~150–300 ms after each corner).
            // Pause→resume already resets in trajResume(); this covers
            // the in-chunk boundaries.
            resetPIDControllers();
        } else {
            // openloopDrive() also detects segment changes via
            // trajGetCurrentSeg() and resets on its own, but we call
            // it here too for immediate baseline capture — the drive
            // tick is 50 Hz and a very short segment (<20 ms) could
            // otherwise use stale baselines.
            openloopResetWatchdog();
        }
    }

    // Broadcast progress periodically
    if (now - lastProgressMs >= TRAJ_PROGRESS_INTERVAL_MS) {
        lastProgressMs = now;
        broadcastProgress();
    }

    return segmentVelocityAt(segments[currentSeg], elapsed);
}

TrajectoryState trajGetState() {
    return state;
}

void trajGetProgress(int* segIdx, float* elapsedMs, float* totalMs) {
    if (segIdx) *segIdx = currentSeg;
    if (elapsedMs) *elapsedMs = (state == TRAJ_RUNNING) ?
        (float)(millis() - trajectoryStartMs) : 0;
    if (totalMs) *totalMs = totalDurationMs;
}

const char* trajGetRunId() {
    return runId;
}

TrajMode trajGetMode() {
    return trajMode;
}

int trajGetCurrentSeg() {
    if (state == TRAJ_IDLE) return -1;
    return currentSeg;
}

const TrajectorySegment* trajGetSegment(int idx) {
    if (idx < 0 || idx >= segCount) return nullptr;
    return &segments[idx];
}
