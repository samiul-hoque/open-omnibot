#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Arduino.h>
#include "mecanum.h"

// ============================================
// Constants
// ============================================

// Bumped from 16 to 64 so a full multi-waypoint trajectory (motion
// segments + SEG_PAUSE markers) fits in a single upload. Worst case
// today is circle_0_5m_strafe: 32 translate + 4 pause = 36.
#define MAX_TRAJECTORY_SEGMENTS 64
#define TRAJ_RUNID_MAX 128
#define TRAJ_PROGRESS_INTERVAL_MS 500

// Settle window after entering a pause before `traj_paused` fires, so
// the overhead camera snapshot sees a mechanically-still robot (motors
// already commanded to zero; this buys the chassis time to actually
// stop).
#define TRAJ_PAUSE_SETTLE_MS 300

// ============================================
// Segment Types
// ============================================

enum SegmentKind : uint8_t {
    SEG_TRANSLATE = 0,       // Constant-velocity translation
    SEG_YAW = 1,             // In-place rotation
    SEG_STRAFE_CIRCLE = 2,   // Fixed-heading circular strafe
    SEG_PAUSE = 3,           // Hold position, wait for explicit resume
};

struct TrajectorySegment {
    SegmentKind kind;
    float durationMs;        // Precomputed at load time (0 for SEG_PAUSE)
    union {
        struct { float vx; float vy; float distance; } translate;
        struct { float w; float angle; } yaw;
        struct { float speed; float radius; } circle;
        // SEG_PAUSE carries no payload — the server correlates pause
        // segment index back to a waypoint label on its side.
    };
};

// ============================================
// State Machine
// ============================================

enum TrajectoryState : uint8_t {
    TRAJ_IDLE = 0,
    TRAJ_ARMED,
    TRAJ_RUNNING,
    TRAJ_PAUSED,             // Halted at a SEG_PAUSE awaiting trajResume()
    TRAJ_COMPLETED,
};

// Execution mode for a loaded trajectory. Closed-loop runs the normal
// PID + IK + (optional) heading-hold path; open-loop bypasses those
// in favor of a fixed 6-direction PWM table (see openloop_executor.h)
// as a tier-0 baseline for the thesis comparison.
enum TrajMode : uint8_t {
    TRAJ_MODE_CLOSED_LOOP = 0,
    TRAJ_MODE_OPEN_LOOP   = 1,
};

// ============================================
// Public API
// ============================================

// Load a trajectory definition. Precomputes segment durations using
// the appropriate source:
//   - CLOSED_LOOP: durations from commanded speeds in each segment
//   - OPEN_LOOP:   durations from the NVS-stored calibrated speeds
//                  for each of the 6 cardinal directions
// Returns true on success, false if invalid (too many segments,
// non-positive durations, or — in open-loop mode — a segment whose
// direction isn't one of the 6 tier-0 cardinals, or a direction
// whose calibration is missing).
bool trajLoad(const char* runId, TrajectorySegment* segs, int count,
              TrajMode mode = TRAJ_MODE_CLOSED_LOOP);

// Arm the trajectory (prepare for start). Call after load.
void trajArm();

// Start execution. Transitions ARMED → RUNNING.
void trajStart();

// Resume from a pause. Transitions PAUSED → RUNNING and advances past
// the pause segment. No-op from any other state.
void trajResume();

// Abort from any non-idle state. Transitions → IDLE.
void trajAbort();

// Called at 50Hz from the motor loop. Returns body-frame velocity command.
// Returns {0,0,0} when not running.
VelocityCommand trajTick();

// Get current state.
TrajectoryState trajGetState();

// Get progress info for telemetry.
void trajGetProgress(int* segIdx, float* elapsedMs, float* totalMs);

// Get the stored runId (for telemetry messages).
const char* trajGetRunId();

// Get the mode (closed-loop vs open-loop) of the currently loaded
// trajectory. Returns the mode used in the most recent successful
// trajLoad() — defaults to CLOSED_LOOP before the first load.
TrajMode trajGetMode();

// Get the current segment index (0-based). Returns -1 when idle.
// Used by the open-loop executor to detect segment transitions and
// reset its per-segment watchdog state.
int trajGetCurrentSeg();

// Read-only view of a loaded segment. Returns nullptr if idx is out
// of range.
const TrajectorySegment* trajGetSegment(int idx);

#endif // TRAJECTORY_H
