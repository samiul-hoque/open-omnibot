#ifndef OPENLOOP_EXECUTOR_H
#define OPENLOOP_EXECUTOR_H

#include <stdint.h>
#include "trajectory.h"  // TrajectorySegment, SegmentKind
#include "openloop_motor_table.h"  // OpenLoopDirection

// ============================================
// Tier-0 Open-Loop Segment Executor
// ============================================
//
// This module drives the motors directly from a 6-row PWM direction
// table, bypassing PID and IK. Activated per-trajectory via
// trajLoad(..., TRAJ_MODE_OPEN_LOOP).
//
// The high-level trajectory state machine (ARMED → RUNNING → PAUSED
// → COMPLETED) still lives in trajectory.cpp; segment durations in
// open-loop mode come from this module's calibration table, not from
// the commanded speed. The executor's `openloopDrive()` is called
// every tick during RUNNING/PAUSED and writes motor PWMs directly.
//
// Three safety layers guard against open-loop runaway:
//   1. Per-segment encoder distance watchdog (>1.3× expected → abort)
//   2. Per-waypoint workspace bounds check on the server side
//   3. Standard E-STOP / client disconnect / velocity timeout from the
//      existing firmware (motors are auto-stopped on loss of command)

// ============================================
// Calibration
// ============================================

// Per-direction calibrated speeds, persisted to NVS namespace "ol_cal"
// via Preferences.
//
// Units: meters/second for translate directions (FWD, BACK, STRAFE_*),
// radians/second for yaw directions (YAW_CCW, YAW_CW).
//
// Base PWM magnitude is the ONE PWM value used for every direction.
// Kept as a single value for simplicity — if individual-direction PWM
// tuning is ever needed, split into 6 fields.
struct OpenLoopCal {
    float speed[OL_DIR_COUNT];  // indexed by OpenLoopDirection
    uint8_t basePwm;            // magnitude [0, 255]
    bool    valid;              // false if NVS was empty / unloaded
};

// Load calibration from NVS. Safe to call at boot even before WiFi is
// up. If NVS has no saved cal, fills the struct with zeros and leaves
// `valid = false`.
void openloopLoadCal();

// Persist a fresh calibration to NVS. Returns true on success.
bool openloopSaveCal(const OpenLoopCal& cal);

// Read-only view of the current in-RAM cal.
const OpenLoopCal& openloopGetCal();

// ============================================
// Segment dispatch helpers (used at load time)
// ============================================

// Classify a trajectory segment into one of the 6 cardinal directions.
// Returns OL_DIR_INVALID if the segment is unsupported by tier 0
// (diagonal strafe, strafe_circle, zero-velocity translate, etc).
// Pause segments return OL_DIR_INVALID — callers handle pauses
// separately.
OpenLoopDirection openloopClassifySegment(const TrajectorySegment& seg);

// Compute segment duration in milliseconds using the calibrated
// per-direction speed. Returns 0 for pause segments. Returns -1.0f if
// the segment is unsupported or its direction's cal speed is zero
// (signals reject-at-load to the caller).
float openloopSegmentDurationMs(const TrajectorySegment& seg);

// Compute the expected encoder-count threshold for this segment,
// summed across all four wheels. Used by the runtime watchdog —
// actual encoder travel that exceeds 1.3× this number triggers an
// abort. Returns 0 for pauses.
uint32_t openloopSegmentExpectedCounts(const TrajectorySegment& seg);

// ============================================
// Runtime driver
// ============================================

// Must be called from main.cpp's motor-update tick at 50 Hz whenever
// the trajectory state machine is in RUNNING or PAUSED and the
// current trajectory's mode is TRAJ_MODE_OPEN_LOOP. Handles:
//   - setting motor PWMs from the table + base magnitude
//   - stopping motors on PAUSED segments
//   - advancing the watchdog for the current segment
//   - firing trajAbort() + broadcasting an error on watchdog trip
//
// Reads current segment + state directly from trajectory.cpp getters;
// has no state of its own beyond per-segment watchdog bookkeeping.
void openloopDrive();

// Called by trajectory.cpp on segment transitions (START and RESUME)
// so the watchdog baseline resets to the current encoder position.
void openloopResetWatchdog();

#endif // OPENLOOP_EXECUTOR_H
