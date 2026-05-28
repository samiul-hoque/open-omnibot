// ============================================
// TrajectoryRunner — scripted experiment execution
// ============================================
//
// Orchestrates a single experiment run end-to-end:
//   idle → armed → running → awaiting_ground_truth → idle
// plus an aborted terminal that writes the CSV/meta with an aborted flag
// and returns to idle.
//
// Execution is delegated to ESP32 firmware. The runner uploads the
// trajectory ONCE on arm (motion segments interleaved with explicit
// `pause` markers at each waypoint). Firmware drives the trajectory
// autonomously, halts at every pause, and emits `traj_paused`. The
// runner listens for each pause, triggers the overhead-camera snapshot,
// then calls `robot.trajResume()` to let the firmware continue. When
// firmware emits `traj_done`, the runner transitions to
// `awaiting_ground_truth`.
//
// This design fixes the two failure modes of the earlier chunk-by-chunk
// approach:
//   (1) Encoders and odometry are only reset once (on arm), instead of
//       once per chunk — so server-side pose accumulates cleanly across
//       the whole run.
//   (2) PID integrators + segment timing are managed inside firmware
//       across pause boundaries, eliminating the surge that the old
//       load_trajectory/trajStart-between-chunks path produced.
//
// Events (on the EventEmitter interface):
//   'armed'      { runId, config }
//   'started'    { runId, startedAt }
//   'tick'       { runId, phase, segmentIdx, elapsedMs, totalMs, ... }
//   'paused'     { runId, waypointIdx, label, segmentIdx }
//   'completed'  { runId, csvPath, durationMs }
//   'aborted'    { runId, reason, csvPath }
//   'stateChange'{ from, to }

import { EventEmitter } from 'node:events';

import { TRAJECTORIES, trajectoryDurationMs, stripPauses } from './trajectories.js';

// Armed but never started — auto-abort so we don't pin the state machine
// and an open CSV indefinitely.
const ARM_TIMEOUT_MS = 5 * 60 * 1000;

// GT never submitted after completion — release the logger.
const GT_TIMEOUT_MS = 10 * 60 * 1000;

// Safety net: if firmware goes silent (no `traj_paused` and no
// `traj_done`) for longer than the whole trajectory takes plus this
// grace, abort. Generous because real PID settling + mechanical lag
// can run noticeably beyond the nominal segment duration at each pause
// boundary.
const RUNNING_GRACE_MS = 30 * 1000;

// If the robot pauses but the runner never sees `traj_paused`+snapshot
// complete within this window, abort. Covers snapshot-service hangs
// and missed-packet scenarios.
const PAUSE_WATCHDOG_MS = 30 * 1000;

// Heading-hold settings used during runs. Deadzone/alpha match the
// firmware defaults used by the post-reprint baseline. Firmware
// default is OFF — we turn it on for each experiment and back off
// after, so teleop stays unaffected.
const HH_GAIN = 1.0;
const HH_DEADZONE = 0.01;
const HH_ALPHA = 0.3;

// After firmware fires `traj_paused` (which itself is delayed by
// TRAJ_PAUSE_SETTLE_MS on the firmware side for mechanical settle),
// wait this long before triggering the overhead-camera snapshot.
//
// This covers the phone camera's end-to-end CONTENT latency: the time
// between a scene physically occurring and the decoded frame appearing
// in our Python grabber. On this rig that's 1.5–2.5 s (phone sensor +
// ISP + H.264 encoder GOP + RTSP network + server FFmpeg decode), much
// larger than the steady-state 200–400 ms value we originally tried.
//
// The background grabber in aruco_detector.py already eliminates buffer
// backlog (frames-arrived-but-not-consumed) — it guarantees the latest
// arrived frame, but that frame's *scene* is still ~2 s behind reality.
// read_fresh() waits for a frame that ARRIVED after the request, which
// is not the same as a frame whose CONTENT is after the request.
//
// Measured 2026-04-19: after a 0.4 m / 4 s strafe segment, the wp08
// snapshot was taken 700 ms after pause entry (settle 300 + lead 400),
// but the frame's content showed the robot 16.5 cm short of its final
// at-rest pose — i.e., mid-segment, ~1.6 s of scene-time ago. Probe
// tests confirmed 1.8–2.5 s of content lag via the CamON timestamp
// watermark. See verify_tape.py / annotate_grid.py for the method.
//
// 2500 ms gives a comfortable margin over the measured upper bound.
// Cost: +2.1 s per waypoint, e.g. ~27 s on a 13-waypoint rotate.
const SNAPSHOT_LEAD_MS_DEFAULT = 2500;

// Workspace-bounds safety envelope for tier-0 open-loop runs. On every
// mid-run waypoint snapshot the runner checks the GT against these
// limits; any pose outside the envelope aborts the run before the
// firmware is told to resume. Matches the dashboard map's grid bounds
// (±1.35 × ±0.55 m) plus ~10 cm of margin so the abort fires while
// the marker is still unambiguously in-frame. Closed-loop tiers 1/2
// skip this check — they've been validated without it and adding
// abort surface late in the campaign is a regression risk.
const TIER0_BOUNDS_X_M = 1.45;
const TIER0_BOUNDS_Y_M = 0.65;

export const STATE = Object.freeze({
    idle: 'idle',
    armed: 'armed',
    running: 'running',
    awaitingGroundTruth: 'awaiting_ground_truth',
    aborted: 'aborted',
});

export class TrajectoryRunner extends EventEmitter {
    constructor({
        robot, localization, logger, setCommand, getTier,
        snapshotFn = null,       // async ({label, runId, trajectory}) → snapshot result
        getPose = null,          // () → current server-side pose, paired with each snapshot
        snapshotLeadMs = SNAPSHOT_LEAD_MS_DEFAULT,
        // Preflight placement validation. If both tolerances are null, preflight
        // is disabled (existing behaviour — tests rely on this). Otherwise, the
        // runner's preflight() method will snap the overhead camera, compare
        // against startHint, and return ok=false if the robot is too far from
        // the intended starting pose.
        preflightHeadingToleranceDeg = null,
        preflightPositionToleranceM = null,
        now = () => Date.now(),
    }) {
        super();
        this._robot = robot;
        this._localization = localization;
        this._logger = logger;
        this._setCommand = setCommand;
        this._getTier = getTier;
        this._snapshotFn = snapshotFn;
        this._getPose = getPose;
        this._snapshotLeadMs = Math.max(0, Number(snapshotLeadMs) || 0);
        this._preflightHeadingTol = preflightHeadingToleranceDeg;
        this._preflightPositionTol = preflightPositionToleranceM;
        this._now = now;

        this._state = STATE.idle;
        this._armTimer = null;
        this._gtTimer = null;
        this._runningTimer = null;
        this._pauseWatchdog = null;
        this._run = null;

        if (this._robot) {
            this._robot.on('traj_done', (msg) => this._onTrajDone(msg));
            this._robot.on('traj_paused', (msg) => this._onTrajPaused(msg));
            this._robot.on('traj_progress', (msg) => this._onTrajProgress(msg));
            this._robot.on('disconnected', () => {
                if (this._state === STATE.running) {
                    this.abort('robot_disconnect');
                }
            });
        }
    }

    getState() {
        return this._state;
    }

    getRun() {
        if (!this._run) return null;
        // Deep-copy so external callers (webServer broadcasts, unit
        // tests, future dashboards) cannot mutate nested structures
        // (startHint, groundTruth.waypoints, measuredStartPose) back
        // into the live run. The earlier shallow `{...this._run}`
        // leaked writes on nested objects — e.g. a test doing
        // `runner.getRun().startHint.headingDeg = -170` would mutate
        // the underlying _run, which in the wild could also happen
        // from an over-eager dashboard JSON.stringify path if it
        // decided to prune fields in place.
        //
        // We exclude the built trajectory and the segmentIdx→label
        // map because they contain non-serialisable / large data
        // that no external consumer needs. structuredClone is a
        // built-in on Node 17+ (we require Node 22 via npm).
        const { built: _built, labelBySegIdx: _lbl, ...serialisable } = this._run;
        return structuredClone(serialisable);
    }

    // --- Public API invoked by the WebSocket bridge ---

    arm({ trajectory, speed, rep = 1, operatorNotes = '', demo = false }) {
        if (this._state !== STATE.idle) {
            throw new Error(`cannot arm in state ${this._state}`);
        }
        if (!TRAJECTORIES[trajectory]) {
            throw new Error(`unknown trajectory: ${trajectory}`);
        }
        if (!Number.isFinite(speed) || speed <= 0) {
            throw new Error(`invalid speed: ${speed}`);
        }

        // Demo mode: strip mid-path pauses so the robot runs the trajectory
        // end-to-end without halting for snapshots. Preflight, per-waypoint
        // snapshots, logging, and the post-run GT state are all disabled
        // elsewhere in this class when `run.demo` is set.
        const rawBuilt = TRAJECTORIES[trajectory].build(speed);
        const built = demo ? stripPauses(rawBuilt) : rawBuilt;
        const tier = this._getTier();
        const armedAt = new Date(this._now()).toISOString();
        const runId = `exp_${trajectory}_${speed.toFixed(2)}_tier${tier}_rep${rep}_${armedAt.replace(/[:.]/g, '-').slice(0, 19)}`;

        // Build a segmentIdx → label lookup so we can attach the right
        // label when firmware emits `traj_paused` with a segment index.
        const labelBySegIdx = new Map();
        for (const wp of built.waypoints || []) {
            labelBySegIdx.set(wp.segmentIdx, wp.label);
        }

        this._run = {
            runId,
            trajectory,
            speed,
            tier,
            rep,
            operatorNotes,
            demo: demo === true,
            built,
            waypointDefs: built.waypoints || [],
            labelBySegIdx,
            totalMs: trajectoryDurationMs(built),
            armedAt,
            startedAt: null,
            endedAt: null,
            waypointsCompleted: 0,
            aborted: false,
            abortedReason: null,
            // Populated by preflight() on the OK path; stays null on
            // failure / skip / never-ran. Declared here so the run
            // object's shape is explicit and _buildMetaBlock's
            // `measuredStartPose: ... || null` coerces from null rather
            // than undefined.
            measuredStartPose: null,
            // Copy the trajectory's startHint onto the run so preflight()
            // and any other downstream code can validate placement without
            // re-resolving the trajectory catalog. Without this the
            // preflight Number.isFinite guards fail silently (startHint is
            // undefined), skip the tolerance checks, and pass through —
            // which is what bit us on 2026-04-19 (robot on circle tape got
            // cleared to run straight_2m).
            startHint: TRAJECTORIES[trajectory]?.startHint
                ? { ...TRAJECTORIES[trajectory].startHint }
                : null,
            // Appended as the firmware reports each `traj_paused`. Shape:
            //   { label, timestamp_ms, segmentIdx,
            //     gt: {x, y, theta_rad, theta_deg, side_px, image_path} | null,
            //     encoder: {x, y, theta_rad, theta_deg} | null,
            //     firmwarePose: {x, y, th} | null,
            //     error? }
            groundTruth: {
                method: 'camera_snapshot',
                waypoints: [],
            },
        };

        // Pre-run hygiene. Firmware's `load_trajectory` handler already
        // resets encoders/odometry/IMU on its side; here we clear the
        // server-side pose so the two align at zero on start.
        this._setCommand({ vx: 0, vy: 0, omega: 0 });
        if (this._robot?.stop) this._robot.stop();
        if (this._localization?.reset) this._localization.reset();

        // Single upload: the entire trajectory with pause markers baked in.
        // Tier 0 requests the firmware's open-loop executor — a fixed
        // 6-direction PWM table instead of PID + IK. Tiers 1/2 use the
        // default closed-loop path. The mode is baked into the current
        // tier at arm time; tier switching is already refused while an
        // experiment is non-idle, so this is stable for the whole run.
        const mode = (this._getTier && this._getTier() === 0) ? 'openloop' : undefined;
        if (this._robot?.loadTrajectory) {
            const sent = this._robot.loadTrajectory(runId, built.segments, mode);
            if (!sent) {
                this._run = null;
                throw new Error('Failed to send trajectory to robot (not connected)');
            }
        }

        // Restart logging with experiment metadata + dedicated filename prefix.
        // Demo mode writes no CSV / meta — the run is a visual demo, not data.
        if (this._logger && !this._run.demo) {
            if (this._logger.stop) this._logger.stop();
            if (this._logger.start) {
                this._logger.start({
                    prefix: runId,
                    experiment: this._buildMetaBlock('armed'),
                });
            }
            if (this._logger.logEvent) this._logger.logEvent('exp_arm');
        }

        this._transition(STATE.armed);
        this._armTimer = setTimeout(() => {
            if (this._state === STATE.armed) {
                this.abort('armed_timeout');
            }
        }, ARM_TIMEOUT_MS);
        if (typeof this._armTimer.unref === 'function') this._armTimer.unref();

        const startHint = TRAJECTORIES[trajectory]?.startHint ?? null;
        this.emit('armed', { runId, config: this._sanitizedRun(), startHint });
    }

    // Pre-start placement check. Takes a ground-truth snapshot and compares
    // the measured (x, y, θ) against the armed trajectory's startHint. Returns
    // { ok: true, measured, skipped? } if the robot is close enough to the
    // intended starting pose, or { ok: false, reason, measured? } if not.
    //
    // Rationale (Option C): we don't physically correct the trajectory for
    // small operator-placement errors — analysis rotates each run into its
    // own start frame, so body-frame metrics are unaffected. But a *large*
    // placement error (wrong heading, robot on the wrong tape, marker not
    // detected) is a setup mistake, and the experiment should refuse to run
    // rather than produce misleading data. This method is the safety gate.
    async preflight() {
        // Capture the runId at entry so that emitted events can carry
        // it unambiguously even if `this._run` gets swapped (see the
        // H4 race inside _runPreflightChecks). Without this, a
        // preflight_failed broadcast for a run_changed_during_preflight
        // outcome would go out with no runId, and any connected
        // dashboard would see "preflight failed" attributed to whatever
        // run is currently armed — typically the brand-new replacement
        // that hasn't actually been validated yet.
        const runIdAtEntry = this._run?.runId || null;
        const result = await this._runPreflightChecks();
        // Fan out to observers (webServer broadcasts `preflight_ok` /
        // `preflight_failed` to all connected browsers via the same
        // pattern as every other runner event — armed, started, tick,
        // aborted, etc). We only emit for *actionable* outcomes: a
        // failure with a reason, or a success that actually measured a
        // pose. The `skipped`/`not_armed` short-circuits are internal
        // bookkeeping and nobody needs to observe them.
        if (!result.ok && result.reason) {
            this.emit('preflight_failed', {
                runId: runIdAtEntry,
                reason: result.reason,
                measured: result.measured || null,
                delta: result.delta || null,
            });
        } else if (result.ok && result.measured) {
            this.emit('preflight_ok', {
                runId: runIdAtEntry,
                measured: result.measured,
            });
        }
        return result;
    }

    async _runPreflightChecks() {
        if (this._state !== STATE.armed) {
            return { ok: false, reason: `not_armed (state=${this._state})` };
        }
        // Demo mode runs off-grid, so placement checks make no sense and
        // would fail on "marker not detected" anyway. Short-circuit pass.
        if (this._run?.demo) {
            return { ok: true, skipped: 'demo' };
        }
        if (this._preflightHeadingTol === null && this._preflightPositionTol === null) {
            return { ok: true, skipped: 'disabled' };
        }
        if (!this._snapshotFn) {
            return { ok: true, skipped: 'no_snapshot_fn' };
        }

        // Capture `run` by identity at entry. The snapshot await below is
        // ~2.5 s long (camera pipeline lag), and within that window other
        // events — manual abort, connection drop triggering
        // runner.abort('robot_disconnect'), a fresh arm() after the
        // aborted→idle transition — can replace `this._run` with a
        // different trajectory. Without this guard, preflight would:
        //   (a) validate the OLD trajectory's startHint against the
        //       current camera reading (unintended measurement)
        //   (b) write `measuredStartPose` onto the orphaned OLD run
        //   (c) return ok:true, and the caller would start the NEW run
        //       with no preflight validation at all
        // All subsequent checks use the local `run` and `startHint`, and
        // before storing measuredStartPose we verify `this._run === run`
        // still holds.
        const run = this._run;
        const startHint = run?.startHint || {};
        let snap;
        try {
            snap = await this._snapshotFn({
                label: 'preflight',
                runId: run.runId,
                trajectory: run.trajectory,
            });
        } catch (err) {
            return { ok: false, reason: `snapshot_error: ${err.message}` };
        }
        if (this._run !== run) {
            return {
                ok: false,
                reason: `run_changed_during_preflight (was ${run.runId}, `
                    + `now ${this._run?.runId ?? 'idle'}). Re-arm and retry.`,
            };
        }
        // fetchSnapshot (index.js) normalises the detector reply to
        // { ok, x, y, theta, thetaDeg, ... }. `ok` is true only when
        // status==200 AND the marker was detected; it's false for
        // transport errors, timeouts, and non-detection. No `detected`
        // field surfaces through this wrapper.
        if (!snap || snap.ok !== true) {
            return {
                ok: false,
                reason: snap?.error
                    ? `snapshot_failed: ${snap.error}`
                    : 'marker_not_detected (is the robot on the grid and visible to the overhead camera?)',
                snapshot: snap || null,
            };
        }

        const measured = {
            x: Number(snap.x),
            y: Number(snap.y),
            thetaDeg: Number(snap.thetaDeg ?? snap.theta_deg),
        };
        if (!Number.isFinite(measured.x) || !Number.isFinite(measured.y)
            || !Number.isFinite(measured.thetaDeg)) {
            return {
                ok: false,
                reason: `preflight snapshot returned non-numeric pose: ${JSON.stringify({x: snap.x, y: snap.y, thetaDeg: snap.thetaDeg})}`,
                snapshot: snap,
            };
        }

        // Run at least one configured check, or fail. If a trajectory's
        // startHint is missing the fields the configured tolerance needs
        // (e.g. a debug trajectory with startHint.text but no x/y/
        // headingDeg), preflight would otherwise "pass" without validating
        // anything — the silent-skip failure mode that bit us on the
        // 2026-04-19 run where startHint wasn't being copied onto the run
        // at all (see H4). Fail-closed when tolerances are configured but
        // the startHint can't feed them.
        let checksPerformed = 0;

        // Heading check.
        if (this._preflightHeadingTol !== null) {
            if (!Number.isFinite(startHint.headingDeg)) {
                return {
                    ok: false,
                    reason: 'heading_tolerance_configured_but_startHint.headingDeg_missing',
                    measured,
                };
            }
            let dTheta = measured.thetaDeg - startHint.headingDeg;
            while (dTheta > 180) dTheta -= 360;
            while (dTheta < -180) dTheta += 360;
            if (Math.abs(dTheta) > this._preflightHeadingTol) {
                // When the heading is off by close to 180°, the most
                // likely cause is the robot being placed backwards on
                // the tape. Surface that as the first hypothesis so
                // operators don't waste a reset cycle nudging a
                // robot that's pointing the wrong way entirely.
                const hint = Math.abs(dTheta) > 165
                    ? 'is the robot facing the wrong way? heading off by nearly 180°'
                    : 'straighten the robot';
                return {
                    ok: false,
                    reason: `heading_out_of_tolerance: measured ${measured.thetaDeg.toFixed(1)}°, expected ${startHint.headingDeg}° (±${this._preflightHeadingTol}°) — ${hint}`,
                    measured,
                    delta: { thetaDeg: dTheta },
                };
            }
            checksPerformed += 1;
        }

        // Position check.
        if (this._preflightPositionTol !== null) {
            if (!Number.isFinite(startHint.x) || !Number.isFinite(startHint.y)) {
                return {
                    ok: false,
                    reason: 'position_tolerance_configured_but_startHint.x/y_missing',
                    measured,
                };
            }
            const dx = measured.x - startHint.x;
            const dy = measured.y - startHint.y;
            const dist = Math.hypot(dx, dy);
            if (dist > this._preflightPositionTol) {
                // The camera-bounded workspace sits roughly within
                // |x| ≤ 1.4 m, |y| ≤ 0.7 m. A measurement well
                // outside that envelope almost always means the
                // overhead homography has drifted (phone moved,
                // recalibration needed) — the robot is physically
                // incapable of being multiple metres off the tape.
                // Call that out instead of blaming placement.
                const outOfGrid = Math.abs(measured.x) > 3 || Math.abs(measured.y) > 3;
                const hint = outOfGrid
                    ? 'measurement is outside the workspace — overhead homography likely broken (recalibrate camera)'
                    : 'check that the robot is on the correct tape';
                return {
                    ok: false,
                    reason: `position_out_of_tolerance: measured (${measured.x.toFixed(2)}, ${measured.y.toFixed(2)}), expected (${startHint.x}, ${startHint.y}), distance ${(dist * 100).toFixed(1)} cm > ${(this._preflightPositionTol * 100).toFixed(0)} cm — ${hint}`,
                    measured,
                    delta: { x: dx, y: dy, distance: dist },
                };
            }
            checksPerformed += 1;
        }

        // Invariant: at least one tolerance is non-null at this point
        // (the all-null case returned `{ok: true, skipped: 'disabled'}`
        // earlier). If we got here without running any check, something
        // is wrong with the state machine.
        if (checksPerformed === 0) {
            return {
                ok: false,
                reason: 'preflight_internal_error: no checks performed despite tolerances configured',
                measured,
            };
        }

        // Second identity check: everything between the snapshot await and
        // here is synchronous JS, so `this._run` cannot have been swapped
        // since the post-snapshot check above. This re-check is defensive
        // — if future maintenance introduces another await in the checks
        // above (e.g. an async measurement adjustment), the guard keeps
        // us from writing to a stale run.
        if (this._run !== run) {
            return {
                ok: false,
                reason: `run_changed_during_preflight (was ${run.runId}, `
                    + `now ${this._run?.runId ?? 'idle'}). Re-arm and retry.`,
            };
        }

        // Store the measured pose on the run so meta.json captures the true
        // start frame. The data pipeline rotates all GT waypoints into this
        // frame when computing metrics.
        run.measuredStartPose = measured;

        return { ok: true, measured };
    }

    start() {
        if (this._state !== STATE.armed) {
            throw new Error(`cannot start in state ${this._state}`);
        }
        this._clearArmTimer();
        const startedAtMs = this._now();
        const startedAt = new Date(startedAtMs).toISOString();
        this._run.startedAt = startedAt;
        this._run.startedAtMs = startedAtMs;

        this._logEvent('exp_start');

        // Enable IMU heading-hold for the run. Firmware default is OFF;
        // every experiment turns it on so results are reproducible.
        if (this._robot?.setHeadingHold) {
            this._robot.setHeadingHold(true, HH_GAIN, HH_DEADZONE, HH_ALPHA);
            this._logEvent('heading_hold_on');
        }

        this._transition(STATE.running);
        this.emit('started', { runId: this._run.runId, startedAt });

        // Firmware executes the trajectory autonomously and emits
        // `traj_paused` / `traj_done` as waypoints and the run end fire.
        if (this._robot?.trajStart) {
            const sent = this._robot.trajStart();
            if (!sent) {
                this.abort('send_failed');
                return;
            }
        }

        this._armRunningTimeout();
    }

    // Overall safety net. If firmware goes silent past the nominal
    // trajectory duration + grace, abort — it's either crashed or the
    // connection has partitioned and we need to release the runner.
    // Re-armed on every pause/resume cycle to account for the time
    // spent paused (snapshot + operator wait).
    _armRunningTimeout() {
        this._clearRunningTimer();
        if (!this._run) return;
        const remainingMs = this._run.totalMs + RUNNING_GRACE_MS;
        this._runningTimer = setTimeout(() => {
            if (this._state === STATE.running) {
                console.error(
                    `Running timeout after ${remainingMs} ms — firmware may be silent`,
                );
                this.abort('running_timeout');
            }
        }, remainingMs);
        if (typeof this._runningTimer.unref === 'function') this._runningTimer.unref();
    }

    abort(reason = 'manual') {
        if (this._state === STATE.idle || this._state === STATE.aborted) return;

        this._clearArmTimer();
        this._clearGtTimer();
        this._clearRunningTimer();
        this._clearPauseWatchdog();
        this._setCommand({ vx: 0, vy: 0, omega: 0 });
        if (this._robot?.trajAbort) this._robot.trajAbort();
        if (this._robot?.stop) this._robot.stop();
        if (this._robot?.setHeadingHold) {
            this._robot.setHeadingHold(false, HH_GAIN, HH_DEADZONE, HH_ALPHA);
            this._logEvent('heading_hold_off');
        }

        if (this._run) {
            this._run.aborted = true;
            this._run.abortedReason = reason;
            this._run.endedAt = new Date(this._now()).toISOString();
        }

        // Demo runs have no logger interactions — no CSV, no meta sidecar.
        const isDemo = this._run?.demo === true;
        const csvPath = (!isDemo && this._logger?.filename) || null;
        if (this._logger && !isDemo) {
            if (this._logger.logEvent) this._logger.logEvent(`exp_abort:${reason}`);
            this._updateMeta('aborted');
            if (this._logger.stop) this._logger.stop();
        }

        const runId = this._run?.runId;
        this._transition(STATE.aborted);
        this.emit('aborted', { runId, reason, csvPath });

        this._run = null;
        this._transition(STATE.idle);
    }

    submitGroundTruth({ xMeas, yMeas, thetaDegMeas, passFail, notes = '' }) {
        this._clearGtTimer();
        if (this._state !== STATE.awaitingGroundTruth) {
            throw new Error(`cannot submit ground truth in state ${this._state}`);
        }
        if (!this._run) throw new Error('no active run');

        // Deep-copy prev before spreading — the old shallow `{...prev}`
        // aliased `prev.waypoints` between the old and new groundTruth
        // objects. No in-tree caller relies on the alias, but downstream
        // consumers (`_buildMetaBlock` output captured by the logger,
        // `emit('completed', { lastWaypointGt })` listeners) can still
        // hold references to entries inside `prev.waypoints`, and the
        // post-submit metadata write would then reflect any mutation
        // those consumers make. structuredClone isolates the new
        // groundTruth from both.
        const prev = this._run.groundTruth
            ? structuredClone(this._run.groundTruth)
            : { method: 'manual', waypoints: [] };
        this._run.groundTruth = {
            ...prev,
            xMeas: Number(xMeas),
            yMeas: Number(yMeas),
            thetaDegMeas: Number(thetaDegMeas),
            passFail: passFail === 'fail' ? 'fail' : 'pass',
            notes,
        };

        if (this._logger) {
            this._updateMeta('ground_truth_submitted');
            if (this._logger.stop) this._logger.stop();
        }

        this._run = null;
        this._transition(STATE.idle);
    }

    // --- Internals ---

    // Firmware has reached a pause segment and physically stopped. Capture
    // a snapshot (if we know a label for this segmentIdx), then release
    // the firmware via `traj_resume`. Snapshot failures are logged as a
    // null-gt waypoint so the run continues — a missed overhead frame is
    // not worth aborting a ten-waypoint sequence.
    _onTrajPaused(msg) {
        if (this._state !== STATE.running || !this._run) return;
        if (msg.runId && msg.runId !== this._run.runId) {
            console.warn(
                `traj_paused runId mismatch: expected ${this._run.runId}, got ${msg.runId}`,
            );
            return;
        }

        const run = this._run;
        const segmentIdx = typeof msg.seg === 'number' ? msg.seg : null;
        const label = segmentIdx !== null
            ? (run.labelBySegIdx.get(segmentIdx) || null)
            : null;

        // The overall trajectory safety net is paused while we snapshot;
        // swap it for a pause-specific watchdog that catches snapshot
        // service hangs.
        this._clearRunningTimer();
        this._armPauseWatchdog();

        this.emit('paused', {
            runId: run.runId,
            segmentIdx,
            label,
            waypointIdx: run.waypointsCompleted,
        });
        this._logEvent(`traj_paused:${label || '(unlabeled)'}`);

        // Capture the snapshot (if we can), then resume regardless of
        // outcome. The resume is always issued — even if label/snapshot
        // is missing — so the firmware never deadlocks at a pause.
        const firmwarePose = msg.pose ? {
            x: Number(msg.pose.x) || 0,
            y: Number(msg.pose.y) || 0,
            th: Number(msg.pose.th) || 0,
        } : null;

        // Wait the camera-pipeline lead time before calling the snapshot
        // service — covers phone exposure + encode + RTSP + decode
        // latency that firmware's mechanical-settle window doesn't.
        // setTimeout(0) keeps the zero-lead path synchronous-equivalent
        // so tests don't need to add extra awaits when snapshotLeadMs=0.
        const scheduleCapture = (cb) => {
            if (this._snapshotLeadMs <= 0) { cb(); return; }
            const t = setTimeout(cb, this._snapshotLeadMs);
            if (typeof t.unref === 'function') t.unref();
        };

        scheduleCapture(() => {
            if (this._state !== STATE.running || this._run !== run) return;
            const capturePromise = this._captureWaypoint(label, segmentIdx, firmwarePose);
            Promise.resolve(capturePromise).then(() => {
                if (this._state !== STATE.running || this._run !== run) return;
                run.waypointsCompleted += 1;
                this._clearPauseWatchdog();
                this._armRunningTimeout();

                // Tier-0 workspace-bounds guard. Reads the GT that
                // _captureWaypoint just pushed; if the robot has
                // drifted outside the envelope we abort BEFORE
                // trajResume so the firmware stays parked. Skipped
                // for tiers 1/2 (validated without this gate and we
                // don't want to introduce new abort paths late).
                //
                // FAIL-CLOSED: a null/invalid GT at a tier-0 pause also
                // aborts. "Marker not detected" usually means the robot
                // has left the camera frame — i.e. the scenario the
                // guard exists for. A 2026-04-20 straight_2m run rolled
                // 1.3 m past +X because the final snapshot failed and
                // the old version fell through to trajResume.
                if (this._getTier && this._getTier() === 0) {
                    const last = run.groundTruth?.waypoints?.at(-1);
                    const gt = last?.gt;
                    const gtValid = gt && Number.isFinite(gt.x) && Number.isFinite(gt.y);
                    if (!gtValid) {
                        const reason = last?.error || 'no ground-truth returned';
                        this.abort(
                            `marker_lost at ${label || `seg${segmentIdx}`}: ${reason} ` +
                            '(tier-0 fails closed — cannot verify bounds without GT)',
                        );
                        return;
                    }
                    if (Math.abs(gt.x) > TIER0_BOUNDS_X_M ||
                        Math.abs(gt.y) > TIER0_BOUNDS_Y_M) {
                        this.abort(
                            `workspace_bounds_violated at ${label || `seg${segmentIdx}`}: ` +
                            `GT=(${gt.x.toFixed(2)}, ${gt.y.toFixed(2)}) ` +
                            `exceeds ±${TIER0_BOUNDS_X_M}×${TIER0_BOUNDS_Y_M} m envelope`,
                        );
                        return;
                    }
                }

                if (this._robot?.trajResume) {
                    const ok = this._robot.trajResume();
                    if (!ok) {
                        this.abort('resume_failed');
                        return;
                    }
                }
                this._logEvent(`traj_resume:${label || '(unlabeled)'}`);
            });
        });
    }

    _captureWaypoint(label, segmentIdx, firmwarePose) {
        if (!this._run) return null;

        const encoderPose = this._getPose ? this._getPose() : null;
        const encoder = encoderPose
            ? {
                x: encoderPose.x,
                y: encoderPose.y,
                theta_rad: encoderPose.theta,
                theta_deg: encoderPose.thetaDeg,
            }
            : null;
        const baseRecord = {
            label,
            timestamp_ms: this._now(),
            segmentIdx,
            encoder,
            firmwarePose,
        };

        if (!label) {
            // Pause without a label shouldn't occur in the current catalog
            // but is harmless — just record it, no snapshot.
            this._run.groundTruth.waypoints.push({
                ...baseRecord,
                gt: null,
                error: 'pause segment had no label',
            });
            this._updateMeta('running');
            return null;
        }

        this._logEvent(`snapshot_requested:${label}`);

        if (!this._snapshotFn) {
            this._run.groundTruth.waypoints.push({
                ...baseRecord,
                gt: null,
                error: 'no snapshot service configured',
            });
            this._updateMeta('running');
            return null;
        }

        return Promise.resolve(this._snapshotFn({
            label,
            runId: this._run.runId,
            trajectory: this._run.trajectory,
        })).then((res) => {
            if (!this._run) return;
            if (res && res.ok) {
                this._run.groundTruth.waypoints.push({
                    ...baseRecord,
                    gt: {
                        x: res.x,
                        y: res.y,
                        theta_rad: res.theta,
                        theta_deg: res.thetaDeg,
                        side_px: res.sidePx,
                        image_path: res.imagePath,
                    },
                });
                this._logEvent(`snapshot_ok:${label}`);
            } else {
                const error = (res && res.error) || 'snapshot failed';
                this._run.groundTruth.waypoints.push({ ...baseRecord, gt: null, error });
                this._logEvent(`snapshot_failed:${label}:${error}`);
            }
            this._updateMeta('running');
        }).catch((err) => {
            if (!this._run) return;
            this._run.groundTruth.waypoints.push({
                ...baseRecord,
                gt: null,
                error: `snapshot exception: ${err.message}`,
            });
            this._logEvent(`snapshot_exception:${label}:${err.message}`);
            this._updateMeta('running');
        });
    }

    // Firmware has finished the whole trajectory. All pauses resolved,
    // the tail motion is done — hand off to GT submission.
    _onTrajDone(msg) {
        if (this._state !== STATE.running || !this._run) return;
        if (msg.runId && msg.runId !== this._run.runId) {
            console.warn(
                `traj_done runId mismatch: expected ${this._run.runId}, got ${msg.runId}`,
            );
            return;
        }

        this._clearRunningTimer();
        this._clearPauseWatchdog();
        this._setCommand({ vx: 0, vy: 0, omega: 0 });
        this._complete();
    }

    _onTrajProgress(msg) {
        if (this._state !== STATE.running || !this._run) {
            if (this._state !== STATE.idle) {
                console.warn(`Received traj_progress in state ${this._state}, ignoring`);
            }
            return;
        }

        const run = this._run;
        const waypointsTotal = run.waypointDefs.length;
        this.emit('tick', {
            runId: run.runId,
            phase: 'running',
            segmentIdx: msg.seg ?? 0,
            elapsedMs: msg.elapsed ?? 0,
            totalMs: msg.total ?? run.totalMs,
            waypointsCompleted: run.waypointsCompleted,
            waypointsTotal,
            currentChunkLabel: this._nextLabelAfter(msg.seg ?? 0),
        });
    }

    // Look up the label of the next pause segment at or after the given
    // segment index — lets the dashboard show "heading toward wpN" while
    // motion segments are active.
    _nextLabelAfter(segIdx) {
        if (!this._run) return null;
        for (const wp of this._run.waypointDefs) {
            if (wp.segmentIdx >= segIdx) return wp.label;
        }
        return null;
    }

    _complete() {
        this._clearRunningTimer();
        this._clearPauseWatchdog();
        this._setCommand({ vx: 0, vy: 0, omega: 0 });
        if (this._robot?.stop) this._robot.stop();
        if (this._robot?.setHeadingHold) {
            this._robot.setHeadingHold(false, HH_GAIN, HH_DEADZONE, HH_ALPHA);
        }

        this._run.endedAt = new Date(this._now()).toISOString();

        const isDemo = this._run.demo === true;
        const csvPath = (!isDemo && this._logger?.filename) || null;
        if (!isDemo) {
            this._logEvent('exp_end');
            this._logEvent('heading_hold_off');
            this._updateMeta('completed');
        }

        const durationMs = new Date(this._run.endedAt).getTime()
            - new Date(this._run.startedAt).getTime();

        const waypoints = this._run.groundTruth?.waypoints || [];
        let lastWaypointGt = null;
        for (let i = waypoints.length - 1; i >= 0; i--) {
            if (waypoints[i].gt) { lastWaypointGt = waypoints[i].gt; break; }
        }
        const runId = this._run.runId;

        // Demo runs skip the awaiting_ground_truth gate: no GT to collect,
        // no CSV sidecar to finalize. Emit `completed` with `demo: true` so
        // the UI can skip showing the GT form, then drop straight to idle.
        if (isDemo) {
            this._run = null;
            this._transition(STATE.idle);
            this.emit('completed', {
                runId,
                csvPath: null,
                durationMs,
                demo: true,
                waypointsTotal: 0,
                waypointsOk: 0,
                lastWaypointGt: null,
            });
            return;
        }

        this._transition(STATE.awaitingGroundTruth);
        this._gtTimer = setTimeout(() => {
            if (this._state === STATE.awaitingGroundTruth) {
                this.abort('gt_timeout');
            }
        }, GT_TIMEOUT_MS);
        if (typeof this._gtTimer.unref === 'function') this._gtTimer.unref();

        this.emit('completed', {
            runId,
            csvPath,
            durationMs,
            waypointsTotal: waypoints.length,
            waypointsOk:    waypoints.filter(w => w.gt).length,
            // Deep-copy so listeners cannot mutate the live waypoint
            // record still held by _run (the metadata write after
            // submitGroundTruth would otherwise reflect the mutation).
            lastWaypointGt: lastWaypointGt ? structuredClone(lastWaypointGt) : null,
        });
    }

    _clearArmTimer()     { if (this._armTimer)     { clearTimeout(this._armTimer);     this._armTimer     = null; } }
    _clearGtTimer()      { if (this._gtTimer)      { clearTimeout(this._gtTimer);      this._gtTimer      = null; } }
    _clearRunningTimer() { if (this._runningTimer) { clearTimeout(this._runningTimer); this._runningTimer = null; } }
    _clearPauseWatchdog(){ if (this._pauseWatchdog){ clearTimeout(this._pauseWatchdog);this._pauseWatchdog= null; } }

    _armPauseWatchdog() {
        this._clearPauseWatchdog();
        this._pauseWatchdog = setTimeout(() => {
            if (this._state === STATE.running) {
                console.error('Pause watchdog tripped — snapshot/resume stalled');
                this.abort('pause_watchdog');
            }
        }, PAUSE_WATCHDOG_MS);
        if (typeof this._pauseWatchdog.unref === 'function') this._pauseWatchdog.unref();
    }

    _transition(next) {
        const from = this._state;
        this._state = next;
        if (from !== next) this.emit('stateChange', { from, to: next });
    }

    _buildMetaBlock(status) {
        if (!this._run) return null;
        return {
            runId: this._run.runId,
            trajectory: this._run.trajectory,
            speed: this._run.speed,
            tier: this._run.tier,
            rep: this._run.rep,
            operatorNotes: this._run.operatorNotes,
            demo: this._run.demo === true,
            totalDurationMs: this._run.totalMs,
            armedAt: this._run.armedAt,
            startedAt: this._run.startedAt,
            endedAt: this._run.endedAt,
            aborted: this._run.aborted,
            abortedReason: this._run.abortedReason,
            // Deep-copy so the logger's captured meta block can't be
            // mutated back into _run (and vice-versa). Primitive fields
            // above are copied by value; these two are objects.
            groundTruth: this._run.groundTruth
                ? structuredClone(this._run.groundTruth)
                : null,
            // Measured world-frame pose of the robot at experiment_start,
            // captured by the preflight snapshot. The analysis pipeline
            // rotates all GT waypoints into this frame so body-frame metrics
            // (loop closure, endpoint error) are invariant to small operator
            // placement heading errors.
            measuredStartPose: this._run.measuredStartPose
                ? structuredClone(this._run.measuredStartPose)
                : null,
            status,
        };
    }

    _updateMeta(status) {
        if (this._run?.demo) return;
        if (!this._logger?.updateExperimentMeta) return;
        this._logger.updateExperimentMeta(this._buildMetaBlock(status));
    }

    // Logger.logEvent wrapper that respects demo mode. Demo runs produce no
    // CSV and no meta.json, so events like `exp_start`, `heading_hold_on`,
    // `traj_paused:*` etc. would write nothing useful even if the logger is
    // attached. Having one helper keeps the call sites readable.
    _logEvent(tag) {
        if (this._run?.demo) return;
        if (this._logger?.logEvent) this._logger.logEvent(tag);
    }

    _sanitizedRun() {
        if (!this._run) return null;
        // Deep-copy for the same reason as `getRun()` — external
        // consumers (the `armed` event listener in webServer) must not
        // be able to mutate nested structures (startHint, groundTruth)
        // back into the live run.
        const { built: _built, labelBySegIdx: _lbl, ...rest } = this._run;
        return structuredClone(rest);
    }
}
