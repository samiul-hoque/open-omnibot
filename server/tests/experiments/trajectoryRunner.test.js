// ============================================
// TrajectoryRunner state machine tests
// ============================================
//
// The runner now uploads the entire trajectory once on arm (motion
// segments interleaved with `pause` markers). Firmware halts on each
// pause and fires `traj_paused`; the runner captures a snapshot, calls
// `trajResume`, and waits for the next pause or `traj_done`.

import { describe, it, mock } from 'node:test';
import assert from 'node:assert';
import { EventEmitter } from 'node:events';

import { TrajectoryRunner, STATE } from '../../src/experiments/trajectoryRunner.js';
import { TRAJECTORIES } from '../../src/experiments/trajectories.js';

function makeMocks() {
    const commands = [];
    const robot = Object.assign(new EventEmitter(), {
        stop: mock.fn(),
        resetEncoders: mock.fn(),
        zeroImu: mock.fn(),
        loadTrajectory: mock.fn(() => true),
        trajStart: mock.fn(() => true),
        trajResume: mock.fn(() => true),
        trajAbort: mock.fn(() => true),
        setHeadingHold: mock.fn(() => true),
    });
    const localization = { reset: mock.fn() };
    const logger = {
        filename: '/tmp/fake.csv',
        _meta: { events: [] },
        events: [],
        started: 0,
        stopped: 0,
        start(sessionInfo) {
            this.started++;
            this._meta = { events: [], ...sessionInfo };
            this.filename = `/tmp/${sessionInfo?.prefix ?? 'fake'}.csv`;
        },
        stop() { this.stopped++; },
        logEvent(tag) { this.events.push(tag); this._meta.events.push({ tag }); },
        updateExperimentMeta(block) { this._meta.experiment = block; this._metaWritten = true; },
        _writeMetadata() { this._metaWritten = true; },
    };
    return { commands, robot, localization, logger };
}

function makeRunner(mocks, extra = {}) {
    const { commands, robot, localization, logger } = mocks;
    return new TrajectoryRunner({
        robot,
        localization,
        logger,
        setCommand: (cmd) => commands.push({ ...cmd }),
        getTier: () => 1,
        now: extra.now,
        snapshotFn: extra.snapshotFn || null,
        getPose: extra.getPose || null,
        // Tests drive the pause→snapshot→resume path synchronously.
        // Zero lead keeps the path microtask-only; the 400 ms default
        // is for production runs against a real camera.
        snapshotLeadMs: extra.snapshotLeadMs ?? 0,
        // Preflight tolerances. Default null (disabled) so existing tests
        // don't accidentally trip the snapshotFn path. Preflight tests
        // pass explicit values.
        preflightHeadingToleranceDeg: extra.preflightHeadingToleranceDeg ?? null,
        preflightPositionToleranceM: extra.preflightPositionToleranceM ?? null,
    });
}

// Drain the microtask queue — the pause→snapshot→resume path chains
// through `.then().catch()` and then another `.then()`, so a single
// `await Promise.resolve()` is not enough. setImmediate runs after all
// currently-queued microtasks.
const flushMicrotasks = () => new Promise((r) => setImmediate(r));

// Drive a runner through all its pauses by replaying `traj_paused` in
// the order the firmware would emit them, then `traj_done`. Awaits
// microtasks between each step so any Promise.resolve() snapshot
// capture settles before the next emit.
async function driveToCompletion(runner, robot, trajectoryKey, speed) {
    const built = TRAJECTORIES[trajectoryKey].build(speed);
    const run = runner.getRun();
    for (const wp of built.waypoints) {
        robot.emit('traj_paused', {
            type: 'traj_paused',
            runId: run.runId,
            seg: wp.segmentIdx,
            pose: { x: 0, y: 0, th: 0 },
        });
        await flushMicrotasks();
    }
    robot.emit('traj_done', {
        type: 'traj_done',
        runId: run.runId,
        pose: { x: 0, y: 0, th: 0 },
    });
    await flushMicrotasks();
}

describe('TrajectoryRunner — state transitions', () => {
    it('starts in idle', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        assert.strictEqual(runner.getState(), STATE.idle);
    });

    it('arm uploads the entire trajectory once (motion + pause segments)', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'straight_2m', speed: 0.4, rep: 1 });

        assert.strictEqual(runner.getState(), STATE.armed);
        assert.strictEqual(mocks.robot.loadTrajectory.mock.callCount(), 1);

        // The full trajectory should have been sent — including pause markers.
        // straight_2m has a leading start-pose pause + 4 motion/pause pairs = 9.
        const [, segments] = mocks.robot.loadTrajectory.mock.calls[0].arguments;
        const pauseCount = segments.filter(s => s.kind === 'pause').length;
        const motionCount = segments.length - pauseCount;
        assert.strictEqual(motionCount, 4, 'four 0.5 m translates');
        assert.strictEqual(pauseCount, 5, 'start + 4 waypoint pauses');

        assert.strictEqual(mocks.localization.reset.mock.callCount(), 1);
        assert.strictEqual(mocks.logger.started, 1);
        assert.ok(mocks.logger.events.includes('exp_arm'));
    });

    it('rejects unknown trajectory', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        assert.throws(
            () => runner.arm({ trajectory: 'bogus', speed: 0.4 }),
            /unknown trajectory/,
        );
        assert.strictEqual(runner.getState(), STATE.idle);
    });

    it('rejects non-positive speed', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        assert.throws(() => runner.arm({ trajectory: 'straight_2m', speed: 0 }), /invalid speed/);
        assert.throws(() => runner.arm({ trajectory: 'straight_2m', speed: -1 }), /invalid speed/);
    });

    it('cannot start before arming', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        assert.throws(() => runner.start(), /cannot start in state idle/);
    });

    it('cannot arm twice without completing first run', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        assert.throws(
            () => runner.arm({ trajectory: 'straight_2m', speed: 0.4 }),
            /cannot arm in state armed/,
        );
    });
});

describe('TrajectoryRunner — running + pause/resume flow', () => {
    it('start() delegates to robot.trajStart()', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks, { now: () => 1_000_000 });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        runner.start();

        assert.strictEqual(runner.getState(), STATE.running);
        assert.strictEqual(mocks.robot.trajStart.mock.callCount(), 1);
        assert.strictEqual(mocks.robot.loadTrajectory.mock.callCount(), 1);
    });

    it('traj_paused triggers resume (no snapshot service configured)', async () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const run = runner.getRun();
        runner.start();

        const pausedEvents = [];
        runner.on('paused', (e) => pausedEvents.push(e));

        const built = TRAJECTORIES.straight_2m.build(0.4);
        const firstWp = built.waypoints[0];
        mocks.robot.emit('traj_paused', {
            type: 'traj_paused',
            runId: run.runId,
            seg: firstWp.segmentIdx,
            pose: { x: 0, y: 0, th: 0 },
        });
        await flushMicrotasks();

        assert.strictEqual(pausedEvents.length, 1);
        assert.strictEqual(pausedEvents[0].label, firstWp.label);
        assert.strictEqual(pausedEvents[0].segmentIdx, firstWp.segmentIdx);
        // Without a snapshot service we still resume immediately (stub waypoint).
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 1);
        // A null-gt waypoint is recorded so the waypoint count stays correct.
        const wps = runner.getRun().groundTruth.waypoints;
        assert.strictEqual(wps.length, 1);
        assert.strictEqual(wps[0].gt, null);
        assert.strictEqual(wps[0].error, 'no snapshot service configured');
    });

    it('traj_paused awaits an async snapshot before resuming', async () => {
        const mocks = makeMocks();
        let resolveSnapshot;
        const pending = new Promise((r) => { resolveSnapshot = r; });
        const snapshotFn = mock.fn(() => pending);
        const runner = makeRunner(mocks, { snapshotFn });

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const run = runner.getRun();
        runner.start();

        const built = TRAJECTORIES.straight_2m.build(0.4);
        mocks.robot.emit('traj_paused', {
            type: 'traj_paused',
            runId: run.runId,
            seg: built.waypoints[0].segmentIdx,
            pose: { x: 0.5, y: 0, th: 0 },
        });
        await flushMicrotasks();

        // Snapshot is in flight; resume has NOT been called yet.
        assert.strictEqual(snapshotFn.mock.callCount(), 1);
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 0);

        // Resolve the snapshot — resume should fire on the next microtask.
        resolveSnapshot({ ok: true, x: 0.5, y: 0.01, theta: 0, thetaDeg: 0, sidePx: 80, imagePath: '/tmp/x.png' });
        await pending;
        await flushMicrotasks();

        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 1);
        const wps = runner.getRun().groundTruth.waypoints;
        assert.strictEqual(wps.length, 1);
        assert.ok(wps[0].gt);
        assert.strictEqual(wps[0].gt.x, 0.5);
    });

    it('snapshotLeadMs delays the snapshot call (camera-pipeline lead)', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({ ok: true, x: 0.5, y: 0, theta: 0, thetaDeg: 0 }));
        const runner = makeRunner(mocks, { snapshotFn, snapshotLeadMs: 50 });

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const run = runner.getRun();
        runner.start();
        const built = TRAJECTORIES.straight_2m.build(0.4);

        mocks.robot.emit('traj_paused', {
            type: 'traj_paused', runId: run.runId,
            seg: built.waypoints[0].segmentIdx, pose: { x: 0.5, y: 0, th: 0 },
        });
        // Immediately after traj_paused fires the snapshot has NOT been
        // called yet — the runner is in its lead-delay window.
        await flushMicrotasks();
        assert.strictEqual(snapshotFn.mock.callCount(), 0);
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 0);

        // After the lead elapses the snapshot fires and resume follows.
        await new Promise((r) => setTimeout(r, 80));
        await flushMicrotasks();
        assert.strictEqual(snapshotFn.mock.callCount(), 1);
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 1);
    });

    it('snapshot failure still resumes and logs a null-gt waypoint', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({ ok: false, error: 'service down' }));
        const runner = makeRunner(mocks, { snapshotFn });

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const run = runner.getRun();
        runner.start();
        const built = TRAJECTORIES.straight_2m.build(0.4);

        mocks.robot.emit('traj_paused', {
            type: 'traj_paused',
            runId: run.runId,
            seg: built.waypoints[0].segmentIdx,
            pose: { x: 0.5, y: 0, th: 0 },
        });
        await flushMicrotasks();

        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 1);
        const wps = runner.getRun().groundTruth.waypoints;
        assert.strictEqual(wps[0].gt, null);
        assert.strictEqual(wps[0].error, 'service down');
    });

    it('traj_done from robot transitions to awaiting_ground_truth', async () => {
        const mocks = makeMocks();
        let fakeNow = 1_000_000;
        const runner = makeRunner(mocks, { now: () => fakeNow });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });

        const completedEvents = [];
        runner.on('completed', (e) => completedEvents.push(e));

        runner.start();
        await driveToCompletion(runner, mocks.robot, 'straight_2m', 0.4);
        fakeNow += 5000;
        // driveToCompletion already emitted traj_done; let microtasks settle.
        await flushMicrotasks();

        assert.strictEqual(runner.getState(), STATE.awaitingGroundTruth);
        assert.strictEqual(completedEvents.length, 1);
        assert.ok(mocks.logger.events.includes('exp_end'));
        // One waypoint per pause (5 total for straight_2m: start + 4 motion waypoints),
        // all null-gt stubs in this test (no snapshotFn injected).
        const wps = runner.getRun()?.groundTruth?.waypoints
            || mocks.logger._meta.experiment.groundTruth.waypoints;
        assert.strictEqual(wps.length, 5);
        for (const w of wps) {
            assert.strictEqual(w.gt, null);
        }
        // 5 pauses → 5 resumes (start + 4 waypoints).
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 5);
    });

    it('traj_progress emits tick events with waypoint progress', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks, { now: () => 1_000_000 });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        runner.start();

        const tickEvents = [];
        runner.on('tick', (e) => tickEvents.push(e));

        mocks.robot.emit('traj_progress', {
            type: 'traj_progress',
            runId: 'test',
            seg: 0,
            elapsed: 1000,
            total: 5000,
        });
        assert.strictEqual(tickEvents.length, 1);
        assert.strictEqual(tickEvents[0].segmentIdx, 0);
        assert.strictEqual(tickEvents[0].waypointsTotal, 5);
    });

    it('abort during running calls robot.trajAbort and transitions to idle', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks, { now: () => 1_000_000 });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });

        const abortedEvents = [];
        runner.on('aborted', (e) => abortedEvents.push(e));

        runner.start();
        runner.abort('estop');

        assert.strictEqual(runner.getState(), STATE.idle);
        assert.strictEqual(abortedEvents.length, 1);
        assert.strictEqual(abortedEvents[0].reason, 'estop');
        assert.strictEqual(mocks.robot.trajAbort.mock.callCount(), 1);
        assert.ok(mocks.logger.events.some(e => e.startsWith('exp_abort')));
    });

    it('ground-truth submission attaches measurements and returns to idle', async () => {
        const mocks = makeMocks();
        const fakeNow = 1_000_000;
        const runner = makeRunner(mocks, { now: () => fakeNow });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        runner.start();
        await driveToCompletion(runner, mocks.robot, 'straight_2m', 0.4);

        assert.strictEqual(runner.getState(), STATE.awaitingGroundTruth);
        runner.submitGroundTruth({
            xMeas: 1.98, yMeas: -0.03, thetaDegMeas: 0.5, passFail: 'pass', notes: 'clean run',
        });
        assert.strictEqual(runner.getState(), STATE.idle);
        const exp = mocks.logger._meta.experiment;
        assert.ok(exp);
        assert.strictEqual(exp.groundTruth.xMeas, 1.98);
        assert.strictEqual(exp.groundTruth.passFail, 'pass');
        assert.strictEqual(exp.groundTruth.waypoints.length, 5);
        assert.strictEqual(mocks.logger.stopped, 2);
    });

    it('rejects ground-truth submission outside awaitingGroundTruth', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        assert.throws(
            () => runner.submitGroundTruth({ xMeas: 0, yMeas: 0, thetaDegMeas: 0, passFail: 'pass' }),
            /cannot submit ground truth/,
        );
    });

    it('getRun() + completed event + final meta are isolated from mutation', async () => {
        // Regression for MR B #9: _sanitizedRun / _buildMetaBlock /
        // `completed` event used to expose live references to nested
        // run state (groundTruth, startHint, lastWaypointGt). A consumer
        // that mutated any of those — a test, a JSON-prune pass in a
        // dashboard, a logger that post-processes its captured meta —
        // would silently corrupt the sidecar JSON or a subsequent run.
        // After the fix every boundary structuredClone's across.
        const mocks = makeMocks();
        let completeEvent = null;
        const runner = makeRunner(mocks);
        runner.on('completed', (e) => { completeEvent = e; });

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });

        // Mutate the `armed`-path sanitizedRun payload via getRun.
        const armedCopy = runner.getRun();
        armedCopy.trajectory = 'HIJACKED';
        armedCopy.groundTruth.waypoints.push({ injected: 'armed' });

        runner.start();
        await driveToCompletion(runner, mocks.robot, 'straight_2m', 0.4);

        // Mutate the emitted `completed` event snapshot.
        completeEvent.lastWaypointGt = { x: 999, y: 999 };

        runner.submitGroundTruth({
            xMeas: 2.0, yMeas: 0, thetaDegMeas: 0, passFail: 'pass',
        });

        // Mutating the captured meta block after submission must not
        // feed back into the (now-stale) run or into any other captured
        // copy. Write, then re-read, to prove it doesn't throw.
        const meta = mocks.logger._meta.experiment;
        const originalWaypointCount = meta.groundTruth.waypoints.length;
        meta.groundTruth.waypoints.push({ injected: 'meta' });

        // The meta block's groundTruth reflects the TRUE submitted
        // values — not the hijacked string from earlier.
        assert.strictEqual(meta.trajectory, 'straight_2m');
        assert.strictEqual(meta.groundTruth.xMeas, 2.0);
        // And the waypoint count at submit time excluded the armed-path
        // injection and the completed-event mutation.
        assert.strictEqual(originalWaypointCount, 5);
    });

    it('traj_paused / traj_done with wrong runId is ignored', async () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        runner.start();
        const built = TRAJECTORIES.straight_2m.build(0.4);

        // Wrong runId — ignored.
        mocks.robot.emit('traj_paused', {
            type: 'traj_paused', runId: 'wrong_id',
            seg: built.waypoints[0].segmentIdx, pose: {},
        });
        await flushMicrotasks();
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 0);

        // Walk through all waypoints with the correct runId.
        await driveToCompletion(runner, mocks.robot, 'straight_2m', 0.4);
        assert.strictEqual(runner.getState(), STATE.awaitingGroundTruth);
    });

    it('square_0m8_rotate drives all 13 pauses and completes', async () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'square_0m8_rotate', speed: 0.4 });
        runner.start();
        await driveToCompletion(runner, mocks.robot, 'square_0m8_rotate', 0.4);

        assert.strictEqual(runner.getState(), STATE.awaitingGroundTruth);
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 13);
        assert.strictEqual(
            runner.getRun()?.groundTruth?.waypoints?.length
                ?? mocks.logger._meta.experiment.groundTruth.waypoints.length,
            13,
        );
    });
});

describe('TrajectoryRunner — failure handling', () => {
    it('arm throws if loadTrajectory returns false (disconnected)', () => {
        const mocks = makeMocks();
        mocks.robot.loadTrajectory = mock.fn(() => false);
        const runner = makeRunner(mocks);
        assert.throws(
            () => runner.arm({ trajectory: 'straight_2m', speed: 0.4 }),
            /Failed to send trajectory/,
        );
        assert.strictEqual(runner.getState(), STATE.idle);
    });

    it('start aborts if trajStart returns false (disconnected)', () => {
        const mocks = makeMocks();
        mocks.robot.trajStart = mock.fn(() => false);
        const runner = makeRunner(mocks);

        const abortedEvents = [];
        runner.on('aborted', (e) => abortedEvents.push(e));

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        runner.start();

        assert.strictEqual(runner.getState(), STATE.idle);
        assert.strictEqual(abortedEvents.length, 1);
        assert.strictEqual(abortedEvents[0].reason, 'send_failed');
    });

    it('resume failure aborts the run', async () => {
        const mocks = makeMocks();
        mocks.robot.trajResume = mock.fn(() => false);
        const runner = makeRunner(mocks);

        const abortedEvents = [];
        runner.on('aborted', (e) => abortedEvents.push(e));

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const run = runner.getRun();
        runner.start();
        const built = TRAJECTORIES.straight_2m.build(0.4);

        mocks.robot.emit('traj_paused', {
            type: 'traj_paused', runId: run.runId,
            seg: built.waypoints[0].segmentIdx, pose: {},
        });
        await flushMicrotasks();

        assert.strictEqual(runner.getState(), STATE.idle);
        assert.strictEqual(abortedEvents.length, 1);
        assert.strictEqual(abortedEvents[0].reason, 'resume_failed');
    });

    it('tier-0 aborts when waypoint snapshot fails (marker lost / fail-closed)', async () => {
        // Regression: a 2026-04-20 straight_2m tier-0 run rolled 1.3 m
        // past the +X wall because the final snapshot returned
        // {ok: false, error: "no marker detected"} and the bounds guard
        // fell through instead of aborting. Invisible robot ≠ safe robot.
        const mocks = makeMocks();
        const runner = makeRunner(mocks, {
            snapshotFn: async () => ({ ok: false, error: 'no marker detected' }),
        });
        runner._getTier = () => 0;

        const abortedEvents = [];
        runner.on('aborted', (e) => abortedEvents.push(e));

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const run = runner.getRun();
        runner.start();
        const built = TRAJECTORIES.straight_2m.build(0.4);

        mocks.robot.emit('traj_paused', {
            type: 'traj_paused', runId: run.runId,
            seg: built.waypoints[0].segmentIdx, pose: { x: 0, y: 0, th: 0 },
        });
        await flushMicrotasks();

        assert.strictEqual(runner.getState(), STATE.idle);
        assert.strictEqual(abortedEvents.length, 1);
        assert.match(abortedEvents[0].reason, /marker_lost/);
        assert.match(abortedEvents[0].reason, /no marker detected/);
        // Firmware must have NOT been told to resume
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 0);
    });

    it('tier-1 snapshot failure does NOT abort (guard is tier-0 only)', async () => {
        // Sanity check: the fail-closed behaviour must not bleed into
        // validated tier-1/2 paths, which deliberately skip bounds.
        const mocks = makeMocks();
        const runner = makeRunner(mocks, {
            snapshotFn: async () => ({ ok: false, error: 'no marker detected' }),
        });
        // default getTier returns 1

        const abortedEvents = [];
        runner.on('aborted', (e) => abortedEvents.push(e));

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const run = runner.getRun();
        runner.start();
        const built = TRAJECTORIES.straight_2m.build(0.4);

        mocks.robot.emit('traj_paused', {
            type: 'traj_paused', runId: run.runId,
            seg: built.waypoints[0].segmentIdx, pose: { x: 0, y: 0, th: 0 },
        });
        await flushMicrotasks();

        assert.strictEqual(abortedEvents.length, 0);
        assert.strictEqual(runner.getState(), STATE.running);
        // Firmware was told to resume because tier-1 doesn't gate on GT
        assert.strictEqual(mocks.robot.trajResume.mock.callCount(), 1);
    });

    it('robot disconnect during running aborts trajectory', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);

        const abortedEvents = [];
        runner.on('aborted', (e) => abortedEvents.push(e));

        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        runner.start();
        assert.strictEqual(runner.getState(), STATE.running);

        mocks.robot.emit('disconnected');

        assert.strictEqual(runner.getState(), STATE.idle);
        assert.strictEqual(abortedEvents.length, 1);
        assert.strictEqual(abortedEvents[0].reason, 'robot_disconnect');
        assert.strictEqual(mocks.robot.trajAbort.mock.callCount(), 1);
    });
});


// ============================================
// Preflight placement-validation tests
// ============================================
//
// preflight() is called by the webServer at experiment_start time,
// before the runner transitions to running. It snaps the overhead
// camera via `snapshotFn`, compares the measured (x, y, θ) against
// the armed trajectory's startHint, and returns ok / fail. On
// success it also stashes `measuredStartPose` on the run so the
// analysis pipeline can rotate GT into the start frame.
//
// straight_2m's startHint is (x=-1.0, y=0.0, headingDeg=0), which
// makes it convenient to hand-pick in-tolerance and out-of-tolerance
// fixture values.

describe('TrajectoryRunner — preflight', () => {
    function armed(mocks, extra = {}) {
        // Arm a runner with preflight enabled at default-production
        // tolerances so the checks actually fire. Individual tests can
        // override via extra.preflightHeadingToleranceDeg /
        // preflightPositionToleranceM.
        const runner = makeRunner(mocks, {
            preflightHeadingToleranceDeg: 5,
            preflightPositionToleranceM: 0.05,
            ...extra,
        });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        return runner;
    }

    it('is skipped when both tolerances are null (default)', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({ ok: true, x: 0, y: 0, thetaDeg: 0 }));
        const runner = makeRunner(mocks, { snapshotFn });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });

        const res = await runner.preflight();
        assert.deepStrictEqual(res, { ok: true, skipped: 'disabled' });
        // snapshotFn must not be called — skipping is the whole point.
        assert.strictEqual(snapshotFn.mock.callCount(), 0);
    });

    it('is skipped when snapshotFn is null even with tolerances set', async () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks, {
            snapshotFn: null,
            preflightHeadingToleranceDeg: 5,
            preflightPositionToleranceM: 0.05,
        });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });

        const res = await runner.preflight();
        assert.deepStrictEqual(res, { ok: true, skipped: 'no_snapshot_fn' });
    });

    it('fails without calling snapshotFn when state is not armed', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({ ok: true, x: -1, y: 0, thetaDeg: 0 }));
        const runner = makeRunner(mocks, {
            snapshotFn,
            preflightHeadingToleranceDeg: 5,
            preflightPositionToleranceM: 0.05,
        });
        // Never armed — state is idle.
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /not_armed/);
        assert.strictEqual(snapshotFn.mock.callCount(), 0);
    });

    it('happy path: writes measuredStartPose and emits preflight_ok', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -1.01, y: 0.02, thetaDeg: 1.5,
        }));
        const runner = armed(mocks, { snapshotFn });
        const okEvents = [];
        runner.on('preflight_ok', (e) => okEvents.push(e));

        const res = await runner.preflight();
        assert.strictEqual(res.ok, true);
        assert.deepStrictEqual(res.measured, { x: -1.01, y: 0.02, thetaDeg: 1.5 });

        // Pose stashed on run so meta.json captures the start frame.
        assert.deepStrictEqual(
            runner.getRun().measuredStartPose,
            { x: -1.01, y: 0.02, thetaDeg: 1.5 },
        );
        // Event emitted for webServer fan-out.
        assert.strictEqual(okEvents.length, 1);
        assert.deepStrictEqual(okEvents[0].measured, { x: -1.01, y: 0.02, thetaDeg: 1.5 });
    });

    it('heading out of tolerance → preflight_failed, no measuredStartPose', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -1.0, y: 0.0, thetaDeg: 10,  // 10° off from expected 0°, tol 5°
        }));
        const runner = armed(mocks, { snapshotFn });
        const failEvents = [];
        runner.on('preflight_failed', (e) => failEvents.push(e));

        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /heading_out_of_tolerance/);
        assert.strictEqual(res.delta.thetaDeg, 10);
        // measuredStartPose NOT written on failure — otherwise the caller
        // could proceed to start() with a validated-looking pose.
        assert.strictEqual(runner.getRun().measuredStartPose, null);
        assert.strictEqual(failEvents.length, 1);
        assert.match(failEvents[0].reason, /heading_out_of_tolerance/);
    });

    it('heading wrap: +170° vs expected −170° → δ = −20° (pass at ±30° tol)', async () => {
        const mocks = makeMocks();
        // Build a custom run with headingDeg: -170 so we can test the wrap.
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -1.0, y: 0.0, thetaDeg: 170,
        }));
        const runner = makeRunner(mocks, {
            snapshotFn,
            preflightHeadingToleranceDeg: 30,
            preflightPositionToleranceM: 0.05,
        });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        // Patch startHint.headingDeg to -170 on the live run so we're
        // exercising the wrap. getRun() now returns a deep-copy per
        // H-V3 — mutations through it are intentionally NOT observed
        // by preflight — so we reach through the private `_run` field
        // directly. Kept as a reminder for any future test that wants
        // to spoof a startHint value: you must touch `_run`, not the
        // getRun() copy.
        runner._run.startHint.headingDeg = -170;

        const res = await runner.preflight();
        assert.strictEqual(res.ok, true, `expected pass, got fail: ${res.reason}`);
        // Wrapped delta should be -20° (170 - (-170) = 340 → -20 after wrap),
        // magnitude 20° < 30° tolerance.
    });

    it('position out of tolerance → preflight_failed', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -0.70, y: 0.0, thetaDeg: 0,   // 30 cm off, tol 5 cm
        }));
        const runner = armed(mocks, { snapshotFn });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /position_out_of_tolerance/);
        assert.ok(Math.abs(res.delta.distance - 0.30) < 1e-9);
    });

    it('heading near 180° off → backwards-placement hint', async () => {
        // Regression for MR B #11: when the robot is placed facing
        // backwards (≈180° off), the preflight reason should suggest
        // flipping the robot rather than "straighten".
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -1.0, y: 0.0, thetaDeg: 178,  // ~178° off from expected 0°
        }));
        const runner = armed(mocks, { snapshotFn });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /facing the wrong way/);
        assert.doesNotMatch(res.reason, /straighten the robot/);
    });

    it('position wildly outside workspace → homography-broken hint', async () => {
        // Regression for MR B #11: a measured pose far outside the
        // workspace envelope almost always means the overhead
        // homography has drifted. The message should call that out
        // instead of asking the operator to move the robot.
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: 12.4, y: -7.1, thetaDeg: 0,
        }));
        const runner = armed(mocks, { snapshotFn });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /homography likely broken/);
        assert.match(res.reason, /recalibrate camera/);
    });

    it('snapshotFn returns {ok: false, error} → preflight_failed: snapshot_failed', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: false, error: 'connection refused',
        }));
        const runner = armed(mocks, { snapshotFn });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /snapshot_failed.*connection refused/);
    });

    it('snapshotFn returns {ok: false} with no error → preflight_failed: marker_not_detected', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({ ok: false }));
        const runner = armed(mocks, { snapshotFn });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /marker_not_detected/);
    });

    it('snapshotFn throws → preflight_failed: snapshot_error', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.reject(new Error('network timeout')));
        const runner = armed(mocks, { snapshotFn });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /snapshot_error.*network timeout/);
    });

    it('non-numeric pose → preflight_failed', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: NaN, y: 0, thetaDeg: 0,
        }));
        const runner = armed(mocks, { snapshotFn });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /non-numeric pose/);
    });

    it('run swapped during snapshot await → run_changed_during_preflight', async () => {
        const mocks = makeMocks();
        // A snapshot function that "resolves" only when we manually
        // release the promise — lets us simulate the abort→re-arm that
        // happens inside the ~2.5 s snapshot window.
        let release;
        const snapshotFn = mock.fn(() => new Promise((resolve) => {
            release = () => resolve({ ok: true, x: -1.0, y: 0, thetaDeg: 0 });
        }));
        const runner = armed(mocks, { snapshotFn });

        // Kick off preflight. It will await inside snapshotFn.
        const preflightPromise = runner.preflight();
        // Now swap the run: abort → (synthetic) re-arm with a new runId.
        // abort() transitions armed → aborted → idle synchronously.
        runner.abort('test_swap');
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });  // new runId
        // Release the original snapshot.
        release();

        const res = await preflightPromise;
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /run_changed_during_preflight/);
        // The NEW run must not carry a measuredStartPose from the stale
        // measurement.
        assert.strictEqual(runner.getRun().measuredStartPose, null);
    });

    it('emits preflight_failed for all failure reasons', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -0.5, y: 0, thetaDeg: 20,  // both heading + position off
        }));
        const runner = armed(mocks, { snapshotFn });
        const events = [];
        runner.on('preflight_failed', (e) => events.push(e));

        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.strictEqual(events.length, 1);
        // Heading check fires first — the event carries the same reason
        // as the return value.
        assert.strictEqual(events[0].reason, res.reason);
    });

    it('fail-closed when heading tolerance is set but startHint.headingDeg is missing', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -1.0, y: 0.0, thetaDeg: 0,
        }));
        const runner = makeRunner(mocks, {
            snapshotFn,
            preflightHeadingToleranceDeg: 5,
            // no position tol — isolates the heading-missing branch
        });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        // Strip headingDeg off the run's startHint to simulate a debug
        // trajectory that forgot to set it.
        delete runner._run.startHint.headingDeg;

        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /heading_tolerance_configured_but_startHint\.headingDeg_missing/);
        assert.strictEqual(runner._run.measuredStartPose, null);
    });

    it('fail-closed when position tolerance is set but startHint.x/y is missing', async () => {
        const mocks = makeMocks();
        const snapshotFn = mock.fn(() => Promise.resolve({
            ok: true, x: -1.0, y: 0.0, thetaDeg: 0,
        }));
        const runner = makeRunner(mocks, {
            snapshotFn,
            preflightPositionToleranceM: 0.05,
        });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        delete runner._run.startHint.x;

        const res = await runner.preflight();
        assert.strictEqual(res.ok, false);
        assert.match(res.reason, /position_tolerance_configured_but_startHint\.x\/y_missing/);
    });

    it('does NOT emit preflight_ok when skipped (disabled tolerances)', async () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);  // no tolerances → disabled
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 });
        const okEvents = [];
        const failEvents = [];
        runner.on('preflight_ok', (e) => okEvents.push(e));
        runner.on('preflight_failed', (e) => failEvents.push(e));

        const res = await runner.preflight();
        assert.strictEqual(res.ok, true);
        // Skipped paths emit nothing — the `measured` field is absent
        // which is how the emit branch gates.
        assert.strictEqual(okEvents.length, 0);
        assert.strictEqual(failEvents.length, 0);
    });
});

describe('TrajectoryRunner — demo mode', () => {
    it('arm(demo=true) strips pauses from the upload', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'straight_2m', speed: 0.4, demo: true });

        const [, segments] = mocks.robot.loadTrajectory.mock.calls[0].arguments;
        assert.strictEqual(segments.filter(s => s.kind === 'pause').length, 0);
        // All motion segments survive (straight_2m has 4 translates).
        assert.strictEqual(segments.length, 4);
    });

    it('arm(demo=true) skips the logger entirely', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'straight_2m', speed: 0.4, demo: true });

        assert.strictEqual(mocks.logger.started, 0, 'no logger.start');
        assert.deepStrictEqual(mocks.logger.events, [], 'no events logged');
        // Sanity: getRun exposes the demo flag so the UI can branch.
        assert.strictEqual(runner.getRun().demo, true);
    });

    it('preflight() short-circuits to ok with skipped=demo', async () => {
        const mocks = makeMocks();
        // Configure tolerances so the normal path would call snapshotFn.
        // Demo short-circuit must happen BEFORE that call.
        const snapshotFn = mock.fn(async () => ({ ok: true, x: 0, y: 0, thetaDeg: 0 }));
        const runner = makeRunner(mocks, {
            snapshotFn,
            preflightHeadingToleranceDeg: 5,
            preflightPositionToleranceM: 0.05,
        });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4, demo: true });
        const res = await runner.preflight();
        assert.strictEqual(res.ok, true);
        assert.strictEqual(res.skipped, 'demo');
        assert.strictEqual(snapshotFn.mock.callCount(), 0, 'snapshot must not be called in demo');
    });

    it('traj_done completes straight to idle (skips awaiting_ground_truth)', async () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks, { now: () => 1_000_000 });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4, demo: true });
        runner.start();

        const completed = [];
        runner.on('completed', (e) => completed.push(e));

        // Demo strips pauses, so only `traj_done` fires — no intermediate
        // traj_paused events.
        mocks.robot.emit('traj_done', {
            type: 'traj_done',
            runId: runner.getRun()?.runId,
            pose: { x: 0, y: 0, th: 0 },
        });
        await flushMicrotasks();

        assert.strictEqual(runner.getState(), STATE.idle, 'demo goes straight back to idle');
        assert.strictEqual(completed.length, 1);
        assert.strictEqual(completed[0].demo, true);
        assert.strictEqual(completed[0].csvPath, null);
        assert.strictEqual(mocks.logger.stopped, 0, 'logger was never started, nothing to stop');
    });

    it('abort during a demo run works and writes no logger events', () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks);
        runner.arm({ trajectory: 'square_0m8_rotate', speed: 0.2, demo: true });
        runner.start();
        assert.strictEqual(runner.getState(), STATE.running);

        runner.abort('estop');
        assert.strictEqual(runner.getState(), STATE.idle);
        assert.strictEqual(mocks.robot.trajAbort.mock.callCount(), 1);
        assert.deepStrictEqual(mocks.logger.events, [], 'no events in demo abort');
    });

    it('non-demo run still exercises the logger + GT state (regression guard)', async () => {
        const mocks = makeMocks();
        const runner = makeRunner(mocks, { now: () => 1_000_000 });
        runner.arm({ trajectory: 'straight_2m', speed: 0.4 /* demo omitted */ });
        runner.start();
        await driveToCompletion(runner, mocks.robot, 'straight_2m', 0.4);

        assert.strictEqual(runner.getState(), STATE.awaitingGroundTruth);
        assert.ok(mocks.logger.started >= 1);
        assert.ok(mocks.logger.events.includes('exp_arm'));
        assert.ok(mocks.logger.events.includes('exp_end'));
    });
});
