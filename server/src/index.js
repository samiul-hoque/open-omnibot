// ============================================
// Omni-2 Robot Control Server
// ============================================
//
// Main entry point for the robot control server.
// Connects to the robot via WebSocket, runs localization,
// and logs data for analysis.
//
// Usage:
//   npm start
//   npm run dev  (with auto-reload)
//

import http from 'node:http';
import { URLSearchParams } from 'node:url';

import { config } from './config.js';
import { RobotClient } from './robot/robotClient.js';
import { Odometry } from './localization/odometry.js';
import { FusionBasic } from './localization/fusionBasic.js';
import { DataLogger } from './logging/dataLogger.js';
import { WebServer } from './web/webServer.js';
import { TrajectoryRunner, STATE as EXP_STATE } from './experiments/trajectoryRunner.js';

// Ground-truth snapshot service (aruco_detector.py). The trajectory runner
// calls fetchSnapshot() at each waypoint; the call resolves whether the
// service is up (normal result), down (ok:false + error), or slow (8 s
// timeout). The runner converts the result into a waypoint record either
// way so a down service doesn't stop a run.
const GT_SNAPSHOT_URL = 'http://127.0.0.1:5055';

function fetchSnapshot({ runId, trajectory, label }) {
    const params = new URLSearchParams({
        runId: String(runId || ''),
        run_id: String(runId || ''),  // service accepts snake_case
        trajectory: String(trajectory || ''),
        label: String(label || ''),
    });
    const url = `${GT_SNAPSHOT_URL}/snapshot?${params.toString()}`;
    return new Promise((resolve) => {
        const req = http.get(url, { timeout: 8000 }, (res) => {
            let body = '';
            res.setEncoding('utf-8');
            res.on('data', (chunk) => { body += chunk; });
            res.on('end', () => {
                let parsed;
                try { parsed = JSON.parse(body); }
                catch (err) {
                    resolve({ ok: false, error: `invalid JSON from snapshot service: ${err.message}` });
                    return;
                }
                const ok = res.statusCode === 200 && parsed.detected === true;
                resolve({
                    ok,
                    x: parsed.x ?? null,
                    y: parsed.y ?? null,
                    theta: parsed.theta ?? null,
                    thetaDeg: parsed.theta_deg ?? null,
                    sidePx: parsed.side_px ?? null,
                    imagePath: parsed.image_path ?? null,
                    error: ok ? null : (parsed.error || `snapshot service returned ${res.statusCode}`),
                });
            });
        });
        req.on('timeout', () => {
            req.destroy();
            resolve({ ok: false, error: 'snapshot service did not respond within 8s' });
        });
        req.on('error', (err) => {
            resolve({ ok: false, error: `snapshot service unreachable: ${err.message} (is aruco_detector.py running?)` });
        });
    });
}

// ============================================
// Terminal Cleanup
// ============================================

let cleanupDone = false;

function cleanup() {
    if (cleanupDone) return;
    cleanupDone = true;

    // Restore terminal to normal mode
    if (process.stdin.isTTY && process.stdin.isRaw) {
        process.stdin.setRawMode(false);
    }

    // Stop robot and disconnect
    robot.stop();
    robot.disconnect();
    logger.stop();
    webServer.stop();
}

// Handle various exit scenarios
process.on('exit', cleanup);
process.on('SIGINT', () => {
    console.log('\nReceived SIGINT, shutting down...');
    cleanup();
    process.exit(0);
});
process.on('SIGTERM', () => {
    console.log('\nReceived SIGTERM, shutting down...');
    cleanup();
    process.exit(0);
});
process.on('uncaughtException', (err) => {
    console.error('\nUncaught exception:', err);
    cleanup();
    process.exit(1);
});
process.on('unhandledRejection', (reason) => {
    console.error('\nUnhandled rejection:', reason);
    cleanup();
    process.exit(1);
});

// ============================================
// Initialize Components
// ============================================

const robot = new RobotClient();
const logger = new DataLogger();

function createLocalization(tier) {
    switch (tier) {
    // Tier 0 still uses encoder odometry for LOGGING — the firmware is
    // running open-loop (no pose feedback used for control), but we
    // record what the odometry would predict so the analysis pipeline
    // can compare tier 0 / 1 / 2 side-by-side on identical metrics.
    case 0: return new Odometry(0);
    case 1: return new Odometry(1);
    case 2: return new FusionBasic();
    default: return new Odometry(1);
    }
}

const TIER_NAMES = {
    0: 'Tier 0: Open-Loop (hardcoded PWM table, no feedback)',
    1: 'Tier 1: Dead Reckoning (encoders only)',
    2: 'Tier 2: Encoder + IMU Fusion',
};

// Select localization based on config
let localization = createLocalization(config.localizationTier);
console.log(`Using ${TIER_NAMES[config.localizationTier] || 'Unknown tier'}`);

// Single entry point for tier switching — used by keyboard, web UI, and any future source.
// Updates config, swaps localization instance, logs the event, and syncs webServer.
// Refuses while an experiment run is active so we don't change the pose
// source mid-trajectory and break the CSV's tier column / pose semantics.
function switchTier(tier) {
    if (tier !== 0 && tier !== 1 && tier !== 2) {
        console.warn(`Refusing invalid tier ${tier} (valid: 0, 1, 2)`);
        return;
    }
    const expState = experimentRunner.getState();
    if (expState !== EXP_STATE.idle) {
        console.warn(`Refusing tier switch while experiment is ${expState}`);
        return;
    }
    config.localizationTier = tier;
    localization = createLocalization(tier);
    webServer.localization = localization;
    experimentRunner._localization = localization;
    console.log(`Switched to ${TIER_NAMES[tier] || 'Unknown tier'}`);
    logger.logEvent(`tier_change:${tier}`);
}

// Current command state.
// `lastActiveCmdTs` timestamps the most recent non-zero setCommand. The 100 ms
// keepalive below uses it to enforce a safety timeout: if a non-zero command
// has not been refreshed within CMD_STALE_MS, it is forced to zero so a crashed
// client (WS reset mid-trajectory) cannot leave the robot driving into a wall.
let currentCommand = { vx: 0, vy: 0, omega: 0 };
let lastActiveCmdTs = 0;
const CMD_STALE_MS = 750;

function setCommand(cmd) {
    currentCommand = cmd;
    if (Math.abs(cmd.vx) > 0 || Math.abs(cmd.vy) > 0 || Math.abs(cmd.omega) > 0) {
        lastActiveCmdTs = Date.now();
    }
}

// Experiment runner — drives scripted trajectories by mutating currentCommand.
// Uses ~10 event listeners (4 here + 6 in webServer); raise the cap to avoid
// Node.js MaxListenersExceededWarning if more are added in the future.
const experimentRunner = new TrajectoryRunner({
    robot: robot,
    localization: localization,
    logger: logger,
    setCommand: setCommand,
    getTier: () => config.localizationTier,
    // Per-waypoint snapshot capture + encoder-pose pairing for the waypoints
    // list. If the snapshot service is down the runner just logs a null-gt
    // entry and continues — the run doesn't abort.
    snapshotFn: fetchSnapshot,
    getPose: () => localization.getPose(),
    // Placement-safety preflight: at experiment_start, snap the overhead
    // camera and compare against the startHint. Reject the run if the robot
    // isn't within ±5° of the intended heading or ±5 cm of the tape.
    //
    // Tightened from the initial 15° / 20 cm on 2026-04-19 after an audit
    // showed the looser bounds allowed worst-case placements that pushed
    // trajectory envelopes outside the workspace (e.g. straight_2m at +15°
    // heading puts the endpoint ~52 cm past the +Y edge; +20 cm Y on a
    // square start moves the north-corner 5 cm past the edge). With
    // ±5° / ±5 cm, the taped placements we verified today (all ≤4 cm,
    // ≤5° errors) still pass, and the worst-case envelope drift over the
    // full trajectory stays inside the 15 cm safety margin.
    //
    // Small within-tolerance misplacement is compensated by the analysis
    // pipeline using the measured start pose (Option C — run's
    // measuredStartPose field in meta.json; see replay.py).
    preflightHeadingToleranceDeg: 5,
    preflightPositionToleranceM: 0.05,
});
experimentRunner.setMaxListeners(20);

experimentRunner.on('armed', (e) => console.log(`[EXP] armed ${e.runId}`));
experimentRunner.on('started', (e) => console.log(`[EXP] started ${e.runId}`));
experimentRunner.on('completed', (e) => console.log(`[EXP] completed ${e.runId} (${e.durationMs} ms)`));
experimentRunner.on('aborted', (e) => console.log(`[EXP] aborted ${e.runId}: ${e.reason}`));

// Web server with shared state
const webServer = new WebServer({
    robot: robot,
    localization: localization,
    logger: logger,
    experimentRunner: experimentRunner,
    getCommand: () => currentCommand,
    setCommand: setCommand,
    setTier: (tier) => switchTier(tier),
    setImuWeight: (weight) => {
        config.fusion.imuWeight = weight;
        if (localization.setImuWeight) localization.setImuWeight(weight);
        console.log(`IMU weight set to ${weight}`);
    },
});

// ============================================
// Robot Event Handlers
// ============================================

robot.on('connected', () => {
    console.log('\n=== Robot Connected ===\n');

    // Reset localization on connect
    localization.reset();

    // Start logging with session metadata
    logger.start();
    logger.logEvent('connect');

    // Notify web UI
    webServer.broadcastConnectionStatus(true);
});

robot.on('log', (msg) => {
    console.log(`[ROBOT] ${msg}`);
    webServer.broadcastLog(msg);
});

robot.on('ack', (msg) => {
    // Forward robot acks (e.g. set_heading_hold confirmation) to browsers
    // so UI state can sync with the firmware's authoritative state.
    webServer.broadcastAck(msg);
});

robot.on('info', (info) => {
    webServer.broadcastRobotInfo(info);
});

robot.on('pong_cal', (msg) => {
    webServer.broadcastPongCal(msg);
});

robot.on('motor_cal_result', (msg) => {
    webServer.broadcastMotorCalResult(msg);
    if (msg.success) {
        const fwd = msg.gainsFwd.map(g => Number(g).toFixed(4)).join(',');
        const rev = msg.gainsRev.map(g => Number(g).toFixed(4)).join(',');
        logger.logEvent(`motor_cal_done:fwd=[${fwd}]_rev=[${rev}]`);
    } else {
        logger.logEvent('motor_cal_failed');
    }
});

// Self-test events → broadcast to browsers
robot.on('self_test_result', (msg) => {
    webServer.broadcastSelfTestResult(msg);
});
robot.on('self_test_complete', (msg) => {
    webServer.broadcastSelfTestComplete(msg);
});

robot.on('disconnected', () => {
    console.log('\n=== Robot Disconnected ===\n');
    // Abort any in-flight experiment that still needs the robot. The
    // `awaiting_ground_truth` state is operator-only (user is walking to
    // the grid with a ruler) and doesn't need the robot to be connected;
    // aborting here would throw away an otherwise clean trajectory because
    // of a transient WiFi dropout during the measurement walk.
    const expState = experimentRunner.getState();
    if (expState !== 'idle' && expState !== 'awaiting_ground_truth') {
        try {
            experimentRunner.abort('robot_disconnect');
        } catch (err) {
            console.error('Failed to abort experiment on disconnect:', err);
        }
    }
    // Keep the logger open while waiting for GT so submitGroundTruth()
    // can finalize the sidecar. Otherwise close it as before.
    if (expState !== 'awaiting_ground_truth') {
        logger.logEvent('disconnect');
        logger.stop();
    } else {
        logger.logEvent('disconnect_during_gt_wait');
    }
    webServer.broadcastConnectionStatus(false);
});

// Track the firmware IMU stuck-read watchdog across sensor ticks so we
// can react exactly once to a false→true transition. Firmware latches
// the flag until reboot, so after the first fire we'd otherwise repeat
// the abort/log actions every 50 ms for the rest of the session.
let prevImuStuck = false;

robot.on('sensors', (data) => {
    const pose = localization.update(data);

    const imuStuck = data.imuStuck === true;
    if (imuStuck && !prevImuStuck) {
        // Durable event in the CSV stream + sticky metadata flag, so a
        // downstream analysis pass can reject tainted sessions without
        // re-reading the event column row by row.
        logger.logEvent('imu_stuck');
        logger.markImuStuck();
        console.warn('\n⚠ IMU stuck detected — firmware watchdog tripped. ' +
                     'Robot needs a power cycle to recover.\n');

        // Abort an active experiment immediately. fw_pose is polluted by
        // the complementary filter chasing the frozen IMU reference, so
        // any in-flight run is already recording garbage — better to
        // kill it now than discover tainted data after the fact. The
        // awaiting_ground_truth state is post-motion (data is already
        // captured) so we don't abort it; the metadata flag above is
        // enough for the analysis pass to reject the run.
        const expState = experimentRunner.getState();
        if (expState === EXP_STATE.armed || expState === EXP_STATE.running) {
            try {
                experimentRunner.abort('imu_stuck');
            } catch (err) {
                console.error('Failed to abort experiment on IMU stuck:', err);
            }
        }
    }
    prevImuStuck = imuStuck;

    logger.log({
        sensorData: data,
        pose: pose,
        firmwarePose: data.firmwarePose || null,
        command: currentCommand,
    });
});

// ============================================
// Status Display
// ============================================

function printStatus() {
    const sensors = robot.getSensors();
    if (!sensors) return;

    const pose = localization.getPose();
    const logStatus = logger.getStatus();

    console.log('─'.repeat(60));
    console.log(`Pose: x=${pose.x.toFixed(3)}m, y=${pose.y.toFixed(3)}m, θ=${pose.thetaDeg.toFixed(1)}°`);
    console.log(`Encoders: [${sensors.encoders.join(', ')}]`);
    console.log(`IMU Yaw: ${sensors.imu.yaw.toFixed(1)}° | Cal: S=${sensors.calibration.sys} G=${sensors.calibration.gyro}`);
    console.log(`Cmd: vx=${currentCommand.vx.toFixed(2)} vy=${currentCommand.vy.toFixed(2)} ω=${currentCommand.omega.toFixed(2)}`);
    console.log(`Log: ${logStatus.rowCount} rows | Tier: ${pose.tier}`);
}

// ============================================
// Simple Keyboard Control (for testing)
// ============================================

// Enable raw mode for single keypress detection
if (process.stdin.isTTY) {
    process.stdin.setRawMode(true);
    process.stdin.resume();
}

const SPEED = config.control.speed;
const TURN_SPEED = config.control.turnSpeed;
const KEY_RELEASE_TIMEOUT = config.control.keyReleaseTimeout;
let keyReleaseTimer = null;

console.log('\n========================================');
console.log('   Omni-2 Robot Control Server');
console.log('========================================\n');
console.log('Keyboard Controls:');
console.log('  W/S     - Forward/Backward');
console.log('  A/D     - Strafe Left/Right');
console.log('  Q/E     - Rotate CCW/CW');
console.log('  1/2     - Switch localization tier');
console.log('  SPACE   - Stop');
console.log('  R       - Reset encoders & pose');
console.log('  P       - Print status');
console.log('  Ctrl+C  - Exit');
console.log('');

process.stdin.on('data', (key) => {
    const char = key.toString().toLowerCase();

    // Ctrl+C to exit (handled by SIGINT, but keep as fallback)
    if (key[0] === 3) {
        console.log('\nExiting...');
        cleanup();
        process.exit(0);
    }
    
    switch (char) {
    case 'w':
        setCommand({ vx: SPEED, vy: 0, omega: 0 });
        break;
    case 's':
        setCommand({ vx: -SPEED, vy: 0, omega: 0 });
        break;
    case 'a':
        setCommand({ vx: 0, vy: SPEED, omega: 0 });
        break;
    case 'd':
        setCommand({ vx: 0, vy: -SPEED, omega: 0 });
        break;
    case 'q':
        setCommand({ vx: 0, vy: 0, omega: TURN_SPEED });
        break;
    case 'e':
        setCommand({ vx: 0, vy: 0, omega: -TURN_SPEED });
        break;
    case ' ':
        setCommand({ vx: 0, vy: 0, omega: 0 });
        robot.stop();
        if (experimentRunner.getState() !== 'idle') {
            experimentRunner.abort('estop');
        }
        console.log('STOP');
        break;
    case 'r':
        robot.resetEncoders();
        localization.reset();
        console.log('Reset encoders and pose');
        break;
    case 'p':
        printStatus();
        break;
    case '0':
    case '1':
    case '2':
        switchTier(parseInt(char, 10));
        break;
    }
    
    // Send command to robot and reset release timer
    if ('wasdqe'.includes(char)) {
        robot.setVelocity(currentCommand.vx, currentCommand.vy, currentCommand.omega);
        clearTimeout(keyReleaseTimer);
        keyReleaseTimer = setTimeout(() => {
            setCommand({ vx: 0, vy: 0, omega: 0 });
            robot.setVelocity(0, 0, 0);
        }, KEY_RELEASE_TIMEOUT);
    }
});

// ============================================
// Main Loop
// ============================================

// Status print interval
setInterval(() => {
    if (robot.isConnected()) {
        printStatus();
    }
}, config.timing.statusInterval);

// Command refresh (keep-alive) — only for teleop. Skip during firmware
// trajectory execution to avoid unnecessary WiFi traffic (firmware
// ignores cmd messages while a trajectory is running).
//
// Safety: if a non-zero command hasn't been refreshed within CMD_STALE_MS,
// force it to zero. Guards against a crashed WS client leaving the robot
// driving (see 2026-04-18 incident — script raised ConnectionResetError
// mid-trajectory, the keep-alive kept replaying vx=0.15, robot hit wall).
let _staleWarned = false;
setInterval(() => {
    if (robot.isConnected() && experimentRunner.getState() !== 'running') {
        const cmdActive =
            Math.abs(currentCommand.vx) > 0 ||
            Math.abs(currentCommand.vy) > 0 ||
            Math.abs(currentCommand.omega) > 0;
        if (cmdActive && Date.now() - lastActiveCmdTs > CMD_STALE_MS) {
            if (!_staleWarned) {
                console.warn(`[safety] cmd stale > ${CMD_STALE_MS}ms — forcing stop`);
                _staleWarned = true;
            }
            currentCommand = { vx: 0, vy: 0, omega: 0 };
            if (logger) logger.logEvent('cmd_timeout_stop');
        } else if (!cmdActive) {
            _staleWarned = false;
        }
        robot.setVelocity(currentCommand.vx, currentCommand.vy, currentCommand.omega);
    }
}, 100);

// ============================================
// Start
// ============================================

console.log(`Connecting to robot at ${config.robot.ip}...`);
console.log('Update config.js with your robot\'s IP address if needed.\n');

// Start web server
webServer.start();

robot.connect();
