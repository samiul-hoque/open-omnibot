#!/usr/bin/env node
// Strafe-left closed-loop debug experiment.
//
// Goal: capture exactly what the firmware's PID loop is doing per wheel
// while the robot is commanded to strafe left. Prior hand-driven tests
// showed that at the same wheel patterns direct PWM strafes correctly
// but the A-key (closed-loop {vx:0, vy:+0.1, w:0}) causes the two front
// wheels to saturate at ±255 in the wrong direction, producing a CCW
// rotation instead of a strafe.
//
// This script:
//   1. Connects directly to the robot's WebSocket (no server in between).
//   2. Resets encoders.
//   3. Sends {type:'cmd', vx:0, vy:+0.1, w:0} at 50 Hz for a fixed window.
//   4. Records all type:'log' messages (the firmware has a TEMP diagnostic
//      in pid_controller.cpp that prints per-wheel tgt/act/pwm once/sec
//      whenever a non-zero command is active).
//   5. Records all type:'sensors' messages so we can compute encoder
//      deltas and raw wheel velocities over the press.
//   6. Sends {type:'stop'} and prints a summary.
//
// IMPORTANT: stop the main server (`npm start`) before running this, or
// run it in a moment when nothing else is streaming cmd messages to the
// robot. Two clients both sending cmd at 50 Hz would fight each other.
//
// Usage:
//   ROBOT_IP=robot.local node evaluation/scripts/strafe_debug_experiment.mjs
//   node evaluation/scripts/strafe_debug_experiment.mjs --duration-ms 3000 --vy 0.10

import WebSocket from 'ws';
import { writeFileSync, mkdirSync } from 'fs';
import { join } from 'path';

const ROBOT_IP = process.env.ROBOT_IP || 'robot.local';
const ROBOT_URL = `ws://${ROBOT_IP}:80/ws`;

function parseArgs() {
    const args = { duration_ms: 3000, vy: 0.10, vx: 0.0, w: 0.0, cmd_hz: 50 };
    for (let i = 2; i < process.argv.length; i++) {
        const a = process.argv[i];
        if (a === '--duration-ms') args.duration_ms = parseInt(process.argv[++i], 10);
        else if (a === '--vy') args.vy = parseFloat(process.argv[++i]);
        else if (a === '--vx') args.vx = parseFloat(process.argv[++i]);
        else if (a === '--w') args.w = parseFloat(process.argv[++i]);
        else if (a === '--cmd-hz') args.cmd_hz = parseInt(process.argv[++i], 10);
        else if (a === '--help' || a === '-h') {
            console.log('strafe_debug_experiment.mjs [--duration-ms N] [--vy 0.10] [--vx 0.0] [--w 0.0] [--cmd-hz 50]');
            process.exit(0);
        }
    }
    return args;
}

const CLI = parseArgs();
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

const runStamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
const outDir = join('../experiments', `strafe_debug_${runStamp}`);
mkdirSync(outDir, { recursive: true });

const record = {
    robot_ip: ROBOT_IP,
    cli: CLI,
    started_at_iso: new Date().toISOString(),
    started_at_ms: Date.now(),
    window: { start_ms: null, end_ms: null },
    pid_logs: [],       // raw [log] lines from firmware
    sensor_start: null, // first sensor sample after reset
    sensor_end: null,   // last sensor sample before stop
    sensor_samples: [], // all in-window samples (kept short — summary printed)
};

function send(ws, obj) {
    ws.send(JSON.stringify(obj));
}

async function main() {
    console.log(`[exp] Connecting to ${ROBOT_URL}`);
    console.log(`[exp] cmd: vx=${CLI.vx} vy=${CLI.vy} w=${CLI.w} for ${CLI.duration_ms}ms at ${CLI.cmd_hz}Hz`);
    const ws = new WebSocket(ROBOT_URL);

    await new Promise((resolve, reject) => {
        ws.once('open', resolve);
        ws.once('error', reject);
    });
    console.log('[exp] Connected');

    // Message handler.
    ws.on('message', (raw) => {
        let msg;
        try { msg = JSON.parse(raw); } catch { return; }

        if (msg.type === 'log') {
            const line = `[log] ${msg.msg}`;
            console.log(line);
            record.pid_logs.push({ t_ms: Date.now(), msg: msg.msg });
        } else if (msg.type === 'sensors') {
            const snap = { t_ms: Date.now(), t_robot: msg.t, enc: msg.enc, vel: msg.vel };
            if (record.window.start_ms != null && record.window.end_ms == null) {
                record.sensor_samples.push(snap);
            }
            // Track first post-reset and latest-ever for summary.
            if (record.sensor_start == null && record.window.start_ms == null) {
                // Captured just before the press begins.
                record.sensor_start = snap;
            }
            record.sensor_end = snap;
        }
    });

    // Settle the connection and drain any queued sensor traffic.
    await sleep(500);

    // Reset encoders so deltas are clean.
    send(ws, { type: 'reset_encoders' });
    console.log('[exp] Sent reset_encoders');
    await sleep(500);

    // Mark pre-press snapshot.
    record.sensor_start = record.sensor_end; // latest sample post-reset
    record.window.start_ms = Date.now();
    console.log(`[exp] Starting ${CLI.duration_ms}ms press at ${new Date(record.window.start_ms).toISOString()}`);

    // Drive the command at cmd_hz. Firmware has a 500ms velocity timeout,
    // so we need to keep ticking.
    const periodMs = Math.max(1, Math.round(1000 / CLI.cmd_hz));
    const end = record.window.start_ms + CLI.duration_ms;
    while (Date.now() < end) {
        send(ws, { type: 'cmd', vx: CLI.vx, vy: CLI.vy, w: CLI.w });
        await sleep(periodMs);
    }
    record.window.end_ms = Date.now();
    console.log(`[exp] Press window ended`);

    // Clean stop.
    send(ws, { type: 'cmd', vx: 0, vy: 0, w: 0 });
    await sleep(200);
    send(ws, { type: 'stop' });
    await sleep(300);

    // Summary.
    record.ended_at_iso = new Date().toISOString();
    record.ended_at_ms = Date.now();

    const jsonPath = join(outDir, 'result.json');
    writeFileSync(jsonPath, JSON.stringify(record, null, 2) + '\n');
    console.log(`\n[exp] Wrote ${jsonPath}`);

    // Print a human summary.
    const startEnc = record.sensor_start?.enc;
    const endEnc = record.sensor_end?.enc;
    if (startEnc && endEnc && startEnc.length === 4 && endEnc.length === 4) {
        const labels = ['L1', 'R1', 'L2', 'R2'];
        const durS = (record.window.end_ms - record.window.start_ms) / 1000;
        console.log('\n[exp] Encoder deltas over press window:');
        for (let i = 0; i < 4; i++) {
            const d = endEnc[i] - startEnc[i];
            const radS = d * (2 * Math.PI) / 1092 / durS;
            const sign = d >= 0 ? '+' : '';
            console.log(`  ${labels[i]}: ${sign}${d} counts (${radS.toFixed(2)} rad/s mean)`);
        }
    } else {
        console.log('\n[exp] WARNING: did not capture start/end sensor samples.');
    }

    console.log(`\n[exp] Captured ${record.pid_logs.length} log line(s) from firmware.`);
    if (record.pid_logs.length === 0) {
        console.log('[exp] WARNING: no log lines received. Either the firmware diagnostic');
        console.log('[exp] is not flashed, wsLog has no WebSocket clients at the moment of');
        console.log('[exp] the call, or isCalibrationMode() was true during the press.');
    }

    ws.close();
}

main().catch(err => {
    console.error('[exp] FAILED:', err);
    process.exit(1);
});
