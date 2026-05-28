#!/usr/bin/env node
// Motor calibration before/after experiment (multi-run).
//
// Connects to the robot over WebSocket and runs N cycles of:
//   reset gains to unity
//   -> BEFORE window (motors open-loop at fixed PWM)
//   -> auto-calibration state machine
//   -> AFTER  window (motors open-loop at the same fixed PWM, now
//      with the freshly-computed gains applied in reporting)
//
// Odd-numbered runs drive +PWM, even-numbered runs drive -PWM. The
// alternation lets a robot sitting on a stationary bench stay roughly
// in place across the campaign and also exercises both the forward
// and reverse gains produced by the auto-calibration routine.
//
// A single phases.json sidecar records window timestamps and computed
// gains for every run so the plotter can slice the server's CSV.
//
// Assumes the server is already running (so its DataLogger is writing
// to server/logs). This script does not touch the server — it is a
// separate WS client to the robot.

import WebSocket from 'ws';
import { writeFileSync, mkdirSync } from 'fs';
import { join } from 'path';

const ROBOT_IP = process.env.ROBOT_IP || 'robot.local';
const ROBOT_URL = `ws://${ROBOT_IP}:80/ws`;

// ---- CLI ------------------------------------------------------------------
function parseArgs() {
    const args = { runs: 1, pwm: 160, window_ms: 10_000 };
    for (let i = 2; i < process.argv.length; i++) {
        const a = process.argv[i];
        if (a === '--runs') args.runs = parseInt(process.argv[++i], 10);
        else if (a === '--pwm') args.pwm = parseInt(process.argv[++i], 10);
        else if (a === '--window-ms') args.window_ms = parseInt(process.argv[++i], 10);
        else if (a === '--help' || a === '-h') {
            console.log('motor_cal_experiment.mjs [--runs N] [--pwm P] [--window-ms MS]');
            process.exit(0);
        }
    }
    if (!Number.isFinite(args.runs) || args.runs < 1) {
        throw new Error(`Invalid --runs ${args.runs}`);
    }
    return args;
}

const CLI = parseArgs();
const TICK_MS = 100;
const PRE_SETTLE_MS = 1_500;
const POST_WINDOW_STOP_MS = 2_500;
const POST_CAL_SETTLE_MS = 2_500;
const INTER_RUN_SETTLE_MS = 2_000;
const CAL_TIMEOUT_MS = 60_000;

const runStamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
const outDir = join('../experiments', `motor_cal_${runStamp}`);
mkdirSync(outDir, { recursive: true });
const phasesPath = join(outDir, 'phases.json');

const phases = {
    robot_ip: ROBOT_IP,
    pwm: CLI.pwm,
    window_ms: CLI.window_ms,
    run_count: CLI.runs,
    started_at_iso: new Date().toISOString(),
    started_at_ms: Date.now(),
    runs: [],
};

const sleep = (ms) => new Promise(r => setTimeout(r, ms));

function send(ws, obj) {
    ws.send(JSON.stringify(obj));
}

async function runWindow(ws, label, pwmSigned, runRecord) {
    console.log(`[exp] ${label} window: PWM ${pwmSigned} for ${CLI.window_ms}ms`);
    runRecord[label].start_ms = Date.now();
    runRecord[label].pwm = pwmSigned;
    const end = Date.now() + CLI.window_ms;
    while (Date.now() < end) {
        send(ws, { type: 'motor_test', motor: 4, pwm: pwmSigned });
        await sleep(TICK_MS);
    }
    runRecord[label].end_ms = Date.now();
    // Ramp PWM back to 0 explicitly (slew-rate limiter) then hard stop.
    for (let i = 0; i < 10; i++) {
        send(ws, { type: 'motor_test', motor: 4, pwm: 0 });
        await sleep(TICK_MS);
    }
    send(ws, { type: 'stop' });
    await sleep(POST_WINDOW_STOP_MS);
}

async function runCalibration(ws, runRecord) {
    console.log('[exp]   starting auto-calibration');
    runRecord.cal.start_ms = Date.now();

    const resultPromise = new Promise((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('Calibration timed out')), CAL_TIMEOUT_MS);
        const onMsg = (raw) => {
            let msg;
            try { msg = JSON.parse(raw); } catch { return; }
            if (msg.type === 'motor_cal_result') {
                clearTimeout(timeout);
                ws.off('message', onMsg);
                resolve(msg);
            }
        };
        ws.on('message', onMsg);
    });

    send(ws, { type: 'start_motor_cal' });
    const result = await resultPromise;
    runRecord.cal.end_ms = Date.now();
    runRecord.cal.success = result.success;
    runRecord.calibrated_gains = { fwd: result.gainsFwd, rev: result.gainsRev };
    console.log(`[exp]   cal done: success=${result.success} fwd=[${result.gainsFwd.map(v => v.toFixed(3)).join(', ')}] rev=[${result.gainsRev.map(v => v.toFixed(3)).join(', ')}]`);
    return result;
}

async function oneRun(ws, runIndex) {
    // Odd runs forward, even runs reverse — keeps a bench-mounted robot
    // in place and exercises both gain directions evenly.
    const sign = (runIndex % 2 === 0) ? 1 : -1;
    const pwmSigned = sign * CLI.pwm;
    console.log(`\n=== run ${runIndex + 1}/${CLI.runs} (pwm=${pwmSigned}) ===`);

    const runRecord = {
        index: runIndex,
        direction: sign > 0 ? 'forward' : 'reverse',
        baseline_gains: { fwd: [1, 1, 1, 1], rev: [1, 1, 1, 1] },
        calibrated_gains: null,
        before: { start_ms: null, end_ms: null, pwm: null },
        after: { start_ms: null, end_ms: null, pwm: null },
        cal: { start_ms: null, end_ms: null, success: null },
    };

    // Reset to unity gains so BEFORE is truly uncalibrated each run.
    send(ws, { type: 'set_motor_gains', gainsFwd: [1, 1, 1, 1], gainsRev: [1, 1, 1, 1] });
    await sleep(PRE_SETTLE_MS);

    await runWindow(ws, 'before', pwmSigned, runRecord);
    await runCalibration(ws, runRecord);
    await sleep(POST_CAL_SETTLE_MS);
    await runWindow(ws, 'after', pwmSigned, runRecord);

    phases.runs.push(runRecord);
    // Persist after every run so a crash partway through still leaves
    // usable data for the runs that completed.
    writeFileSync(phasesPath, JSON.stringify(phases, null, 2) + '\n');
}

async function main() {
    console.log(`[exp] Connecting to ${ROBOT_URL}`);
    console.log(`[exp] runs=${CLI.runs} pwm=${CLI.pwm} window_ms=${CLI.window_ms}`);
    const ws = new WebSocket(ROBOT_URL);

    await new Promise((resolve, reject) => {
        ws.once('open', resolve);
        ws.once('error', reject);
    });
    console.log('[exp] Connected');

    ws.on('message', (raw) => {
        let msg;
        try { msg = JSON.parse(raw); } catch { return; }
        if (msg.type === 'log') console.log(`[robot] ${msg.msg}`);
    });

    try {
        for (let i = 0; i < CLI.runs; i++) {
            await oneRun(ws, i);
            if (i < CLI.runs - 1) await sleep(INTER_RUN_SETTLE_MS);
        }

        send(ws, { type: 'stop' });
        await sleep(500);

        phases.ended_at_iso = new Date().toISOString();
        phases.ended_at_ms = Date.now();

        writeFileSync(phasesPath, JSON.stringify(phases, null, 2) + '\n');
        console.log(`\n[exp] Wrote ${phasesPath}`);
    } finally {
        ws.close();
    }
}

main().catch(err => {
    console.error('[exp] FAILED:', err);
    writeFileSync(phasesPath, JSON.stringify({ ...phases, error: String(err) }, null, 2) + '\n');
    process.exit(1);
});
