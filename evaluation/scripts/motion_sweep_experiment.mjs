#!/usr/bin/env node
// Motion-sweep experiment.
//
// Exercises the six movement commands (W/S forward-backward, A/D
// strafe-left/right, Q/E rotate-CCW/CW) at several speeds with multiple
// repeats per (command, speed) cell. For every run we:
//
//   - reset encoders
//   - send {type:'cmd', vx, vy, w} at 50 Hz for a fixed window
//   - capture per-cycle PID diagnostics via wsLog
//   - record encoder deltas and compute mean rad/s per wheel
//   - compare actual to IK-expected (tracking error)
//
// Output: results.json + results.csv under
// evaluation/experiments/motion_sweep_<stamp>/ plus a printed summary.
//
// Assumes the server is NOT running (we own the cmd stream at 50 Hz).
// Usage:
//   ROBOT_IP=robot.local node evaluation/scripts/motion_sweep_experiment.mjs
//   node evaluation/scripts/motion_sweep_experiment.mjs --runs 3 --duration-ms 2000

import WebSocket from 'ws';
import { writeFileSync, mkdirSync } from 'fs';
import { join } from 'path';

const ROBOT_IP = process.env.ROBOT_IP || 'robot.local';
const ROBOT_URL = `ws://${ROBOT_IP}:80/ws`;

// ---- Physical constants (must match firmware config.h) --------------------
const WHEEL_RADIUS = 0.04;            // m
const LX = 0.1175;                    // m, half track width
const LY = 0.0953;                    // m, half wheelbase
const L_SUM = LX + LY;
const COUNTS_PER_WHEEL_REV = 1092;

// ---- CLI ------------------------------------------------------------------
function parseArgs() {
    const args = {
        runs: 2,
        duration_ms: 2000,
        cmd_hz: 50,
        // Linear speeds (m/s) and angular speeds (rad/s).
        lin_speeds: [0.05, 0.10, 0.20],
        ang_speeds: [0.20, 0.40, 0.80],
    };
    for (let i = 2; i < process.argv.length; i++) {
        const a = process.argv[i];
        if (a === '--runs') args.runs = parseInt(process.argv[++i], 10);
        else if (a === '--duration-ms') args.duration_ms = parseInt(process.argv[++i], 10);
        else if (a === '--cmd-hz') args.cmd_hz = parseInt(process.argv[++i], 10);
        else if (a === '--lin-speeds') args.lin_speeds = process.argv[++i].split(',').map(parseFloat);
        else if (a === '--ang-speeds') args.ang_speeds = process.argv[++i].split(',').map(parseFloat);
        else if (a === '--help' || a === '-h') {
            console.log('motion_sweep_experiment.mjs [--runs N] [--duration-ms MS] [--cmd-hz HZ]');
            console.log('                            [--lin-speeds 0.05,0.10,0.20] [--ang-speeds 0.20,0.40,0.80]');
            process.exit(0);
        }
    }
    return args;
}

const CLI = parseArgs();
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

const runStamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
const outDir = join('../experiments', `motion_sweep_${runStamp}`);
mkdirSync(outDir, { recursive: true });

// ---- Command matrix -------------------------------------------------------
// Each entry: { key, label, make(speed) -> {vx, vy, w} }
//
// Note on indexing: the firmware's WebSocket broadcast of `enc` presents the
// external convention [L1, R1, R2, L2] (lines 609/622 of websocket_server.cpp
// permute internal counts[3] into wire slot 2 and counts[2] into slot 3).
// So msg.enc[0..3] == [L1, R1, R2, L2]. We work in that external convention
// throughout, and compute IK-expected wheel speeds in the same order.
const COMMANDS = [
    { key: 'W', label: 'forward',     axis: 'lin', make: (s) => ({ vx:  s, vy:  0, w: 0 }) },
    { key: 'S', label: 'backward',    axis: 'lin', make: (s) => ({ vx: -s, vy:  0, w: 0 }) },
    { key: 'A', label: 'strafe_left', axis: 'lin', make: (s) => ({ vx:  0, vy:  s, w: 0 }) },
    { key: 'D', label: 'strafe_right',axis: 'lin', make: (s) => ({ vx:  0, vy: -s, w: 0 }) },
    { key: 'Q', label: 'rotate_ccw',  axis: 'ang', make: (s) => ({ vx:  0, vy:  0, w:  s }) },
    { key: 'E', label: 'rotate_cw',   axis: 'ang', make: (s) => ({ vx:  0, vy:  0, w: -s }) },
];

// Pairs of opposite-direction commands. When run in (a, b, a, b, ...) order
// at matched speed, each pair's net displacement cancels — important on the
// ground so a long repeated-direction block doesn't drift the robot into a
// wall. On the stand the pairing has no physical effect.
const COMMAND_PAIRS = [
    [COMMANDS[0], COMMANDS[1]],  // W / S
    [COMMANDS[2], COMMANDS[3]],  // A / D
    [COMMANDS[4], COMMANDS[5]],  // Q / E
];

// IK-expected wheel speeds for a given body command, in the external
// [L1, R1, R2, L2] order so they can be compared against msg.enc deltas
// directly.
function ikExpected({ vx, vy, w }) {
    const r = WHEEL_RADIUS;
    const L = L_SUM;
    const omega_L1 = (vx + vy - L * w) / r;
    const omega_R1 = (vx - vy + L * w) / r;
    const omega_R2 = (vx + vy + L * w) / r;
    const omega_L2 = (vx - vy - L * w) / r;
    // External wire convention:
    return [omega_L1, omega_R1, omega_R2, omega_L2];
}

function countsToRadS(dCounts, durS) {
    return (dCounts / COUNTS_PER_WHEEL_REV) * 2 * Math.PI / durS;
}

// ---- Single-run primitive -------------------------------------------------
async function runOne(ws, cmdSpec, speed) {
    const cmd = cmdSpec.make(speed);
    const expected = ikExpected(cmd);

    const pidLogs = [];
    const sensorSamples = [];
    const onMsg = (raw) => {
        let msg;
        try { msg = JSON.parse(raw); } catch { return; }
        if (msg.type === 'log' && typeof msg.msg === 'string' && msg.msg.startsWith('PID ')) {
            pidLogs.push({ t_ms: Date.now(), msg: msg.msg });
        } else if (msg.type === 'sensors') {
            sensorSamples.push({ t_ms: Date.now(), enc: msg.enc, vel: msg.vel });
        }
    };
    ws.on('message', onMsg);

    // Reset encoders and settle.
    ws.send(JSON.stringify({ type: 'reset_encoders' }));
    await sleep(400);

    // Capture the post-reset sample as our start snapshot.
    const startSample = sensorSamples[sensorSamples.length - 1];
    const startEnc = startSample ? startSample.enc.slice() : null;

    // Drive the command at cmd_hz Hz for duration_ms.
    const periodMs = Math.max(1, Math.round(1000 / CLI.cmd_hz));
    const pressStart = Date.now();
    const pressEnd = pressStart + CLI.duration_ms;
    while (Date.now() < pressEnd) {
        ws.send(JSON.stringify({ type: 'cmd', ...cmd }));
        await sleep(periodMs);
    }
    const pressStop = Date.now();

    // Stop + brief settle.
    ws.send(JSON.stringify({ type: 'cmd', vx: 0, vy: 0, w: 0 }));
    await sleep(150);
    ws.send(JSON.stringify({ type: 'stop' }));
    await sleep(300);

    // Capture end snapshot (last sensor sample we received during the press).
    // Find the newest sample whose timestamp is within [pressStart, pressStop].
    const endSample = [...sensorSamples].reverse().find(
        s => s.t_ms >= pressStart && s.t_ms <= pressStop
    );
    const endEnc = endSample ? endSample.enc.slice() : null;

    ws.off('message', onMsg);

    const durS = (pressStop - pressStart) / 1000;
    const actual = [null, null, null, null];
    if (startEnc && endEnc) {
        for (let i = 0; i < 4; i++) {
            actual[i] = countsToRadS(endEnc[i] - startEnc[i], durS);
        }
    }

    return {
        cmd_spec: { key: cmdSpec.key, label: cmdSpec.label },
        speed,
        cmd,
        expected_rad_s: expected,
        actual_rad_s: actual,
        track_err_rad_s: actual.map((a, i) => a == null ? null : a - expected[i]),
        pid_logs: pidLogs,
        duration_s: durS,
        start_enc: startEnc,
        end_enc: endEnc,
    };
}

// ---- Main -----------------------------------------------------------------
async function main() {
    console.log(`[exp] Connecting to ${ROBOT_URL}`);
    console.log(`[exp] runs=${CLI.runs} duration=${CLI.duration_ms}ms cmd_hz=${CLI.cmd_hz}`);
    console.log(`[exp] lin_speeds=${CLI.lin_speeds} ang_speeds=${CLI.ang_speeds}`);
    const ws = new WebSocket(ROBOT_URL);
    await new Promise((resolve, reject) => {
        ws.once('open', resolve);
        ws.once('error', reject);
    });
    console.log('[exp] Connected');
    await sleep(400);  // let first sensor sample arrive

    const results = {
        robot_ip: ROBOT_IP,
        cli: CLI,
        started_at_iso: new Date().toISOString(),
        started_at_ms: Date.now(),
        constants: { WHEEL_RADIUS, LX, LY, L_SUM, COUNTS_PER_WHEEL_REV },
        wheel_order: ['L1', 'R1', 'R2', 'L2'],  // external wire order
        runs: [],
    };

    try {
        for (const [specA, specB] of COMMAND_PAIRS) {
            const speeds = specA.axis === 'lin' ? CLI.lin_speeds : CLI.ang_speeds;
            for (const speed of speeds) {
                // Alternate A, B, A, B, ... so net displacement cancels per pair.
                for (let r = 0; r < CLI.runs; r++) {
                    for (const spec of [specA, specB]) {
                        const label = `${spec.key} (${spec.label}) @ ${speed}${spec.axis === 'lin' ? ' m/s' : ' rad/s'} run ${r + 1}/${CLI.runs}`;
                        console.log(`[exp] ${label}`);
                        const run = await runOne(ws, spec, speed);
                        run.run_index = r;
                        results.runs.push(run);

                        // One-line summary:
                        if (run.actual_rad_s.every(v => v != null)) {
                            const err = run.track_err_rad_s.map(e => e.toFixed(2));
                            const act = run.actual_rad_s.map(a => a.toFixed(2));
                            const exp = run.expected_rad_s.map(e => e.toFixed(2));
                            console.log(`        exp=[${exp.join(' ')}] act=[${act.join(' ')}] err=[${err.join(' ')}]`);
                        } else {
                            console.log(`        WARNING: missing start/end sensor sample`);
                        }

                        // Inter-run settle.
                        await sleep(500);
                    }
                }
            }
        }

        results.ended_at_iso = new Date().toISOString();
        results.ended_at_ms = Date.now();

        const jsonPath = join(outDir, 'results.json');
        writeFileSync(jsonPath, JSON.stringify(results, null, 2) + '\n');
        console.log(`\n[exp] Wrote ${jsonPath}`);

        // CSV for easy plotting / inspection.
        const csvLines = ['run_index,cmd_key,cmd_label,axis,speed,vx,vy,w,'
            + 'exp_L1,exp_R1,exp_R2,exp_L2,'
            + 'act_L1,act_R1,act_R2,act_L2,'
            + 'err_L1,err_R1,err_R2,err_L2,'
            + 'duration_s'];
        for (const run of results.runs) {
            const axis = COMMANDS.find(c => c.key === run.cmd_spec.key).axis;
            const row = [
                run.run_index,
                run.cmd_spec.key,
                run.cmd_spec.label,
                axis,
                run.speed,
                run.cmd.vx, run.cmd.vy, run.cmd.w,
                ...run.expected_rad_s,
                ...run.actual_rad_s.map(v => v == null ? '' : v),
                ...run.track_err_rad_s.map(v => v == null ? '' : v),
                run.duration_s,
            ];
            csvLines.push(row.join(','));
        }
        const csvPath = join(outDir, 'results.csv');
        writeFileSync(csvPath, csvLines.join('\n') + '\n');
        console.log(`[exp] Wrote ${csvPath}`);

        // Aggregate summary per (cmd, speed): mean & std of per-wheel tracking error across repeats.
        console.log('\n[exp] Summary (mean tracking error rad/s across repeats, per wheel [L1 R1 R2 L2]):');
        const groupKey = (r) => `${r.cmd_spec.key}@${r.speed}`;
        const groups = new Map();
        for (const r of results.runs) {
            const k = groupKey(r);
            if (!groups.has(k)) groups.set(k, []);
            groups.get(k).push(r);
        }
        for (const [k, gr] of groups) {
            const means = [0, 0, 0, 0];
            const stds = [0, 0, 0, 0];
            let n = 0;
            for (const r of gr) {
                if (!r.track_err_rad_s.every(v => v != null)) continue;
                for (let i = 0; i < 4; i++) means[i] += r.track_err_rad_s[i];
                n++;
            }
            if (n === 0) { console.log(`  ${k}: no samples`); continue; }
            for (let i = 0; i < 4; i++) means[i] /= n;
            for (const r of gr) {
                if (!r.track_err_rad_s.every(v => v != null)) continue;
                for (let i = 0; i < 4; i++) stds[i] += (r.track_err_rad_s[i] - means[i]) ** 2;
            }
            for (let i = 0; i < 4; i++) stds[i] = Math.sqrt(stds[i] / n);
            const fmt = (arr) => arr.map(v => (v >= 0 ? '+' : '') + v.toFixed(2)).join(' ');
            console.log(`  ${k.padEnd(8)}  mean=[${fmt(means)}]  std=[${stds.map(s => s.toFixed(2)).join(' ')}]  n=${n}`);
        }
    } finally {
        ws.close();
    }
}

main().catch(err => {
    console.error('[exp] FAILED:', err);
    process.exit(1);
});
