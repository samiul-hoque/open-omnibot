#!/usr/bin/env node
// Ground-truth motion characterization using IMU + encoder streams.
//
// Why this exists: on a stand the wheel tracking looks clean (see the
// 2026-04-15 motion_sweep results) but the user reports A/D strafing on
// the ground produces noticeable forward/backward drift. That could be:
//
//   (a) wheel-speed asymmetry → non-zero vx in forward kinematics,
//   (b) mecanum roller/ground slip → non-zero vx in the IMU accel
//       even when forward kinematics says vx=0,
//   (c) chassis yaw during the strafe (uneven front/rear friction) →
//       a non-zero gz / yaw change during the press.
//
// This script runs one press of each of W/S/A/D/Q/E with the robot on
// the floor and compares the wheel-derived body velocity (FK) against
// the IMU's accelerometer + gyro signals. Whichever channel shows the
// drift first tells us which bucket the bug is in.
//
// Setup: robot on the floor with ~1m clear around it; server NOT
// running (we own the cmd stream).
//
// Usage:
//   ROBOT_IP=robot.local node evaluation/scripts/ground_imu_experiment.mjs
//   node evaluation/scripts/ground_imu_experiment.mjs --duration-ms 1500 --lin 0.10 --ang 0.30

import WebSocket from 'ws';
import { writeFileSync, mkdirSync } from 'fs';
import { join } from 'path';

const ROBOT_IP = process.env.ROBOT_IP || 'robot.local';
const ROBOT_URL = `ws://${ROBOT_IP}:80/ws`;

const WHEEL_RADIUS = 0.04;
const LX = 0.1175, LY = 0.0953;
const L_SUM = LX + LY;
const COUNTS_PER_WHEEL_REV = 1092;

function parseArgs() {
    const args = {
        duration_ms: 1000, lin: 0.10, ang: 0.30, inter_run_ms: 2000, cmd_hz: 50,
        heading_hold: 'both',  // 'off' | 'on' | 'both'
        heading_hold_gain: 1.0,
    };
    for (let i = 2; i < process.argv.length; i++) {
        const a = process.argv[i];
        if (a === '--duration-ms') args.duration_ms = parseInt(process.argv[++i], 10);
        else if (a === '--lin') args.lin = parseFloat(process.argv[++i]);
        else if (a === '--ang') args.ang = parseFloat(process.argv[++i]);
        else if (a === '--inter-run-ms') args.inter_run_ms = parseInt(process.argv[++i], 10);
        else if (a === '--cmd-hz') args.cmd_hz = parseInt(process.argv[++i], 10);
        else if (a === '--heading-hold') args.heading_hold = process.argv[++i];
        else if (a === '--heading-hold-gain') args.heading_hold_gain = parseFloat(process.argv[++i]);
        else if (a === '--help' || a === '-h') {
            console.log('ground_imu_experiment.mjs [--duration-ms MS] [--lin M/S] [--ang RAD/S]');
            console.log('                          [--inter-run-ms MS] [--heading-hold off|on|both]');
            console.log('                          [--heading-hold-gain 1.0]');
            process.exit(0);
        }
    }
    return args;
}

const CLI = parseArgs();
const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const send = (ws, obj) => ws.send(JSON.stringify(obj));

const runStamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
const outDir = join('../experiments', `ground_imu_${runStamp}`);
mkdirSync(outDir, { recursive: true });

// Note on indexing: msg.enc / msg.vel on the wire are [L1, R1, R2, L2]
// (external convention; see websocket_server.cpp:609/622). We work in that
// order throughout and compute FK in the same order.
const COMMANDS = [
    { key: 'W', label: 'forward',     axis: 'lin', make: (s) => ({ vx:  s, vy:  0, w: 0 }) },
    { key: 'S', label: 'backward',    axis: 'lin', make: (s) => ({ vx: -s, vy:  0, w: 0 }) },
    { key: 'A', label: 'strafe_left', axis: 'lin', make: (s) => ({ vx:  0, vy:  s, w: 0 }) },
    { key: 'D', label: 'strafe_right',axis: 'lin', make: (s) => ({ vx:  0, vy: -s, w: 0 }) },
    { key: 'Q', label: 'rotate_ccw',  axis: 'ang', make: (s) => ({ vx:  0, vy:  0, w:  s }) },
    { key: 'E', label: 'rotate_cw',   axis: 'ang', make: (s) => ({ vx:  0, vy:  0, w: -s }) },
];

// Forward kinematics, external [L1, R1, R2, L2] order. Matches firmware
// mecanum.cpp after the 2026-04-15 vy sign fix — positive vy = left.
//   vx = (r/4)     * ( ω_L1 + ω_R1 + ω_R2 + ω_L2)
//   vy = (r/4)     * ( ω_L1 - ω_R1 + ω_R2 - ω_L2)
//   w  = (r/(4·L)) * (-ω_L1 + ω_R1 + ω_R2 - ω_L2)
function forwardKinematics([o_L1, o_R1, o_R2, o_L2]) {
    const r = WHEEL_RADIUS, L = L_SUM;
    return {
        vx: (r / 4) * (o_L1 + o_R1 + o_R2 + o_L2),
        vy: (r / 4) * (o_L1 - o_R1 + o_R2 - o_L2),
        w:  (r / (4 * L)) * (-o_L1 + o_R1 + o_R2 - o_L2),
    };
}

function mean(xs) {
    if (!xs.length) return null;
    return xs.reduce((a, b) => a + b, 0) / xs.length;
}

function std(xs) {
    if (xs.length < 2) return 0;
    const m = mean(xs);
    return Math.sqrt(xs.map(x => (x - m) ** 2).reduce((a, b) => a + b, 0) / xs.length);
}

// Wrap yaw delta into (-180, 180] to handle 360° wraparound.
function unwrapYawDelta(d) {
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
}

async function runOne(ws, cmdSpec, speed) {
    const cmd = cmdSpec.make(speed);
    const samples = [];
    const onMsg = (raw) => {
        let msg;
        try { msg = JSON.parse(raw); } catch { return; }
        if (msg.type === 'sensors') {
            samples.push({
                t: Date.now(),
                enc: msg.enc,
                vel: msg.vel,
                yaw: msg.imu?.yaw ?? 0,
                gz: msg.imu?.gz ?? 0,
                ax: msg.imu?.ax ?? 0,
                ay: msg.imu?.ay ?? 0,
            });
        }
    };
    ws.on('message', onMsg);

    // Reset encoders and settle. Do NOT zero IMU here — BNO055 zero is a
    // ~ few-hundred-ms operation and it'd interact with the command start.
    send(ws, { type: 'reset_encoders' });
    await sleep(400);

    // Quiescent-noise window: capture samples with the robot stopped, so
    // we can report an IMU noise floor alongside each run. This makes
    // post-hoc interpretation of the during-motion std meaningful.
    const quiescentStart = Date.now();
    const quiescentWindowMs = 1500;
    await sleep(quiescentWindowMs);
    const quiescentEnd = Date.now();
    const quiescentSamples = samples.filter(s => s.t >= quiescentStart && s.t <= quiescentEnd);
    const qGz = quiescentSamples.map(s => s.gz);
    const qAx = quiescentSamples.map(s => s.ax);
    const qAy = quiescentSamples.map(s => s.ay);
    const quiescent = {
        gz_mean: mean(qGz), gz_std: std(qGz),
        ax_mean: mean(qAx), ax_std: std(qAx),
        ay_mean: mean(qAy), ay_std: std(qAy),
        n_samples: quiescentSamples.length,
    };

    // Pre-press snapshot (last sample we've seen).
    const startSample = samples[samples.length - 1];

    const pressStart = Date.now();
    const pressEnd = pressStart + CLI.duration_ms;
    const periodMs = Math.max(1, Math.round(1000 / CLI.cmd_hz));
    while (Date.now() < pressEnd) {
        send(ws, { type: 'cmd', ...cmd });
        await sleep(periodMs);
    }
    const pressStop = Date.now();
    send(ws, { type: 'cmd', vx: 0, vy: 0, w: 0 });
    await sleep(150);
    send(ws, { type: 'stop' });
    await sleep(300);

    ws.off('message', onMsg);

    // Window samples to press window.
    const during = samples.filter(s => s.t >= pressStart && s.t <= pressStop);
    const endSample = during[during.length - 1] ?? samples[samples.length - 1];

    const durS = (pressStop - pressStart) / 1000;

    // Wheel speeds (rad/s) averaged from encoder start/end.
    const wheelSpeeds = [null, null, null, null];
    if (startSample && endSample) {
        for (let i = 0; i < 4; i++) {
            wheelSpeeds[i] = ((endSample.enc[i] - startSample.enc[i]) / COUNTS_PER_WHEEL_REV) * 2 * Math.PI / durS;
        }
    }

    const fk = wheelSpeeds.every(w => w != null) ? forwardKinematics(wheelSpeeds) : null;

    // IMU summaries over the press window.
    const axs = during.map(s => s.ax);
    const ays = during.map(s => s.ay);
    const gzs = during.map(s => s.gz);
    const imu = {
        ax_mean: mean(axs), ax_std: std(axs),
        ay_mean: mean(ays), ay_std: std(ays),
        gz_mean: mean(gzs), gz_std: std(gzs),
        yaw_start: startSample?.yaw ?? null,
        yaw_end: endSample?.yaw ?? null,
        yaw_delta: (startSample && endSample) ? unwrapYawDelta(endSample.yaw - startSample.yaw) : null,
        n_samples: during.length,
    };

    return {
        cmd_spec: { key: cmdSpec.key, label: cmdSpec.label },
        speed,
        cmd,
        duration_s: durS,
        wheel_speeds_rad_s: wheelSpeeds,
        fk_body_vel: fk,
        imu,
        quiescent,
    };
}

async function main() {
    console.log(`[exp] Connecting to ${ROBOT_URL}`);
    console.log(`[exp] Robot should be on the floor, ~1m clear all sides. Each press is ${CLI.duration_ms}ms.`);
    console.log(`[exp] lin=${CLI.lin} m/s  ang=${CLI.ang} rad/s  inter_run=${CLI.inter_run_ms}ms`);
    const ws = new WebSocket(ROBOT_URL);
    await new Promise((resolve, reject) => {
        ws.once('open', resolve);
        ws.once('error', reject);
    });
    console.log('[exp] Connected. Starting in 2s — keep robot on the grid.');
    await sleep(2000);

    const results = {
        robot_ip: ROBOT_IP,
        cli: CLI,
        started_at_iso: new Date().toISOString(),
        wheel_order: ['L1', 'R1', 'R2', 'L2'],
        runs: [],
    };

    // Build phase list based on --heading-hold flag.
    const phases = CLI.heading_hold === 'both'
        ? [{ hh: false }, { hh: true }]
        : [{ hh: CLI.heading_hold === 'on' }];

    try {
        for (const phase of phases) {
            // Toggle heading-hold on the robot for this phase.
            send(ws, { type: 'set_heading_hold', enabled: phase.hh, gain: CLI.heading_hold_gain });
            console.log(`\n==== phase: heading-hold ${phase.hh ? 'ON' : 'OFF'} ====`);
            await sleep(400);  // let the ack settle

            for (const spec of COMMANDS) {
                const speed = spec.axis === 'lin' ? CLI.lin : CLI.ang;
                console.log(`\n[exp] ${spec.key} (${spec.label}) @ ${speed}${spec.axis === 'lin' ? ' m/s' : ' rad/s'}  (hh=${phase.hh ? 'ON' : 'off'})`);
                const run = await runOne(ws, spec, speed);
                run.heading_hold = phase.hh;
                results.runs.push(run);

                // Human summary.
                if (run.fk_body_vel && run.imu.yaw_delta != null) {
                    const fk = run.fk_body_vel;
                    const imu = run.imu;
                    const q = run.quiescent;
                    console.log(`   FK body vel:  vx=${fk.vx.toFixed(3)} m/s  vy=${fk.vy.toFixed(3)} m/s  w=${fk.w.toFixed(3)} rad/s`);
                    console.log(`   IMU accel:    ax=${imu.ax_mean.toFixed(3)}±${imu.ax_std.toFixed(2)}  ay=${imu.ay_mean.toFixed(3)}±${imu.ay_std.toFixed(2)}  (m/s²)`);
                    console.log(`   IMU yaw:      delta=${imu.yaw_delta.toFixed(2)}°  gz_mean=${imu.gz_mean.toFixed(3)}±${imu.gz_std.toFixed(3)} rad/s  n=${imu.n_samples}`);
                    console.log(`   Quiescent:    gz=${q.gz_mean.toFixed(3)}±${q.gz_std.toFixed(3)} rad/s  ax=${q.ax_mean.toFixed(3)}±${q.ax_std.toFixed(2)}  ay=${q.ay_mean.toFixed(3)}±${q.ay_std.toFixed(2)}  n=${q.n_samples}`);
                } else {
                    console.log('   (missing data — check WebSocket connection)');
                }

                await sleep(CLI.inter_run_ms);
            }
        }

        // Leave heading-hold off at exit so we don't surprise a later session.
        send(ws, { type: 'set_heading_hold', enabled: false });
        await sleep(200);

        results.ended_at_iso = new Date().toISOString();
        const jsonPath = join(outDir, 'results.json');
        writeFileSync(jsonPath, JSON.stringify(results, null, 2) + '\n');
        console.log(`\n[exp] Wrote ${jsonPath}`);

        // Compact table for quick reading.
        console.log('\n[exp] Summary table (all values mean over press window; q_gz_std from pre-press quiescent window):');
        console.log('      hh  | cmd  |  FK vx    FK vy    FK w    |  IMU gz    gz_std  |  yaw Δ°  | q_gz_std');
        console.log('      --- + ---- + -------- -------- -------- + -------- -------- + -------- + --------');
        for (const run of results.runs) {
            if (!run.fk_body_vel || run.imu.yaw_delta == null) continue;
            const fk = run.fk_body_vel, imu = run.imu, q = run.quiescent;
            const fmt = (v) => (v >= 0 ? '+' : '') + v.toFixed(3);
            const fmtDeg = (v) => (v >= 0 ? '+' : '') + v.toFixed(1);
            const hh = run.heading_hold ? ' ON' : 'off';
            console.log(`      ${hh} | ${run.cmd_spec.key.padEnd(4)} | ${fmt(fk.vx).padStart(8)} ${fmt(fk.vy).padStart(8)} ${fmt(fk.w).padStart(8)} | ${fmt(imu.gz_mean).padStart(8)} ${imu.gz_std.toFixed(3).padStart(8)} | ${fmtDeg(imu.yaw_delta).padStart(6)}  | ${q.gz_std.toFixed(3).padStart(8)}`);
        }
    } finally {
        ws.close();
    }
}

main().catch(err => {
    console.error('[exp] FAILED:', err);
    process.exit(1);
});
