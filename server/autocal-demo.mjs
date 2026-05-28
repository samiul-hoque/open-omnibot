// Autocal demonstration snapshot capture.
// Mirrors the browser calibration.js auto-cal sequence (6 directions,
// PWM 100, 700 ms drive, 1500 ms settle) but is READ-ONLY — does not
// call set_openloop_cal, so NVS values are preserved.
//
// For each direction: snap before → drive → settle → snap after →
// compute body-frame delta and measured speed. Writes all PNGs and
// a summary.json under evaluation/snapshots/autocal/<runId>/.

import WebSocket from 'ws';
import fs from 'fs';
import path from 'path';

const SERVER_WS = 'ws://localhost:3000/ws';
const ARUCO_URL = 'http://localhost:5055';
const REPO_ROOT = path.resolve(path.dirname(new URL(import.meta.url).pathname), '..');

// Mirror of calibration.js constants
const DIRS = [
    { key: 'fwd',      label: 'Forward (+X)',  signs: [+1, +1, +1, +1], kind: 'linear' },
    { key: 'back',     label: 'Backward (-X)', signs: [-1, -1, -1, -1], kind: 'linear' },
    { key: 'strafe_l', label: 'Strafe L (+Y)', signs: [+1, -1, -1, +1], kind: 'linear' },
    { key: 'strafe_r', label: 'Strafe R (-Y)', signs: [-1, +1, +1, -1], kind: 'linear' },
    { key: 'yaw_ccw',  label: 'Yaw CCW (+ω)',  signs: [-1, +1, -1, +1], kind: 'yaw' },
    { key: 'yaw_cw',   label: 'Yaw CW (-ω)',   signs: [+1, -1, +1, -1], kind: 'yaw' },
];
const INT_TO_EXT = [0, 1, 3, 2]; // internal L1,R1,L2,R2 → wire L1,R1,R2,L2
const PWM = 100;
const DRIVE_MS = 700;
// Must cover mechanical settle (~1 s) PLUS RTSP CONTENT lag (~2 s).
// read_fresh() drains the buffer but the scene itself is ~1.5–2.5 s behind
// reality — a 1500 ms settle produces snap_after frames whose content is
// still mid-drive. See trajectoryRunner.js:68-89 for the full measurement.
const SETTLE_MS = 3500;
const TICK_MS = 20;
const PAUSE_BETWEEN_MS = 800; // extra breather between directions

const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
const RUN_ID = `autocal_demo_${ts}`;
const TRAJECTORY = 'autocal';
const SNAP_DIR = path.join(REPO_ROOT, 'evaluation', 'snapshots', TRAJECTORY, RUN_ID);

async function snap(label) {
    const url = `${ARUCO_URL}/snapshot?trajectory=${TRAJECTORY}&run_id=${RUN_ID}&label=${label}`;
    const res = await fetch(url);
    if (!res.ok) throw new Error(`snapshot ${label} failed: HTTP ${res.status}`);
    const data = await res.json();
    if (!data.detected) throw new Error(`snapshot ${label}: marker not detected`);
    return data;
}

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

function normalizeAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
}

function drive(ws, dir, durationMs) {
    const start = Date.now();
    return new Promise((resolve) => {
        const tick = () => {
            for (let i = 0; i < 4; i++) {
                ws.send(JSON.stringify({
                    type: 'motor_test',
                    motor: INT_TO_EXT[i],
                    pwm: dir.signs[i] * PWM,
                }));
            }
            if (Date.now() - start >= durationMs) {
                clearInterval(interval);
                ws.send(JSON.stringify({ type: 'stop' }));
                resolve();
            }
        };
        tick();
        const interval = setInterval(tick, TICK_MS);
    });
}

function measureLinear(key, before, after) {
    const dxW = after.x - before.x;
    const dyW = after.y - before.y;
    const c = Math.cos(before.theta);
    const s = Math.sin(before.theta);
    const dxBody =  c * dxW + s * dyW;
    const dyBody = -s * dxW + c * dyW;
    switch (key) {
    case 'fwd':      return { signedDisp: dxBody,  speed: dxBody  / (DRIVE_MS / 1000), dxBody, dyBody };
    case 'back':     return { signedDisp: -dxBody, speed: -dxBody / (DRIVE_MS / 1000), dxBody, dyBody };
    case 'strafe_l': return { signedDisp: dyBody,  speed: dyBody  / (DRIVE_MS / 1000), dxBody, dyBody };
    case 'strafe_r': return { signedDisp: -dyBody, speed: -dyBody / (DRIVE_MS / 1000), dxBody, dyBody };
    }
}

function measureYaw(key, before, after) {
    const dTheta = normalizeAngle(after.theta - before.theta);
    const signed = key === 'yaw_ccw' ? dTheta : -dTheta;
    return { signedDisp: signed, speed: signed / (DRIVE_MS / 1000), dThetaRad: dTheta };
}

async function main() {
    fs.mkdirSync(SNAP_DIR, { recursive: true });
    console.log(`[demo] run_id: ${RUN_ID}`);
    console.log(`[demo] snapshots → ${SNAP_DIR}`);

    // Pre-flight health on aruco
    const health = await fetch(`${ARUCO_URL}/health`).then((r) => r.json());
    if (!health.ok) throw new Error('aruco service not healthy');
    console.log(`[demo] aruco ok, grabber ${health.grabber.latest_frame_age_ms} ms fresh`);

    // Connect server WS
    const ws = new WebSocket(SERVER_WS);
    await new Promise((resolve, reject) => {
        ws.once('open', resolve);
        ws.once('error', reject);
    });
    console.log(`[demo] connected to ${SERVER_WS}`);

    const summary = {
        runId: RUN_ID,
        startedAt: new Date().toISOString(),
        params: { pwm: PWM, driveMs: DRIVE_MS, settleMs: SETTLE_MS },
        directions: [],
    };

    for (const dir of DIRS) {
        console.log(`\n[demo] ── ${dir.key} (${dir.label}) ──`);

        const before = await snap(`${dir.key}_before`);
        console.log(`[demo] before: x=${before.x.toFixed(3)} y=${before.y.toFixed(3)} θ=${before.theta_deg.toFixed(1)}° side=${before.side_px.toFixed(1)}px`);

        await drive(ws, dir, DRIVE_MS);
        await sleep(SETTLE_MS);

        const after = await snap(`${dir.key}_after`);
        console.log(`[demo] after:  x=${after.x.toFixed(3)} y=${after.y.toFixed(3)} θ=${after.theta_deg.toFixed(1)}°`);

        const m = dir.kind === 'linear'
            ? measureLinear(dir.key, before, after)
            : measureYaw(dir.key, before, after);

        if (dir.kind === 'linear') {
            console.log(`[demo] Δbody=(${m.dxBody.toFixed(3)}, ${m.dyBody.toFixed(3)}) m  signed=${m.signedDisp.toFixed(3)} m  speed=${m.speed.toFixed(3)} m/s`);
        } else {
            console.log(`[demo] Δθ=${(m.dThetaRad * 180 / Math.PI).toFixed(2)}°  signed=${(m.signedDisp * 180 / Math.PI).toFixed(2)}°  speed=${m.speed.toFixed(3)} rad/s`);
        }

        summary.directions.push({
            key: dir.key,
            label: dir.label,
            kind: dir.kind,
            before: { x: before.x, y: before.y, theta_deg: before.theta_deg, image_path: before.image_path, side_px: before.side_px },
            after:  { x: after.x,  y: after.y,  theta_deg: after.theta_deg,  image_path: after.image_path,  side_px: after.side_px },
            measurement: m,
        });

        await sleep(PAUSE_BETWEEN_MS);
    }

    summary.endedAt = new Date().toISOString();
    fs.writeFileSync(path.join(SNAP_DIR, 'summary.json'), JSON.stringify(summary, null, 2));
    ws.close();

    console.log(`\n[demo] complete.`);
    console.log(`[demo] summary: ${path.join(SNAP_DIR, 'summary.json')}`);
    console.log(`[demo] snapshots: ${SNAP_DIR}/*.png  (12 files)`);

    // Compact results table
    console.log('\n| direction | measured speed | handover baseline |');
    console.log('|---|---|---|');
    const baseline = { fwd: 0.497, back: 0.504, strafe_l: 0.373, strafe_r: 0.373, yaw_ccw: 2.02, yaw_cw: 1.85 };
    for (const d of summary.directions) {
        const unit = d.kind === 'linear' ? 'm/s' : 'rad/s';
        console.log(`| ${d.key} | ${d.measurement.speed.toFixed(3)} ${unit} | ${baseline[d.key]} ${unit} |`);
    }
}

main().catch((err) => {
    console.error(`[demo] FAILED: ${err.message}`);
    process.exit(1);
});
