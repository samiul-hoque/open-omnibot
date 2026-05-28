// Free-wheel bench check. Robot on blocks. Drives each direction for
// ~2s at 10Hz, samples wheel velocities during the last half of each
// burst, prints a summary. Wire index order: [L1, R1, R2, L2].
//
// Expected magnitudes at v=0.2 m/s translate: ~5.0 rad/s per wheel
// (wheel_radius = 0.04 m).
// At omega=0.5 rad/s in-place yaw: ~2.66 rad/s per wheel
// ((Lx+Ly)/r * omega = (0.1175+0.0953)/0.04 * 0.5).

import WebSocket from 'ws';

const URL = 'ws://robot.local/ws';
const TRANSLATE_SPEED = 0.2;   // m/s
const YAW_SPEED = 0.5;         // rad/s
const BURST_MS = 2000;
const SAMPLE_START_MS = 1000;  // sample the steady-state second half
const KEEPALIVE_MS = 100;      // firmware timeout is 500 ms

const tests = [
    { name: 'forward  (W)',  cmd: { vx:  TRANSLATE_SPEED, vy: 0, w: 0 } },
    { name: 'backward (S)',  cmd: { vx: -TRANSLATE_SPEED, vy: 0, w: 0 } },
    { name: 'strafe L (A)',  cmd: { vx: 0, vy:  TRANSLATE_SPEED, w: 0 } },
    { name: 'strafe R (D)',  cmd: { vx: 0, vy: -TRANSLATE_SPEED, w: 0 } },
    { name: 'rotate + (Q)',  cmd: { vx: 0, vy: 0, w:  YAW_SPEED } },
    { name: 'rotate - (E)',  cmd: { vx: 0, vy: 0, w: -YAW_SPEED } },
];

const ws = new WebSocket(URL);
let samples = [];
let sampleActive = false;

function send(obj) { ws.send(JSON.stringify(obj)); }

ws.on('message', (raw) => {
    let msg;
    try { msg = JSON.parse(raw.toString()); } catch { return; }
    if (msg.type === 'sensors' && sampleActive) {
        samples.push(msg.vel);
    }
});

function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

function mean(arr) { return arr.reduce((a, b) => a + b, 0) / arr.length; }

async function runBurst(name, cmd) {
    samples = [];
    const start = Date.now();
    const ki = setInterval(() => send({ type: 'cmd', ...cmd }), KEEPALIVE_MS);
    send({ type: 'cmd', ...cmd });

    // Wait for the sampling window to open.
    await sleep(SAMPLE_START_MS);
    sampleActive = true;
    await sleep(BURST_MS - SAMPLE_START_MS);
    sampleActive = false;
    clearInterval(ki);
    send({ type: 'stop' });

    const n = samples.length;
    if (n === 0) {
        console.log(`  ${name.padEnd(16)} no samples`);
        return;
    }
    const perWheel = [0, 1, 2, 3].map(i => mean(samples.map(s => s[i])));
    const spread = Math.max(...perWheel.map(Math.abs)) - Math.min(...perWheel.map(Math.abs));
    const elapsed = Date.now() - start;
    console.log(`  ${name.padEnd(16)} L1=${perWheel[0].toFixed(2).padStart(6)}  R1=${perWheel[1].toFixed(2).padStart(6)}  R2=${perWheel[2].toFixed(2).padStart(6)}  L2=${perWheel[3].toFixed(2).padStart(6)}  rad/s  [|spread|=${spread.toFixed(2)}, n=${n}, ${elapsed}ms]`);

    // Wheels need a moment to spin down before the next test.
    await sleep(1000);
}

ws.on('open', async () => {
    console.log('connected to', URL);
    console.log('wheel index order: [L1, R1, R2, L2]   (rad/s, means over 1 s steady-state window)\n');
    for (const t of tests) {
        await runBurst(t.name, t.cmd);
    }
    send({ type: 'stop' });
    await sleep(200);
    ws.close();
    process.exit(0);
});

ws.on('error', (e) => { console.error('ws error:', e.message); process.exit(1); });
setTimeout(() => { console.error('bench timeout'); process.exit(2); }, 30000);
