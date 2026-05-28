// Debug variant of the straight_2m smoketest.
//
// Enables firmware PID debug mode BEFORE arming so every sensor
// broadcast includes the per-wheel {tgt, act, err, p, i, d, ff, pwm}
// block. Server logs these to CSV as dbg_* columns. Intended for
// on-stand or on-floor diagnostic runs where we need to see PID
// internals (integrator convergence, PWM ceilings, feedforward
// accuracy) — not for thesis data.
//
// Usage:
//   node smoketest_straight_debug.mjs
//
// Disables debug mode automatically on exit.
import WebSocket from 'ws';

const URL = 'ws://localhost:3000/ws';
const TRAJECTORY = 'straight_2m';
const SPEED = 0.10;
const TIMEOUT_MS = 90_000;

const ws = new WebSocket(URL);
let runId = null;
let debugEnabled = false;

function send(obj) { ws.send(JSON.stringify(obj)); }

function disableDebugThenExit(code) {
    if (debugEnabled) {
        send({ type: 'set_debug', enabled: false });
        setTimeout(() => { ws.close(); process.exit(code); }, 300);
    } else {
        ws.close(); process.exit(code);
    }
}

ws.on('open', () => {
    console.log('[smoke-dbg] connected, enabling PID debug (rate_divider=1 — every tick)');
    send({ type: 'set_debug', enabled: true, rate_divider: 1 });
    // Give the ack time to come back before arming
    setTimeout(() => {
        console.log(`[smoke-dbg] arming ${TRAJECTORY} @ ${SPEED} m/s`);
        send({
            type: 'experiment_arm',
            trajectory: TRAJECTORY,
            speed: SPEED,
            rep: 1,
            operatorNotes: 'debug run — PID diag logged',
        });
    }, 500);
});

ws.on('message', (raw) => {
    let msg;
    try { msg = JSON.parse(raw.toString()); } catch { return; }

    if (msg.type === 'ack' && msg.cmd === 'set_debug') {
        debugEnabled = msg.enabled === true;
        console.log(`[smoke-dbg] debug ${debugEnabled ? 'ENABLED' : 'disabled'} (rate_divider=${msg.rate_divider})`);
        return;
    }

    if (msg.type === 'experiment_armed') {
        runId = msg.runId;
        console.log(`[smoke-dbg] armed: ${runId}`);
        if (msg.startHint?.text) console.log(`[smoke-dbg] placement hint: ${msg.startHint.text}`);
        console.log('[smoke-dbg] starting in 2 s...');
        setTimeout(() => send({ type: 'experiment_start' }), 2000);
        return;
    }

    if (msg.type === 'experiment_started') {
        console.log(`[smoke-dbg] started at ${msg.startedAt}`);
        return;
    }

    if (msg.type === 'experiment_tick') {
        if (Number.isFinite(msg.waypointsTotal) && msg.waypointsTotal > 0) {
            const label = msg.currentChunkLabel || '(none)';
            process.stdout.write(
                `\r[smoke-dbg] waypoint ${msg.waypointsCompleted + 1}/${msg.waypointsTotal} → ${label}  `,
            );
        }
        return;
    }

    if (msg.type === 'experiment_state') {
        if (msg.to !== 'running') console.log(`\n[smoke-dbg] state: ${msg.from} → ${msg.to}`);
        return;
    }

    if (msg.type === 'experiment_completed') {
        console.log(`\n[smoke-dbg] completed: ${msg.durationMs} ms total, `
            + `${msg.waypointsOk}/${msg.waypointsTotal} waypoints captured`);
        if (msg.lastWaypointGt) {
            const g = msg.lastWaypointGt;
            console.log(`[smoke-dbg] last waypoint GT: x=${g.x.toFixed(3)} y=${g.y.toFixed(3)} θ=${g.theta_deg.toFixed(1)}°`);
            send({
                type: 'experiment_ground_truth',
                xMeas: g.x,
                yMeas: g.y,
                thetaDegMeas: g.theta_deg,
                passFail: 'pass',
                notes: 'debug run auto-submit',
            });
        } else {
            console.log('[smoke-dbg] no camera waypoints — skipping auto-submit');
        }
        return;
    }

    if (msg.type === 'experiment_preflight_failed') {
        const m = msg.measured
            ? ` measured=(${msg.measured.x?.toFixed?.(2)}, ${msg.measured.y?.toFixed?.(2)}, ${msg.measured.thetaDeg?.toFixed?.(1)}°)`
            : '';
        console.log(`\n[smoke-dbg] preflight FAILED: ${msg.reason}${m}`);
        disableDebugThenExit(6);
    }

    if (msg.type === 'experiment_preflight_ok' && msg.measured) {
        console.log(`[smoke-dbg] preflight ok: measured=(${msg.measured.x.toFixed(2)}, ${msg.measured.y.toFixed(2)}, ${msg.measured.thetaDeg.toFixed(1)}°)`);
        return;
    }

    if (msg.type === 'experiment_aborted') {
        console.log(`\n[smoke-dbg] aborted: ${msg.reason}`);
        disableDebugThenExit(2);
    }

    if (msg.type === 'experiment_error') {
        console.log(`\n[smoke-dbg] ERROR: ${msg.error}`);
        disableDebugThenExit(3);
    }

    if (msg.type === 'state' && runId) {
        const expState = msg.experiment?.state;
        if (expState === 'idle' && runId && !global._done) {
            global._done = true;
            console.log('[smoke-dbg] done — back to idle');
            disableDebugThenExit(0);
        }
    }
});

ws.on('error', (e) => { console.error('[smoke-dbg] ws error:', e.message); process.exit(4); });

setTimeout(() => {
    console.error(`[smoke-dbg] timeout (${TIMEOUT_MS / 1000}s)`);
    disableDebugThenExit(5);
}, TIMEOUT_MS);
