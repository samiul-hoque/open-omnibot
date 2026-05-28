// Smoke-test runner for square_0m8_rotate.
//
// Arms the trajectory, starts it, and streams the runner's own events
// (state changes, ticks with waypointsCompleted/Total, completion).
// Auto-submits the ground-truth form at completion using the runner's
// last-waypoint auto-fill so the runner cleans up to idle without the
// operator having to touch the browser.
//
// Inspect afterwards: evaluation/snapshots/square_0m8_rotate/<runId>/*.png +
// the run's meta.json under server/logs/.
import WebSocket from 'ws';

const URL = 'ws://localhost:3000/ws';
const TRAJECTORY = 'square_0m8_rotate';
const SPEED = 0.10;

// Motion budget: 8 × 0.4 m translates at 0.10 m/s (32 s) + 4 × 90° yaws
// at 0.5 rad/s (≈12.6 s) + 13 × (settle 300 ms + snapshot lead 400 ms)
// pause overhead (≈9.1 s) ≈ 54 s. Allow headroom for GT submit + slop.
const TIMEOUT_MS = 180_000;

const ws = new WebSocket(URL);
let runId = null;

function send(obj) { ws.send(JSON.stringify(obj)); }

ws.on('open', () => {
    console.log(`[smoke] connected, arming ${TRAJECTORY} @ ${SPEED} m/s`);
    send({
        type: 'experiment_arm',
        trajectory: TRAJECTORY,
        speed: SPEED,
        rep: 1,
        operatorNotes: 'smoke test — square_0m8_rotate from (+0.4, -0.35) +Y',
    });
});

ws.on('message', (raw) => {
    let msg;
    try { msg = JSON.parse(raw.toString()); } catch { return; }

    if (msg.type === 'experiment_armed') {
        runId = msg.runId;
        console.log(`[smoke] armed: ${runId}`);
        if (msg.startHint?.text) console.log(`[smoke] placement hint: ${msg.startHint.text}`);
        console.log('[smoke] starting in 2 s...');
        setTimeout(() => send({ type: 'experiment_start' }), 2000);
        return;
    }

    if (msg.type === 'experiment_started') {
        console.log(`[smoke] started at ${msg.startedAt}`);
        return;
    }

    if (msg.type === 'experiment_tick') {
        if (Number.isFinite(msg.waypointsTotal) && msg.waypointsTotal > 0) {
            const label = msg.currentChunkLabel || '(none)';
            process.stdout.write(
                `\r[smoke] waypoint ${msg.waypointsCompleted + 1}/${msg.waypointsTotal} → ${label}  `,
            );
        }
        return;
    }

    if (msg.type === 'experiment_state') {
        if (msg.to !== 'running') console.log(`\n[smoke] state: ${msg.from} → ${msg.to}`);
        return;
    }

    if (msg.type === 'experiment_completed') {
        console.log(`\n[smoke] completed: ${msg.durationMs} ms total, `
            + `${msg.waypointsOk}/${msg.waypointsTotal} waypoints captured`);
        if (msg.lastWaypointGt) {
            const g = msg.lastWaypointGt;
            console.log(`[smoke] last waypoint GT: x=${g.x.toFixed(3)} y=${g.y.toFixed(3)} θ=${g.theta_deg.toFixed(1)}°`);
            send({
                type: 'experiment_ground_truth',
                xMeas: g.x,
                yMeas: g.y,
                thetaDegMeas: g.theta_deg,
                passFail: 'pass',
                notes: 'smoke test auto-submit',
            });
        } else {
            console.log('[smoke] no camera waypoints — skipping auto-submit (operator must close the run)');
        }
        return;
    }

    if (msg.type === 'experiment_preflight_failed') {
        const m = msg.measured
            ? ` measured=(${msg.measured.x?.toFixed?.(2)}, ${msg.measured.y?.toFixed?.(2)}, ${msg.measured.thetaDeg?.toFixed?.(1)}°)`
            : '';
        console.log(`\n[smoke] preflight FAILED: ${msg.reason}${m}`);
        ws.close(); process.exit(6);
    }

    if (msg.type === 'experiment_preflight_ok' && msg.measured) {
        console.log(`[smoke] preflight ok: measured=(${msg.measured.x.toFixed(2)}, ${msg.measured.y.toFixed(2)}, ${msg.measured.thetaDeg.toFixed(1)}°)`);
        return;
    }

    if (msg.type === 'experiment_aborted') {
        console.log(`\n[smoke] aborted: ${msg.reason}`);
        ws.close(); process.exit(2);
    }

    if (msg.type === 'experiment_error') {
        console.log(`\n[smoke] ERROR: ${msg.error}`);
        ws.close(); process.exit(3);
    }

    if (msg.type === 'state' && runId) {
        const expState = msg.experiment?.state;
        if (expState === 'idle' && runId && !global._done) {
            global._done = true;
            console.log('[smoke] done — back to idle');
            setTimeout(() => { ws.close(); process.exit(0); }, 200);
        }
    }
});

ws.on('error', (e) => { console.error('[smoke] ws error:', e.message); process.exit(4); });

setTimeout(() => { console.error(`[smoke] timeout (${TIMEOUT_MS / 1000}s)`); process.exit(5); }, TIMEOUT_MS);
