// ============================================
// Dashboard View — Joystick, Map, Sensors, Keyboard
// ============================================

import { App } from './app.js';
import { Joystick } from './joystick.js';
import { RobotMap } from './map.js';

const T = App.setText.bind(App);
let robotMap = null;
let _joystick = null;

// --- State ---
// Slider bounds in index.html: speed 0.05–0.30 m/s, turn 0.05–1.00 rad/s.
// Bounds picked from motion_sweep_experiment.mjs tracking data (2026-04-15)
// to stay below IK saturation under combined motion at MAX_WHEEL_SPEED=12.
let maxSpeed = 0.15;
let maxTurnSpeed = 0.30;
const activeKeys = new Set();
const rotateButtonState = { left: false, right: false };
let keyCommandInterval = null;
let currentTier = 1;

// Combined angular velocity from keyboard (q/e) + on-screen rotate buttons.
function currentW() {
    let w = 0;
    if (activeKeys.has('q') || rotateButtonState.left) w += maxTurnSpeed;
    if (activeKeys.has('e') || rotateButtonState.right) w -= maxTurnSpeed;
    return w;
}

// Demo mode
let demoInterval = null;
let demoState = null;

// ============================================
// Joystick
// ============================================

function initJoystick() {
    const canvas = document.getElementById('d-joystick-canvas');
    if (!canvas) return;
    _joystick = new Joystick(canvas, {
        onMove(nx, ny) {
            const vx = -ny * maxSpeed;  // up = forward
            const vy = -nx * maxSpeed;  // left = left
            App.send({ type: 'cmd', vx, vy, w: currentW() });
        },
        onRelease() {
            App.send({ type: 'cmd', vx: 0, vy: 0, w: currentW() });
        },
    });
}

// Start/stop the 100 Hz keep-alive that drives the robot while
// WASD/QE or the on-screen rotate buttons are held.
function refreshKeyCommandLoop() {
    const anyActive = activeKeys.size > 0
        || rotateButtonState.left || rotateButtonState.right;
    if (anyActive && !keyCommandInterval) {
        sendKeyboardCommand();
        keyCommandInterval = setInterval(sendKeyboardCommand, 100);
    } else if (!anyActive && keyCommandInterval) {
        clearInterval(keyCommandInterval);
        keyCommandInterval = null;
        App.send({ type: 'cmd', vx: 0, vy: 0, w: 0 });
    }
}

function setRotateButton(side, pressed) {
    rotateButtonState[side] = pressed;
    refreshKeyCommandLoop();
}

// ============================================
// Keyboard Control (global — always active)
// ============================================

function sendKeyboardCommand() {
    let vx = 0, vy = 0;
    if (activeKeys.has('w')) vx += maxSpeed;
    if (activeKeys.has('s')) vx -= maxSpeed;
    if (activeKeys.has('a')) vy += maxSpeed;
    if (activeKeys.has('d')) vy -= maxSpeed;
    App.send({ type: 'cmd', vx, vy, w: currentW() });
}

document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') return;
    const k = e.key.toLowerCase();
    if ('wasdqe'.includes(k) && !activeKeys.has(k)) {
        activeKeys.add(k);
        refreshKeyCommandLoop();
    }
});

document.addEventListener('keyup', (e) => {
    const k = e.key.toLowerCase();
    activeKeys.delete(k);
    refreshKeyCommandLoop();
});

window.addEventListener('blur', () => {
    activeKeys.clear();
    rotateButtonState.left = false;
    rotateButtonState.right = false;
    document.getElementById('d-rotate-left')?.classList.remove('active');
    document.getElementById('d-rotate-right')?.classList.remove('active');
    if (keyCommandInterval) {
        clearInterval(keyCommandInterval);
        keyCommandInterval = null;
    }
    App.send({ type: 'cmd', vx: 0, vy: 0, w: 0 });
});

// Register keyboard cleanup with E-STOP
App.onEmergencyStop(() => {
    activeKeys.clear();
    rotateButtonState.left = false;
    rotateButtonState.right = false;
    document.getElementById('d-rotate-left')?.classList.remove('active');
    document.getElementById('d-rotate-right')?.classList.remove('active');
    if (keyCommandInterval) { clearInterval(keyCommandInterval); keyCommandInterval = null; }
});

// Clean up keyboard interval and joystick when switching away from dashboard
App.on('viewChanged', (view) => {
    if (view !== 'dashboard') {
        activeKeys.clear();
        rotateButtonState.left = false;
        rotateButtonState.right = false;
        if (keyCommandInterval) {
            clearInterval(keyCommandInterval);
            keyCommandInterval = null;
            App.send({ type: 'cmd', vx: 0, vy: 0, w: 0 });
        }
        if (_joystick) { _joystick.destroy(); _joystick = null; }
    } else {
        // Recreate joystick when returning to dashboard
        if (!_joystick) initJoystick();
    }
});

// ============================================
// Orientation Compass
// ============================================

let lastYaw = 0;
function drawOrientation(yaw) {
    const canvas = document.getElementById('d-orientation-canvas');
    if (!canvas) return;
    lastYaw = yaw || 0;
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const cx = W / 2, cy = H / 2, r = Math.min(cx, cy) - 8;

    const t = (window.Theme && window.Theme.tokens) ? window.Theme.tokens() : {};
    const accentRing = t.border || 'rgba(59,130,246,0.3)';
    const tickColor  = t.text || '#dae2fd';
    const headingCol = t.error || '#ef4444';
    const centerCol  = t.text || '#dae2fd';

    ctx.clearRect(0, 0, W, H);

    // Circle
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.strokeStyle = accentRing;
    ctx.lineWidth = 1;
    ctx.stroke();

    // Tick marks
    ctx.strokeStyle = tickColor;
    for (let deg = 0; deg < 360; deg += 10) {
        const rad = deg * Math.PI / 180;
        const inner = deg % 90 === 0 ? r - 10 : r - 5;
        ctx.globalAlpha = deg % 90 === 0 ? 0.6 : 0.25;
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(rad) * inner, cy + Math.sin(rad) * inner);
        ctx.lineTo(cx + Math.cos(rad) * r, cy + Math.sin(rad) * r);
        ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // Robot heading arrow.
    //
    // Reference frame:
    //   - Idle / teleop (no measuredStartPose captured): IMU-local.
    //     Yaw=0 at arm time → needle points to screen +X (robot forward).
    //   - During a run (preflight_ok fired → measuredStartPose set):
    //     world-frame. The needle tracks the robot's heading in the map's
    //     world frame, so e.g. `square_rotate` with startHint.headingDeg=90
    //     shows the needle pointing to +Y right after arm.
    //
    // Geometry: the arrow is drawn tip-up (local +Y negative in canvas).
    // ctx.rotate is CW-positive because canvas Y is flipped, so to point
    // tip in world-direction θ (CCW-positive, 0 = +X right) we rotate by
    // (π/2 − θ). Equivalently with `heading = −yaw_rad`: `heading + π/2`.
    const mp = expState.measuredStartPose;
    const runActive = expState.current === 'running'
        || expState.current === 'awaiting_ground_truth';
    const startThetaDeg = (mp && runActive) ? (mp.theta * 180 / Math.PI) : 0;
    const worldYawDeg = startThetaDeg + (yaw || 0);
    const heading = -worldYawDeg * Math.PI / 180;
    const arrowLen = r * 0.65;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(heading + Math.PI / 2);
    ctx.beginPath();
    ctx.moveTo(0, -arrowLen);
    ctx.lineTo(-6, 6);
    ctx.lineTo(6, 6);
    ctx.closePath();
    ctx.fillStyle = headingCol;
    ctx.fill();
    ctx.restore();

    // Center dot
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fillStyle = centerCol;
    ctx.fill();
}

// Re-render the orientation gauge when the theme changes.
if (window.Theme && typeof window.Theme.onChange === 'function') {
    window.Theme.onChange(() => drawOrientation(lastYaw));
}

// ============================================
// Sensor Updates
// ============================================

function updateDashboardState(state) {
    if (!state.sensors) return;
    const s = state.sensors;
    const p = state.pose || {};

    // Firmware-raised system warnings (sticky-until-power-cycle). Keep
    // the banner in sync on every state tick so a transient WebSocket
    // reconnect doesn't leave the UI out of step with the robot.
    updateSystemWarnings(s);

    // Sync experiment panel with server state on every broadcast.
    // This handles browser refresh and WebSocket reconnect — the
    // dashboard picks up the runner's current state immediately
    // instead of showing a stale 'idle' until the next event.
    if (state.experiment && state.experiment.state && state.experiment.state !== expState.current) {
        setExpState(state.experiment.state);
    }

    // Pose
    T('d-pose-x', (p.x || 0).toFixed(3) + ' m');
    T('d-pose-y', (p.y || 0).toFixed(3) + ' m');
    T('d-pose-theta', (p.thetaDeg || 0).toFixed(1) + '\u00B0');

    // Velocities
    const v = s.vel || [0, 0, 0, 0];
    T('d-vel-front', (v[0] || 0).toFixed(2) + ' / ' + (v[1] || 0).toFixed(2));
    T('d-vel-rear', (v[2] || 0).toFixed(2) + ' / ' + (v[3] || 0).toFixed(2));

    // Encoders
    const enc = s.enc || [0, 0, 0, 0];
    T('d-enc-front', enc[0] + ' / ' + enc[1]);
    T('d-enc-rear', enc[2] + ' / ' + enc[3]);

    // IMU
    if (s.imu) {
        T('d-imu-yaw', (s.imu.yaw || 0).toFixed(1) + '\u00B0');
        T('d-imu-pitch', (s.imu.pitch || 0).toFixed(1) + '\u00B0');
        T('d-imu-roll', (s.imu.roll || 0).toFixed(1) + '\u00B0');
        T('d-imu-gz', (s.imu.gyroZ || 0).toFixed(2) + ' rad/s');
        T('d-imu-yaw-big', (s.imu.yaw || 0).toFixed(1) + '\u00B0');
        drawOrientation(s.imu.yaw);
    }

    // Calibration dots
    if (s.cal) {
        updateCalDot('d-cal-sys', s.cal.sys);
        updateCalDot('d-cal-gyro', s.cal.gyro);
        updateCalDot('d-cal-accel', s.cal.accel);
        updateCalDot('d-cal-mag', s.cal.mag);
    }

    // Command
    const cmd = state.command || {};
    T('d-cmd-vx', (cmd.vx || 0).toFixed(2) + ' m/s');
    T('d-cmd-vy', (cmd.vy || 0).toFixed(2) + ' m/s');
    T('d-cmd-w', (cmd.omega || 0).toFixed(2) + ' rad/s');

    // Logging
    T('d-log-rows', (state.logging || {}).rowCount || 0);

    // Robot heap — format bytes as KiB with 1 decimal, render '--' when
    // the firmware didn't include the field (older builds or pre-connect).
    const fmtKiB = (bytes) => bytes > 0 ? (bytes / 1024).toFixed(1) + ' KiB' : '--';
    T('d-heap-free', fmtKiB(s.heap?.free));
    T('d-heap-min',  fmtKiB(s.heap?.min));

    // Tier. Explicit number check — `state.config.tier === 0` must not
    // be dropped as falsy, otherwise the dropdown can't sync to tier 0
    // and any server→client sync of that tier silently no-ops.
    if (state.config && typeof state.config.tier === 'number') {
        applyTierVisibility(state.config.tier);
    }

    // Map. When a run is active and we captured a world-frame origin
    // from preflight, project body-frame odometry into world frame
    // before updating the map — so the trail visually begins at the
    // ghost (world-frame startHint ≈ measuredStartPose) rather than
    // snapping to the map origin. Outside a run the pose is shown
    // verbatim.
    if (robotMap) {
        const startPose = expState.measuredStartPose;
        const runActive = expState.current === 'running'
            || expState.current === 'awaiting_ground_truth';
        if (startPose && runActive) {
            const bx = p.x || 0;
            const by = p.y || 0;
            const bt = p.theta || 0;
            const c = Math.cos(startPose.theta);
            const s = Math.sin(startPose.theta);
            const wx = startPose.x + c * bx - s * by;
            const wy = startPose.y + s * bx + c * by;
            const wt = startPose.theta + bt;
            robotMap.updatePose(wx, wy, wt);
        } else {
            robotMap.updatePose(p.x || 0, p.y || 0, p.theta || 0);
        }
    }
}

function updateCalDot(id, level) {
    const el = document.getElementById(id);
    if (!el) return;
    el.className = 'cal-dot cal-' + Math.min(level || 0, 3);
}

// ============================================
// System Warning Banner
// ============================================
//
// Driven by sticky firmware fault flags in the sensor stream. Currently
// wired for imuStuck — the BNO055 stuck-read watchdog — but structured
// so additional fault sources can be surfaced by extending the list.
function updateSystemWarnings(sensors) {
    const banner = document.getElementById('sys-warning-banner');
    const textEl = document.getElementById('sys-warning-text');
    const hintEl = document.getElementById('sys-warning-hint');
    if (!banner || !textEl || !hintEl) return;

    const warnings = [];
    if (sensors && sensors.imuStuck) {
        warnings.push({
            text: '⚠ IMU NOT RESPONDING — gyro readings frozen',
            hint: 'Power-cycle the robot (full power off, then on) to recover.',
        });
    }

    if (warnings.length === 0) {
        banner.classList.remove('active');
        return;
    }

    // Show the first (highest priority) warning. Keeping this simple —
    // concurrent multi-warning is vanishingly rare and a single banner
    // is easier to read than a stack.
    textEl.textContent = warnings[0].text;
    hintEl.textContent = warnings[0].hint;
    banner.classList.add('active');
}

// ============================================
// Tier-Based Visibility
// ============================================

function applyTierVisibility(tier) {
    // Tier pills: always sync on every state broadcast so a dropped
    // setTier is visibly reverted (pending falls off, old pill stays
    // active). This runs even when tier === currentTier so the pending
    // class gets cleared after a no-op retry.
    document.querySelectorAll('.tier-pill').forEach((btn) => {
        const t = Number(btn.dataset.tier);
        btn.classList.toggle('active', t === tier);
        btn.classList.remove('pending');
    });

    if (tier === currentTier) return;
    currentTier = tier;
    // Re-evaluate tier-0-and-demo warning banner visibility.
    if (typeof _updateDemoWarning === 'function') _updateDemoWarning();

    // IMU card + orientation: tier 2+
    const imuCard = document.getElementById('d-imu-card');
    const orientArea = document.getElementById('d-orientation-area');
    if (imuCard) imuCard.style.display = tier >= 2 ? '' : 'none';
    if (orientArea) orientArea.style.display = tier >= 2 ? '' : 'none';

    // IMU weight slider: tier 2 only
    const imuWeightRow = document.getElementById('imu-weight-row');
    if (imuWeightRow) imuWeightRow.style.display = tier === 2 ? '' : 'none';

    // Map layers
    if (robotMap) {
        robotMap.setLayerVisible('imuFusion', tier >= 2);
    }
}

// ============================================
// Demo Mode
// ============================================

function startDemo() {
    demoState = { x: 0, y: 0, theta: 0, vx: 0, vy: 0, omega: 0, enc: [0,0,0,0] };
    demoInterval = setInterval(() => {
        // Simple kinematic simulation
        const dt = 0.1;
        demoState.x += demoState.vx * Math.cos(demoState.theta) * dt;
        demoState.y += demoState.vx * Math.sin(demoState.theta) * dt;
        demoState.theta += demoState.omega * dt;
        for (let i = 0; i < 4; i++) demoState.enc[i] += Math.round((Math.random() - 0.5) * 10);

        const state = {
            type: 'state',
            connected: true,
            pose: { x: demoState.x, y: demoState.y, theta: demoState.theta, thetaDeg: demoState.theta * 180 / Math.PI },
            sensors: {
                enc: demoState.enc,
                vel: [1.2, -1.2, -1.2, 1.2].map(v => v + (Math.random() - 0.5) * 0.2),
                imu: { yaw: demoState.theta * 180 / Math.PI, pitch: 0, roll: 0, gyroZ: demoState.omega },
                cal: { sys: 3, gyro: 3, accel: 3, mag: 2 },
            },
            command: { vx: demoState.vx, vy: demoState.vy, omega: demoState.omega },
            logging: { rowCount: 0 },
            config: { tier: currentTier },
        };
        updateDashboardState(state);
    }, 100);
}

function stopDemo() {
    clearInterval(demoInterval);
    demoInterval = null;
}

App.on('demoSend', (data) => {
    if (!demoState) return;
    if (data.type === 'cmd') {
        demoState.vx = data.vx || 0;
        demoState.vy = data.vy || 0;
        demoState.omega = data.w || 0;
    }
});

App.on('demoModeChanged', (enabled) => {
    if (enabled) startDemo(); else stopDemo();
});

// ============================================
// Modals
// ============================================

function setupModals() {
    // Robot IP modal
    const ipBtn = document.getElementById('d-robot-ip-btn');
    const ipModal = document.getElementById('d-robot-ip-modal');
    const ipCancel = document.getElementById('d-robot-ip-cancel');
    const ipConnect = document.getElementById('d-robot-ip-connect');
    const ipInput = document.getElementById('d-robot-ip-input');

    if (ipBtn) ipBtn.addEventListener('click', () => {
        ipModal.classList.add('visible');
        App.send({ type: 'getRobotIp' });
    });
    if (ipCancel) ipCancel.addEventListener('click', () => ipModal.classList.remove('visible'));

    function connectToIp() {
        const ip = ipInput.value.trim();
        if (/^(\d{1,3}\.){3}\d{1,3}$/.test(ip)) {
            App.send({ type: 'setRobotIp', ip });
            ipModal.classList.remove('visible');
            App.addLog('Connecting to ' + ip + '...', 'info');
        } else {
            T('d-robot-ip-status', 'Invalid IP format');
        }
    }
    if (ipConnect) ipConnect.addEventListener('click', connectToIp);
    if (ipInput) ipInput.addEventListener('keydown', (e) => { if (e.key === 'Enter') connectToIp(); });

    App.on('robotIp', (msg) => { if (ipInput) ipInput.value = msg.ip || ''; });
    App.on('robotIpError', (msg) => T('d-robot-ip-status', msg.error || 'Error'));
}

// ============================================
// Experiments panel
// ============================================

const expState = {
    current: 'idle',
    lastRunId: null,
    lastConfig: null,
    history: [],
    // World-frame origin captured from preflight_ok. Populated with
    // { x, y, theta (radians) } while a run is live so the dashboard
    // can project body-frame odometry into world frame — without this
    // the trail on the map renders at (0,0), disconnected from the
    // ghost-robot that's drawn in world frame.
    measuredStartPose: null,
    // True from arm() of a demo-mode run until the runner returns to idle.
    // Used to suppress preflight banners, hide the GT form on completion,
    // and gate the tier-0 demo warning.
    demoActive: false,
    // Cached catalog from the server's `trajectory_catalog` broadcast,
    // used to re-render the dropdown when the demo checkbox toggles
    // (demoOnly entries appear only when demo is checked).
    trajectories: null,
};

// Shared timer handle + start timestamp for the ground-truth prompt.
let _gtTimerId = null;
let _gtStartMs = 0;

function _resetGtFields() {
    for (const id of ['d-exp-gt-x', 'd-exp-gt-y', 'd-exp-gt-theta', 'd-exp-gt-notes']) {
        const el = document.getElementById(id);
        if (el) {
            el.value = '';
            el.classList.remove('exp-gt-missing');
        }
    }
    const pass = document.querySelector('input[name="d-exp-pf"][value="pass"]');
    if (pass) pass.checked = true;
    const err = document.getElementById('d-exp-gt-error');
    if (err) err.textContent = '';
}

function _startGtTimer() {
    _stopGtTimer();
    _gtStartMs = Date.now();
    const tick = () => {
        const el = document.getElementById('d-exp-gt-timer');
        if (!el) return;
        const s = Math.floor((Date.now() - _gtStartMs) / 1000);
        const mm = String(Math.floor(s / 60)).padStart(2, '0');
        const ss = String(s % 60).padStart(2, '0');
        el.textContent = `${mm}:${ss}`;
    };
    tick();
    _gtTimerId = setInterval(tick, 1000);
}

function _stopGtTimer() {
    if (_gtTimerId) {
        clearInterval(_gtTimerId);
        _gtTimerId = null;
    }
}

function _showPlaceHint(startHint) {
    // Show the "place the robot" banner with the per-trajectory world
    // start pose. Called on experiment_armed; hidden in setExpState when
    // the runner leaves the armed state.
    const wrap = document.getElementById('d-exp-place-hint');
    const text = document.getElementById('d-exp-place-hint-text');
    if (!wrap || !text) return;
    if (!startHint || !startHint.text) {
        wrap.style.display = 'none';
        text.textContent = '';
        return;
    }
    text.textContent = startHint.text;
    wrap.style.display = '';
}

function _hidePlaceHint() {
    const wrap = document.getElementById('d-exp-place-hint');
    if (wrap) wrap.style.display = 'none';
}

// Preflight status banner — separate from the place-hint so the two can
// coexist (the place hint explains *where* to put the robot, the
// preflight banner reports *whether* the camera-measured pose passed).
// Styles are neutral/success/error depending on `kind`.
function _showPreflightStatus(kind, text) {
    const el = document.getElementById('d-exp-preflight-status');
    if (!el) return;
    el.textContent = text;
    el.style.display = '';
    if (kind === 'ok') {
        el.style.border = '1px solid #10b981';
        el.style.background = 'rgba(16,185,129,0.10)';
        el.style.color = '#10b981';
    } else if (kind === 'fail') {
        el.style.border = '1px solid #ef4444';
        el.style.background = 'rgba(239,68,68,0.10)';
        el.style.color = '#ef4444';
    } else {
        // neutral / in-progress
        el.style.border = '1px solid var(--ak-text-muted, #888)';
        el.style.background = 'rgba(136,136,136,0.08)';
        el.style.color = 'var(--ak-text, #ccc)';
    }
}

function _hidePreflightStatus() {
    const el = document.getElementById('d-exp-preflight-status');
    if (el) el.style.display = 'none';
}

function _resetGtCaptureUI() {
    const status = document.getElementById('d-exp-gt-capture-status');
    if (status) { status.textContent = ''; status.style.color = ''; }
    const preview = document.getElementById('d-exp-gt-preview');
    if (preview) preview.style.display = 'none';
    const img = document.getElementById('d-exp-gt-preview-img');
    if (img) img.src = '';
    const btn = document.getElementById('d-exp-gt-capture');
    if (btn) { btn.disabled = false; btn.textContent = 'Capture with camera'; }
}

// Short attention beep using the WebAudio API — no asset required.
function _beepForGt() {
    try {
        const Ctx = window.AudioContext || window.webkitAudioContext;
        if (!Ctx) return;
        const ctx = new Ctx();
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = 'sine';
        osc.frequency.value = 880;
        gain.gain.value = 0.08;
        osc.connect(gain).connect(ctx.destination);
        osc.start();
        osc.stop(ctx.currentTime + 0.22);
        osc.onended = () => ctx.close();
    } catch { /* browser blocks autoplay — silent-fail is fine */ }
}

function setExpState(next) {
    const prev = expState.current;
    expState.current = next;
    const statusEl = document.getElementById('d-exp-status');
    if (statusEl) {
        statusEl.textContent = next.replace(/_/g, ' ');
        statusEl.dataset.state = next;
    }
    // Button enablement by state
    const armBtn = document.getElementById('d-exp-arm');
    const startBtn = document.getElementById('d-exp-start');
    const abortBtn = document.getElementById('d-exp-abort');
    const gt = document.getElementById('d-exp-gt');
    const progress = document.getElementById('d-exp-progress');

    if (armBtn) armBtn.disabled = next !== 'idle';
    if (startBtn) startBtn.disabled = next !== 'armed';
    if (abortBtn) abortBtn.disabled = next !== 'armed' && next !== 'running';
    if (gt) gt.style.display = next === 'awaiting_ground_truth' ? '' : 'none';
    if (progress) progress.style.display = next === 'running' ? '' : 'none';

    // Hide WASD teleop reference during experiments so it doesn't
    // visually overlap the preflight banner / place hint / GT form
    // that all share the bottom of the map column. Teleop isn't
    // meaningful mid-run anyway — the trajectory has motor control.
    const kbHelp = document.getElementById('d-keyboard-help');
    if (kbHelp) kbHelp.style.display = next === 'idle' ? '' : 'none';

    // Start-position banner lives with the armed state only. Once the
    // runner transitions out (start, abort, error), hide the hint.
    if (next !== 'armed') _hidePlaceHint();

    // Preflight banner: clear it on any state transition AWAY from armed
    // (running, awaiting_ground_truth, aborted, idle). Within armed —
    // including repeated Start clicks after a failed preflight — the
    // banner stays so the operator can reference the rejection reason.
    // On a fresh arm (idle→armed), also clear so prior failures from
    // a previous run don't bleed through.
    if (next !== 'armed' || prev !== 'armed') _hidePreflightStatus();

    // Start-hint ghost on the map lives with `armed` only. Once the
    // runner has actually started (preflight_ok → running) or bailed
    // (aborted), the ghost is no longer useful and would overlap with
    // the live robot icon.
    if (next !== 'armed' && robotMap) robotMap.clearStartHint();

    // World-frame projection only applies while a run is live. Drop
    // the captured origin once we leave running / awaiting-GT, so
    // subsequent tele-op poses render in body frame as before.
    if (next !== 'running' && next !== 'awaiting_ground_truth') {
        expState.measuredStartPose = null;
    }

    // Ground-truth prompt side effects. On entry: reset form, start the
    // elapsed-since-complete timer, scroll into view, and beep so the
    // operator notices even if they were looking at the robot.
    if (next === 'awaiting_ground_truth' && prev !== 'awaiting_ground_truth') {
        _resetGtFields();
        _startGtTimer();
        _beepForGt();
        if (gt && typeof gt.scrollIntoView === 'function') {
            gt.scrollIntoView({ behavior: 'smooth', block: 'center' });
        }
        const xInput = document.getElementById('d-exp-gt-x');
        if (xInput) xInput.focus();
    } else if (next !== 'awaiting_ground_truth' && prev === 'awaiting_ground_truth') {
        _stopGtTimer();
        _resetGtCaptureUI();
    }

    // Lock config inputs while a run is live
    const locked = next === 'armed' || next === 'running' || next === 'awaiting_ground_truth';
    for (const id of ['d-exp-trajectory', 'd-exp-speed', 'd-exp-rep', 'd-exp-notes']) {
        const el = document.getElementById(id);
        if (el) el.disabled = locked;
    }
}

function appendHistoryRow({ runId, trajectory, speed, tier, rep, status, note }) {
    expState.history.push({ runId, trajectory, speed, tier, rep, status, note, time: Date.now() });
    const listEl = document.getElementById('d-exp-history-list');
    const countEl = document.getElementById('d-exp-history-count');
    if (!listEl) return;
    const empty = listEl.querySelector('.exp-history-empty');
    if (empty) empty.remove();
    const row = document.createElement('div');
    row.className = 'exp-history-row exp-history-' + status;
    row.innerHTML =
        `<span class="exp-h-traj">${trajectory}</span>`
        + ` <span class="exp-h-speed">${Number(speed).toFixed(2)}</span>`
        + ` <span class="exp-h-tier">tier${tier}</span>`
        + ` <span class="exp-h-rep">#${rep}</span>`
        + ` <span class="exp-h-status">${status}</span>`;
    if (note) row.title = note;
    listEl.insertBefore(row, listEl.firstChild);
    if (countEl) countEl.textContent = String(expState.history.length);
}

function readExperimentConfig() {
    return {
        trajectory: document.getElementById('d-exp-trajectory').value,
        speed: Number(document.getElementById('d-exp-speed').value),
        rep: Number(document.getElementById('d-exp-rep').value) || 1,
        operatorNotes: document.getElementById('d-exp-notes').value || '',
        demo: document.getElementById('d-exp-demo')?.checked === true,
    };
}

// Rebuild the trajectory <select> from the server-provided catalog. Called
// on the `trajectory_catalog` broadcast (one-shot at connect) and whenever
// the demo checkbox toggles — `demoOnly` entries are hidden unless demo
// mode is active, so adding a custom demo-only trajectory server-side is
// the only change needed to surface it in the dropdown.
function _rebuildTrajectoryDropdown() {
    const sel = document.getElementById('d-exp-trajectory');
    if (!sel || !expState.trajectories) return;
    const demoOn = document.getElementById('d-exp-demo')?.checked === true;
    const prev = sel.value;
    const visible = expState.trajectories.filter((t) => demoOn || !t.demoOnly);
    sel.innerHTML = visible.map((t) =>
        `<option value="${t.key}">${t.label}${t.demoOnly ? ' [demo]' : ''}</option>`,
    ).join('');
    // Preserve the operator's previous selection if it's still visible.
    if (visible.some((t) => t.key === prev)) sel.value = prev;
}

// Tier 0 + demo = no bounds guard, no pauses, no GT. Warn the operator so
// the abort button / E-STOP are on their mind. Banner appears the moment
// either tier or demo flips in and hides when both aren't true.
function _updateDemoWarning() {
    const banner = document.getElementById('d-exp-demo-warning');
    if (!banner) return;
    const demoOn = document.getElementById('d-exp-demo')?.checked === true
        || expState.demoActive === true;
    const show = demoOn && currentTier === 0;
    banner.style.display = show ? 'block' : 'none';
}

function initExperiments() {
    const armBtn = document.getElementById('d-exp-arm');
    const startBtn = document.getElementById('d-exp-start');
    const abortBtn = document.getElementById('d-exp-abort');
    const gtSave = document.getElementById('d-exp-gt-save');
    const demoCheck = document.getElementById('d-exp-demo');

    // Wire the trajectory catalog broadcast (one-shot, sent per connection)
    // and the demo-checkbox toggle so the dropdown always reflects the
    // current demoOnly filter.
    App.on('trajectory_catalog', (msg) => {
        expState.trajectories = Array.isArray(msg.trajectories) ? msg.trajectories : [];
        _rebuildTrajectoryDropdown();
    });
    if (demoCheck) {
        demoCheck.addEventListener('change', () => {
            _rebuildTrajectoryDropdown();
            _updateDemoWarning();
        });
    }

    if (armBtn) armBtn.addEventListener('click', () => {
        const cfg = readExperimentConfig();
        expState.lastConfig = cfg;
        App.send({ type: 'experiment_arm', ...cfg });
    });
    if (startBtn) startBtn.addEventListener('click', () => {
        // Preflight takes ~2.5 s (camera pipeline latency). Show a
        // neutral "running" banner immediately so the operator knows
        // the click registered and the snapshot is in flight — the
        // previous UX was a silent pause that looked like "START does
        // nothing". In demo mode the server short-circuits preflight,
        // so don't show the preflight status at all.
        if (!expState.demoActive) {
            _showPreflightStatus('neutral', 'Running preflight check (camera snapshot)...');
        }
        App.send({ type: 'experiment_start' });
    });
    if (abortBtn) abortBtn.addEventListener('click', () => {
        App.send({ type: 'experiment_abort', reason: 'ui' });
    });
    if (gtSave) gtSave.addEventListener('click', () => {
        const xEl = document.getElementById('d-exp-gt-x');
        const yEl = document.getElementById('d-exp-gt-y');
        const tEl = document.getElementById('d-exp-gt-theta');
        const nEl = document.getElementById('d-exp-gt-notes');
        const errEl = document.getElementById('d-exp-gt-error');
        const passFail = document.querySelector('input[name="d-exp-pf"]:checked')?.value || 'pass';

        // Required-field validation. Empty string would coerce to 0 and
        // silently submit (0, 0, 0) — useless data. Highlight missing
        // fields in red and show an inline error instead.
        const missing = [];
        for (const el of [xEl, yEl, tEl]) {
            if (!el) continue;
            const v = (el.value ?? '').trim();
            if (v === '' || !Number.isFinite(Number(v))) {
                el.classList.add('exp-gt-missing');
                missing.push(el.id.split('-').pop());
            } else {
                el.classList.remove('exp-gt-missing');
            }
        }
        if (missing.length) {
            if (errEl) errEl.textContent = `Missing: ${missing.join(', ')}. All three grid readings are required.`;
            return;
        }
        if (passFail === 'fail' && !(nEl?.value || '').trim()) {
            if (nEl) nEl.focus();
            if (errEl) errEl.textContent = 'Notes required when marking a run as Fail.';
            return;
        }
        if (errEl) errEl.textContent = '';

        App.send({
            type: 'experiment_ground_truth',
            xMeas: Number(xEl.value) / 100,
            yMeas: Number(yEl.value) / 100,
            thetaDegMeas: Number(tEl.value),
            passFail,
            notes: (nEl?.value || '').trim(),
        });

        // Auto-increment rep for convenience
        const repEl = document.getElementById('d-exp-rep');
        if (repEl) repEl.value = String(Number(repEl.value || 0) + 1);
        // Record in history using the last known run config
        if (expState.lastConfig && expState.lastRunId) {
            appendHistoryRow({
                runId: expState.lastRunId,
                trajectory: expState.lastConfig.trajectory,
                speed: expState.lastConfig.speed,
                tier: currentTier,
                rep: expState.lastConfig.rep,
                status: passFail,
                note: (nEl?.value || '').trim(),
            });
        }
        // Field reset happens via setExpState() once the runner leaves
        // the awaiting_ground_truth state — no manual clear needed here.
    });

    // "Capture with camera" — calls the aruco snapshot service via the
    // server, auto-fills x/y/θ from the detected pose. If the service is
    // down or the marker isn't visible, falls back to manual entry.
    const gtCapture = document.getElementById('d-exp-gt-capture');
    const gtCaptureStatus = document.getElementById('d-exp-gt-capture-status');
    if (gtCapture) gtCapture.addEventListener('click', () => {
        gtCapture.disabled = true;
        gtCapture.textContent = 'Capturing…';
        if (gtCaptureStatus) {
            gtCaptureStatus.textContent = 'Querying snapshot service…';
            gtCaptureStatus.style.color = 'var(--ak-text-muted)';
        }
        if (!App.send({ type: 'experiment_capture_snapshot', label: 'end' })) {
            // WS dropped — no result will come. Restore UI immediately.
            gtCapture.disabled = false;
            gtCapture.textContent = 'Capture with camera';
            if (gtCaptureStatus) {
                gtCaptureStatus.textContent = 'Not connected to server. Type values manually.';
                gtCaptureStatus.style.color = 'var(--ak-error, #ef4444)';
            }
        }
    });

    App.on('experimentSnapshotResult', (msg) => {
        if (gtCapture) {
            gtCapture.disabled = false;
            gtCapture.textContent = 'Capture with camera';
        }
        if (!msg.ok) {
            if (gtCaptureStatus) {
                gtCaptureStatus.textContent = `Failed: ${msg.error || 'unknown'}. Type values manually.`;
                gtCaptureStatus.style.color = 'var(--ak-error, #ef4444)';
            }
            App.addLog(`GT snapshot failed: ${msg.error}`, 'error');
            return;
        }
        // Populate the fields (form uses cm + degrees; service returns m + rad).
        const xEl = document.getElementById('d-exp-gt-x');
        const yEl = document.getElementById('d-exp-gt-y');
        const tEl = document.getElementById('d-exp-gt-theta');
        if (xEl) xEl.value = (msg.x * 100).toFixed(1);
        if (yEl) yEl.value = (msg.y * 100).toFixed(1);
        if (tEl) tEl.value = Number(msg.thetaDeg).toFixed(1);
        // Clear any prior error highlights.
        for (const el of [xEl, yEl, tEl]) if (el) el.classList.remove('exp-gt-missing');
        const errEl = document.getElementById('d-exp-gt-error');
        if (errEl) errEl.textContent = '';
        // Show the annotated image inline so the operator can eyeball
        // that the detector found the right marker.
        const preview = document.getElementById('d-exp-gt-preview');
        const img = document.getElementById('d-exp-gt-preview-img');
        const link = document.getElementById('d-exp-gt-preview-link');
        const metaEl = document.getElementById('d-exp-gt-preview-meta');
        if (img && msg.imagePath) {
            const url = '/api/snapshot-image?path=' + encodeURIComponent(msg.imagePath) + '&t=' + Date.now();
            img.src = url;
            if (link) link.href = url;
            if (metaEl) metaEl.textContent = `side=${Math.round(msg.sidePx)}px — ${msg.imagePath}`;
            if (preview) preview.style.display = '';
        }
        if (gtCaptureStatus) {
            gtCaptureStatus.textContent = `Filled from snapshot (side ${Math.round(msg.sidePx)} px). Review and Save.`;
            gtCaptureStatus.style.color = 'var(--ak-success, #10b981)';
        }
        App.addLog(`GT snapshot: x=${msg.x.toFixed(3)}m y=${msg.y.toFixed(3)}m θ=${msg.thetaDeg.toFixed(1)}°`, 'info');
    });

    // Explicit "Discard run" path — makes skipping a conscious action
    // (with a confirm) rather than a silent close-the-tab drop.
    const gtDiscard = document.getElementById('d-exp-gt-discard');
    if (gtDiscard) gtDiscard.addEventListener('click', () => {
        const ok = window.confirm(
            'Discard this run?\n\n'
            + 'The CSV will be kept but no ground truth will be recorded, so '
            + 'the analysis pipeline will skip it and you will need to re-run '
            + 'this rep. Proceed?',
        );
        if (!ok) return;
        App.send({ type: 'experiment_abort', reason: 'skipped_gt' });
    });

    // beforeunload guard: prevent accidentally refreshing or closing the
    // tab while a run is waiting on ground-truth input. The browser shows
    // a generic dialog; message text is largely ignored in modern browsers.
    window.addEventListener('beforeunload', (e) => {
        if (expState.current === 'awaiting_ground_truth') {
            e.preventDefault();
            e.returnValue = 'A run is waiting for its ground-truth measurement. '
                + 'Submit or discard the measurement first.';
            return e.returnValue;
        }
    });

    App.on('experiment_armed', (msg) => {
        expState.lastRunId = msg.runId;
        expState.demoActive = msg.config?.demo === true;
        const demoTag = expState.demoActive ? ' [DEMO]' : '';
        App.addLog(`Experiment armed: ${msg.runId}${demoTag}`, 'info');
        // Demo runs are off-grid visual demos — no place-hint, no start-hint
        // on the map. The place-hint banner assumes the robot is being
        // positioned on the tape grid, which isn't how demo mode is used.
        if (!expState.demoActive) {
            _showPlaceHint(msg.startHint);
            if (robotMap) robotMap.setStartHint(msg.startHint);
        } else {
            _hidePlaceHint();
            if (robotMap) robotMap.clearStartHint();
        }
        _hidePreflightStatus();
        _updateDemoWarning();
    });
    App.on('experiment_started', (msg) => {
        App.addLog(`Experiment started: ${msg.runId}`, 'info');
        _hidePreflightStatus();
        if (robotMap) {
            robotMap.clearTrail();
            robotMap.clearStartHint();
        }
    });
    // Preflight outcome listeners — until this block was added, the
    // server's preflight_failed broadcast had no UI handler, so a
    // rejected snapshot (bad placement, marker occluded, homography
    // drift) looked like "START does nothing" to the operator. Now
    // every outcome lands in the status banner and the app log.
    App.on('experiment_preflight_ok', (msg) => {
        const m = msg.measured || {};
        const coord = (Number.isFinite(m.x) && Number.isFinite(m.y) && Number.isFinite(m.thetaDeg))
            ? ` — measured (${m.x.toFixed(2)}, ${m.y.toFixed(2)}, ${m.thetaDeg.toFixed(1)}°)`
            : '';
        _showPreflightStatus('ok', `Preflight passed${coord}. Starting trajectory...`);
        App.addLog(`Preflight ok${coord}`, 'info');
        // Capture the world-frame origin so the trail renders in world
        // frame. Runner-side odometry resets to (0,0,0) on arm — this
        // pose is what that zero means in world coordinates.
        if (Number.isFinite(m.x) && Number.isFinite(m.y) && Number.isFinite(m.thetaDeg)) {
            expState.measuredStartPose = {
                x: m.x,
                y: m.y,
                theta: m.thetaDeg * Math.PI / 180,
            };
            if (robotMap) robotMap.clearTrail();
        }
    });
    App.on('experiment_preflight_failed', (msg) => {
        const reason = msg.reason || 'unknown';
        _showPreflightStatus('fail', `Preflight FAILED — ${reason}`);
        App.addLog(`Preflight failed: ${reason}`, 'error');
    });
    App.on('experiment_tick', (msg) => {
        const bar = document.getElementById('d-exp-progress-bar');
        const txt = document.getElementById('d-exp-progress-text');
        if (bar && msg.totalMs) {
            const pct = Math.max(0, Math.min(100, (msg.elapsedMs / msg.totalMs) * 100));
            bar.style.width = pct.toFixed(1) + '%';
        }
        if (txt) {
            // Multi-waypoint runs report waypointsCompleted / waypointsTotal
            // alongside the legacy segment counter. Prefer the waypoint
            // framing when it's present; fall back to the raw segment
            // counter for older single-chunk trajectories.
            if (Number.isFinite(msg.waypointsTotal) && msg.waypointsTotal > 0) {
                const nextLabel = msg.currentChunkLabel ? ` → ${msg.currentChunkLabel}` : '';
                txt.textContent = `Waypoint ${msg.waypointsCompleted + 1} / ${msg.waypointsTotal}${nextLabel}`;
            } else {
                txt.textContent = `Segment ${msg.segmentIdx + 1} — ${Math.round(msg.elapsedMs)} / ${Math.round(msg.totalMs)} ms`;
            }
        }
    });
    App.on('experiment_completed', (msg) => {
        const demoTag = msg.demo ? ' [DEMO]' : '';
        App.addLog(`Experiment completed${demoTag}: ${msg.runId} (${msg.durationMs} ms)`, 'info');
        // Demo runs skip the GT state entirely — the runner returns straight
        // to idle and this message lands with demo:true. History row gets a
        // demo tag and we bail before the GT-form autofill code below.
        if (msg.demo) {
            if (expState.lastConfig && msg.runId) {
                appendHistoryRow({
                    runId: msg.runId,
                    trajectory: expState.lastConfig.trajectory,
                    speed: expState.lastConfig.speed,
                    tier: currentTier,
                    rep: expState.lastConfig.rep,
                    status: 'demo',
                    note: `demo ${msg.durationMs} ms`,
                });
            }
            expState.demoActive = false;
            _updateDemoWarning();
            return;
        }
        // Auto-fill the GT form from the last successful waypoint snapshot
        // so the operator can just review + Save. Manual entry still works
        // if the camera missed the final waypoint.
        if (msg.lastWaypointGt) {
            const xEl = document.getElementById('d-exp-gt-x');
            const yEl = document.getElementById('d-exp-gt-y');
            const tEl = document.getElementById('d-exp-gt-theta');
            if (xEl) xEl.value = (msg.lastWaypointGt.x * 100).toFixed(1);
            if (yEl) yEl.value = (msg.lastWaypointGt.y * 100).toFixed(1);
            if (tEl) tEl.value = Number(msg.lastWaypointGt.theta_deg).toFixed(1);
            for (const el of [xEl, yEl, tEl]) if (el) el.classList.remove('exp-gt-missing');
        }
        const status = document.getElementById('d-exp-gt-capture-status');
        if (status && Number.isFinite(msg.waypointsTotal) && msg.waypointsTotal > 0) {
            const ok = msg.waypointsOk;
            const total = msg.waypointsTotal;
            let color = 'var(--ak-warning, #f59e0b)';
            if (ok === total) color = 'var(--ak-success, #10b981)';
            else if (ok === 0) color = 'var(--ak-error, #ef4444)';
            status.textContent = `Camera captured ${ok}/${total} waypoints. Review values and Save.`;
            status.style.color = color;
        }
    });
    App.on('experiment_aborted', (msg) => {
        App.addLog(`Experiment aborted (${msg.reason}): ${msg.runId}`, 'error');
        if (expState.lastConfig && msg.runId) {
            appendHistoryRow({
                runId: msg.runId,
                trajectory: expState.lastConfig.trajectory,
                speed: expState.lastConfig.speed,
                tier: currentTier,
                rep: expState.lastConfig.rep,
                status: expState.demoActive ? 'demo-aborted' : 'aborted',
                note: msg.reason,
            });
        }
        expState.demoActive = false;
        _updateDemoWarning();
    });
    App.on('experiment_state', (msg) => {
        setExpState(msg.to);
    });
    App.on('experiment_error', (msg) => {
        App.addLog(`Experiment error: ${msg.error}`, 'error');
    });

    setExpState('idle');
}

// ============================================
// Init
// ============================================

function init() {
    initJoystick();

    // Init map — camera usable workspace (2.7 × 1.1 m, origin centred).
    // X ∈ [−1.35, +1.35] is the long axis; Y ∈ [−0.55, +0.55] the short.
    // Matches the RobotMap class default but explicit here so a future
    // change to the class default doesn't silently drift the dashboard.
    const mapCanvas = document.getElementById('d-map-canvas');
    if (mapCanvas) {
        robotMap = new RobotMap(mapCanvas);
        robotMap.setGridBounds(-1.35, 1.35, -0.55, 0.55);
    }

    // Sliders
    const speedSlider = document.getElementById('d-speed');
    const turnSlider = document.getElementById('d-turn');

    if (speedSlider) speedSlider.addEventListener('input', (e) => {
        maxSpeed = Number(e.target.value) / 100;
        T('d-speed-val', maxSpeed.toFixed(2));
    });
    if (turnSlider) turnSlider.addEventListener('input', (e) => {
        maxTurnSpeed = Number(e.target.value) / 100;
        T('d-turn-val', maxTurnSpeed.toFixed(2));
    });

    // Heading-hold toggle. Sends the firmware's set_heading_hold message.
    // Default off (matches firmware boot state). The ack's enabled field is
    // treated as the source of truth for the checkbox if it disagrees with
    // what we sent (e.g. firmware rejects or another client toggles).
    const headingHoldCheckbox = document.getElementById('d-heading-hold');
    if (headingHoldCheckbox) {
        headingHoldCheckbox.addEventListener('change', (e) => {
            App.send({ type: 'set_heading_hold', enabled: e.target.checked });
        });
    }
    App.on('ack', (msg) => {
        if (msg.cmd === 'set_heading_hold' && headingHoldCheckbox) {
            headingHoldCheckbox.checked = !!msg.enabled;
        }
    });

    // Rotate buttons (press-and-hold). Pointer events so both mouse and
    // touch work; pointerleave/pointercancel ensure the command releases
    // even if the gesture leaves the button before release.
    const attachRotateButton = (btn, side) => {
        if (!btn) return;
        const press = (e) => {
            e.preventDefault();
            btn.setPointerCapture?.(e.pointerId);
            btn.classList.add('active');
            setRotateButton(side, true);
        };
        const release = (e) => {
            e.preventDefault?.();
            btn.classList.remove('active');
            setRotateButton(side, false);
        };
        btn.addEventListener('pointerdown', press);
        btn.addEventListener('pointerup', release);
        btn.addEventListener('pointercancel', release);
        btn.addEventListener('pointerleave', release);
    };
    attachRotateButton(document.getElementById('d-rotate-left'), 'left');
    attachRotateButton(document.getElementById('d-rotate-right'), 'right');

    // Buttons
    document.getElementById('d-stop-btn')?.addEventListener('click', () => App.emergencyStop());
    document.getElementById('d-reset-btn')?.addEventListener('click', () => {
        App.send({ type: 'resetPose' });
        if (robotMap) robotMap.clearTrail();
        App.addLog('Pose reset', 'info');
    });
    document.getElementById('d-zero-imu-btn')?.addEventListener('click', () => {
        App.send({ type: 'zero_imu' });
        App.addLog('Zero IMU sent', 'info');
    });

    // Map controls
    document.getElementById('d-zoom-in')?.addEventListener('click', () => robotMap?.zoomIn());
    document.getElementById('d-zoom-out')?.addEventListener('click', () => robotMap?.zoomOut());
    document.getElementById('d-clear-trail')?.addEventListener('click', () => robotMap?.clearTrail());
    document.getElementById('d-center')?.addEventListener('click', () => robotMap?.centerOnRobot());
    document.getElementById('d-apply-grid')?.addEventListener('click', () => {
        const w = parseFloat(document.getElementById('d-grid-w').value) || 2;
        const h = parseFloat(document.getElementById('d-grid-h').value) || 2;
        if (robotMap) robotMap.setGridSize(w, h);
    });

    // Layer toggles
    document.querySelectorAll('#d-layer-toggles input[type="checkbox"]').forEach(cb => {
        cb.addEventListener('change', () => {
            if (robotMap) robotMap.setLayerVisible(cb.dataset.layer, cb.checked);
        });
    });

    // Subscribe to state
    App.on('state', updateDashboardState);

    // Experiments panel
    initExperiments();

    // Modals
    setupModals();

    // Auto-detect demo mode from URL
    if (location.search.includes('demo=1') || location.hash === '#demo') {
        const cb = document.getElementById('demo-mode');
        if (cb) { cb.checked = true; App.demoMode = true; startDemo(); }
    }
}

document.addEventListener('DOMContentLoaded', init);
