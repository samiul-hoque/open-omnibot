// ============================================
// Calibration View — Motors, IMU, Motor Cal
// ============================================

import { App } from './app.js';

const T = App.setText.bind(App);
const MOTOR_NAMES = ['L1', 'R1', 'R2', 'L2'];
const LOCKUP_THRESHOLD_MS = 500;
const LOCKUP_ENC_THRESHOLD = 5;

// Row-to-internal-index permutation for motor-gain arrays.
// The dashboard rows are in EXTERNAL wire order [L1, R1, R2, L2], but
// firmware emits gain arrays in INTERNAL order [L1, R1, L2, R2]. Rows
// 2 and 3 need a swap when reading gains from the firmware or writing
// manual-override gains back. (Sensor enc/vel arrays are already
// permuted firmware-side, so no permutation is needed for those.)
const ROW_TO_INT = [0, 1, 3, 2];

// --- Motor State ---
const motorState = [
    { pwm: 0, running: false, interval: null, lastEnc: 0, velocity: 0, lockup: false },
    { pwm: 0, running: false, interval: null, lastEnc: 0, velocity: 0, lockup: false },
    { pwm: 0, running: false, interval: null, lastEnc: 0, velocity: 0, lockup: false },
    { pwm: 0, running: false, interval: null, lastEnc: 0, velocity: 0, lockup: false },
];
const lockupTimers = [null, null, null, null];

// Uniformity
let uniformityRunning = false;
let uniformityInterval = null;
const uniformityData = [{}, {}, {}, {}];

// Motor calibration
let motorCalRunning = false;
let motorCalLogListener = null;

// ============================================
// Motor Cards
// ============================================

function createMotorCards() {
    const container = document.getElementById('c-motor-cards');
    if (!container) return;
    container.innerHTML = MOTOR_NAMES.map((name, i) => `
        <div class="motor-card" id="c-motor-card-${i}">
            <div class="motor-label">${name}</div>
            <div class="motor-pwm" id="c-pwm-val-${i}">0</div>
            <input type="range" min="-255" max="255" value="0" id="c-pwm-slider-${i}">
            <button class="motor-btn" id="c-motor-btn-${i}">Run</button>
            <div class="motor-stats">
                <div>Enc: <span id="c-enc-${i}">0</span></div>
                <div>Vel: <span id="c-vel-${i}">0.00</span> rad/s</div>
            </div>
            <div class="lockup-warn" id="c-lockup-${i}">POSSIBLE LOCKUP</div>
        </div>
    `).join('');

    // Wire sliders and buttons
    for (let i = 0; i < 4; i++) {
        const slider = document.getElementById('c-pwm-slider-' + i);
        slider.addEventListener('input', () => {
            motorState[i].pwm = Number(slider.value);
            T('c-pwm-val-' + i, slider.value);
        });

        const btn = document.getElementById('c-motor-btn-' + i);
        btn.addEventListener('click', () => toggleMotor(i));
    }
}

function toggleMotor(i) {
    const m = motorState[i];
    const btn = document.getElementById('c-motor-btn-' + i);
    if (m.running) {
        clearInterval(m.interval);
        m.interval = null;
        m.running = false;
        btn.textContent = 'Run';
        btn.classList.remove('running');
        App.send({ type: 'motor_test', motor: i, pwm: 0 });
    } else {
        m.running = true;
        btn.textContent = 'Stop';
        btn.classList.add('running');
        m.interval = setInterval(() => {
            App.send({ type: 'motor_test', motor: i, pwm: m.pwm });
        }, 200);
    }
}

// ============================================
// Uniformity Test
// ============================================

function createUniformityBars() {
    const container = document.getElementById('c-uniformity-bars');
    if (!container) return;
    container.innerHTML = MOTOR_NAMES.map((name, i) => `
        <div class="uniformity-bar-row">
            <span class="uniformity-bar-label">${name}</span>
            <div class="uniformity-bar-bg"><div class="uniformity-bar-fill" id="c-ubar-${i}" style="width:0%;"></div></div>
            <span class="uniformity-bar-pct" id="c-upct-${i}">--</span>
        </div>
    `).join('');
}

function startUniformity() {
    if (uniformityRunning) return;
    uniformityRunning = true;
    const pwm = Number(document.getElementById('c-uniformity-pwm').value);
    for (let i = 0; i < 4; i++) {
        uniformityData[i] = { velocity: 0, encDelta: 0, lastEnc: 0 };
    }
    uniformityInterval = setInterval(() => {
        App.send({ type: 'motor_test', motor: 4, pwm });
    }, 200);
}

function stopUniformity() {
    uniformityRunning = false;
    clearInterval(uniformityInterval);
    uniformityInterval = null;
    App.send({ type: 'motor_test', motor: 4, pwm: 0 });

    // Compute summary
    const vels = uniformityData.map(d => Math.abs(d.velocity));
    const mean = vels.reduce((a, b) => a + b, 0) / 4;
    if (mean > 0.1) {
        const summary = MOTOR_NAMES.map((n, i) => {
            const pct = Math.round((vels[i] / mean) * 100);
            return `${n}: ${pct}%`;
        }).join(' | ');
        T('c-uniformity-summary', summary);
    }
}

function updateUniformityBars() {
    const vels = uniformityData.map(d => Math.abs(d.velocity));
    const mean = vels.reduce((a, b) => a + b, 0) / 4;
    if (mean < 0.1) return;

    for (let i = 0; i < 4; i++) {
        const pct = (vels[i] / mean) * 100;
        const dev = Math.abs(pct - 100);
        const bar = document.getElementById('c-ubar-' + i);
        const pctEl = document.getElementById('c-upct-' + i);
        if (bar) {
            bar.style.width = Math.min(pct, 150) + '%';
            bar.style.background = dev < 10 ? 'var(--ak-success)' : dev < 25 ? 'var(--ak-warning)' : 'var(--ak-error)';
        }
        if (pctEl) pctEl.textContent = pct.toFixed(0) + '%';
    }
}

// ============================================
// Sensor Updates
// ============================================

function updateCalSensors(state) {
    if (!state.sensors) return;
    const s = state.sensors;
    const imu = s.imu || {};
    const cal = s.cal || {};
    const enc = s.enc || [0, 0, 0, 0];
    const vel = s.vel || [0, 0, 0, 0];

    // IMU display
    T('c-imu-yaw', (imu.yaw || 0).toFixed(1) + '\u00B0');
    T('c-imu-pitch', (imu.pitch || 0).toFixed(1) + '\u00B0');
    T('c-imu-roll', (imu.roll || 0).toFixed(1) + '\u00B0');
    T('c-imu-gz', (imu.gyroZ || 0).toFixed(3) + ' rad/s');
    T('c-imu-ax', (imu.accelX || 0).toFixed(3) + ' m/s\u00B2');
    T('c-imu-ay', (imu.accelY || 0).toFixed(3) + ' m/s\u00B2');

    // Cal indicators
    ['sys', 'gyro', 'accel', 'mag'].forEach(k => {
        const el = document.getElementById('c-cal-' + k);
        if (el) el.className = 'cal-dot cal-' + Math.min(cal[k] || 0, 3);
    });

    // Cal guidance
    updateCalGuidance(cal);

    // Motor data
    for (let i = 0; i < 4; i++) {
        T('c-enc-' + i, enc[i]);
        T('c-vel-' + i, (vel[i] || 0).toFixed(2));

        const m = motorState[i];
        const encDelta = Math.abs(enc[i] - m.lastEnc);
        m.velocity = Number(vel[i]);
        m.lastEnc = enc[i];

        // Lockup detection
        if (m.running && Math.abs(m.pwm) > 50) {
            if (encDelta < LOCKUP_ENC_THRESHOLD) {
                if (!lockupTimers[i]) {
                    lockupTimers[i] = setTimeout(() => {
                        m.lockup = true;
                        const warn = document.getElementById('c-lockup-' + i);
                        if (warn) warn.classList.add('visible');
                    }, LOCKUP_THRESHOLD_MS);
                }
            } else {
                clearTimeout(lockupTimers[i]);
                lockupTimers[i] = null;
                m.lockup = false;
                const warn = document.getElementById('c-lockup-' + i);
                if (warn) warn.classList.remove('visible');
            }
        } else {
            clearTimeout(lockupTimers[i]);
            lockupTimers[i] = null;
            m.lockup = false;
            const warn = document.getElementById('c-lockup-' + i);
            if (warn) warn.classList.remove('visible');
        }

        // Uniformity
        if (uniformityRunning) {
            uniformityData[i].velocity = Number(vel[i]);
            uniformityData[i].encDelta = encDelta;
        }
    }

    if (uniformityRunning) updateUniformityBars();

    // Cal mode banner
    const anyRunning = motorState.some(m => m.running) || uniformityRunning;
    const banner = document.getElementById('c-cal-banner');
    if (banner) banner.classList.toggle('visible', anyRunning);
}

function updateCalGuidance(cal) {
    const el = document.getElementById('c-cal-guidance');
    if (!el) return;
    const msgs = [];
    if ((cal.gyro || 0) < 3) msgs.push('Gyro: Keep the robot completely still.');
    if ((cal.mag || 0) < 3) msgs.push('Mag: Slowly rotate robot in figure-8 pattern.');
    if ((cal.accel || 0) < 3) msgs.push('Accel: Place robot on each of 6 faces briefly.');
    if (msgs.length > 0 && (cal.sys || 0) < 3) msgs.push('System: Complete gyro, accel, and mag first.');

    if (msgs.length > 0) {
        el.style.display = '';
        el.textContent = msgs.join(' ');
    } else {
        el.style.display = 'none';
    }
}

// ============================================
// Motor Calibration
// ============================================

function updateMotorGainsUI(fwd, rev) {
    for (let i = 0; i < 4; i++) {
        // Row i (external label) gets the gain value at internal index ROW_TO_INT[i]
        const vf = Number(fwd[ROW_TO_INT[i]]), vr = Number(rev[ROW_TO_INT[i]]);
        const ef = document.getElementById('c-gf-' + i);
        const er = document.getElementById('c-gr-' + i);
        if (ef) {
            ef.textContent = vf.toFixed(4);
            const dev = Math.abs(vf - 1.0);
            ef.style.color = dev < 0.02 ? 'var(--ak-success)' : dev < 0.05 ? 'var(--ak-warning)' : 'var(--ak-error)';
        }
        if (er) {
            er.textContent = vr.toFixed(4);
            const dev = Math.abs(vr - 1.0);
            er.style.color = dev < 0.02 ? 'var(--ak-success)' : dev < 0.05 ? 'var(--ak-warning)' : 'var(--ak-error)';
        }
        const mf = document.getElementById('c-mf-' + i);
        const mr = document.getElementById('c-mr-' + i);
        if (mf) mf.value = vf.toFixed(3);
        if (mr) mr.value = vr.toFixed(3);
    }
}

function startAutoCal() {
    if (motorCalRunning) return;
    motorCalRunning = true;
    document.getElementById('c-mcal-progress').style.display = 'block';
    document.getElementById('c-mcal-result').style.display = 'none';
    document.getElementById('c-mcal-bar').style.width = '0%';
    T('c-mcal-progress-text', 'Starting calibration...');
    const btn = document.getElementById('c-btn-auto-cal');
    if (btn) { btn.disabled = true; btn.textContent = 'Running...'; }
    App.send({ type: 'start_motor_cal' });
    App.addLog('Auto motor calibration started', 'info');

    motorCalLogListener = (msg) => {
        const text = msg.msg || msg;
        if (typeof text === 'string' && text.includes('Motor cal:')) {
            const match = text.match(/step (\d+)\/(\d+)/);
            if (match) {
                const pct = (parseInt(match[1]) / parseInt(match[2])) * 100;
                document.getElementById('c-mcal-bar').style.width = pct + '%';
                T('c-mcal-progress-text', text.replace('Motor cal: ', ''));
            }
        }
    };
}

function handleMotorCalResult(msg) {
    motorCalRunning = false;
    motorCalLogListener = null;
    document.getElementById('c-mcal-progress').style.display = 'none';
    const btn = document.getElementById('c-btn-auto-cal');
    if (btn) { btn.disabled = false; btn.textContent = 'Auto-Calibrate'; }

    const resultDiv = document.getElementById('c-mcal-result');
    const resultText = document.getElementById('c-mcal-result-text');
    if (resultDiv) resultDiv.style.display = 'block';

    if (msg.success && msg.gainsFwd && msg.gainsRev) {
        updateMotorGainsUI(msg.gainsFwd, msg.gainsRev);
        const labels = ['L1', 'R1', 'R2', 'L2'];
        const summary = labels.map((l, i) =>
            l + ' F:' + Number(msg.gainsFwd[ROW_TO_INT[i]]).toFixed(3) + ' R:' + Number(msg.gainsRev[ROW_TO_INT[i]]).toFixed(3),
        ).join(' | ');
        if (resultText) {
            resultText.textContent = 'Calibration complete: ' + summary;
            resultText.style.borderLeftColor = 'var(--ak-success)';
        }
        App.addLog('Motor cal complete: ' + summary, 'info');
    } else {
        if (resultText) {
            resultText.textContent = 'Calibration failed \u2014 motors did not move.';
            resultText.style.borderLeftColor = 'var(--ak-error)';
        }
        App.addLog('Motor calibration FAILED', 'error');
    }
}

// ============================================
// E-STOP (motor cleanup)
// ============================================

function emergencyStopMotors() {
    for (let i = 0; i < 4; i++) {
        const m = motorState[i];
        if (m.running) {
            clearInterval(m.interval);
            m.interval = null;
            m.running = false;
            const btn = document.getElementById('c-motor-btn-' + i);
            if (btn) { btn.textContent = 'Run'; btn.classList.remove('running'); }
        }
        const slider = document.getElementById('c-pwm-slider-' + i);
        if (slider) slider.value = 0;
        m.pwm = 0;
        T('c-pwm-val-' + i, '0');
    }
    if (uniformityRunning) stopUniformity();
}

App.onEmergencyStop(emergencyStopMotors);

// ============================================
// Ground-Truth Camera Calibration
// ============================================

// Calibration quality thresholds. Tuned on this rig: typical runs land
// at p50 ≈ 9 mm / p95 ≈ 20 mm / 30+ inliers, so 30/60/20 reject bad
// runs with headroom. Raise if the phone consistently sits farther or
// the grid has fewer visible intersections (e.g. robot occluding half
// the grid during calibration).
const GT_P50_BAD_MM = 30;
const GT_P95_BAD_MM = 60;
const GT_MIN_INLIERS = 20;

function setGtStatus(kind, msg) {
    const el = document.getElementById('c-gt-cal-status');
    if (!el) return;
    el.style.display = 'block';
    el.textContent = msg;
    let color = 'var(--ak-text-muted, #9ca3af)';
    if (kind === 'ok') color = 'var(--ak-success, #10b981)';
    else if (kind === 'warn') color = 'var(--ak-warning, #f59e0b)';
    else if (kind === 'fail') color = 'var(--ak-error, #ef4444)';
    el.style.color = color;
}

function fmtMm(v) { return (v === null || v === undefined) ? '—' : `${Number(v).toFixed(1)} mm`; }

function updateGtCal(msg) {
    if (msg.error) {
        T('c-gt-cal-at', '(no calibration file)');
        T('c-gt-cal-p50', '—');
        T('c-gt-cal-p95', '—');
        T('c-gt-cal-inliers', '—');
        setGtStatus('fail', msg.error);
        return;
    }
    T('c-gt-cal-at', msg.calibratedAt || '—');
    T('c-gt-cal-p50', fmtMm(msg.p50mm));
    T('c-gt-cal-p95', fmtMm(msg.p95mm));
    T('c-gt-cal-inliers', msg.inliers ?? '—');
    // Quality indicator based on the thresholds defined above.
    const p50Bad = msg.p50mm > GT_P50_BAD_MM;
    const p95Bad = msg.p95mm > GT_P95_BAD_MM;
    const inliersBad = (msg.inliers ?? 0) < GT_MIN_INLIERS;
    if (p50Bad || inliersBad) {
        setGtStatus('fail', 'Calibration quality is poor — rerun after re-seating the phone.');
    } else if (p95Bad) {
        setGtStatus('warn', 'p95 is noisy; tail of GT positions will have ~6 cm error.');
    } else {
        setGtStatus('ok', 'Calibration within spec.');
    }
}

function handleGtCalResult(msg) {
    const btn = document.getElementById('c-btn-gt-calibrate');
    if (btn) { btn.disabled = false; btn.textContent = 'Recalibrate'; }
    if (!msg.ok) {
        const reason = msg.error || `exit ${msg.exitCode}`;
        setGtStatus('fail', `Calibration failed: ${reason}`);
        App.addLog(`GT camera calibration FAILED: ${reason}`, 'error');
        if (msg.stderr) App.addLog('[gt-cal stderr] ' + msg.stderr.slice(-200), 'error');
        return;
    }
    // Populate the metric rows + quality indicator from this response.
    updateGtCal(msg);
    App.addLog(`GT camera calibration OK: p50=${fmtMm(msg.p50mm)}, inliers=${msg.inliers}`, 'info');
}

// Stop motors when leaving the calibration view to prevent orphaned intervals
App.on('viewChanged', (view) => {
    if (view !== 'calibration') {
        emergencyStopMotors();
    }
});

// ============================================
// Init
// ============================================

function init() {
    createMotorCards();
    createUniformityBars();

    // Uniformity controls
    const uPwm = document.getElementById('c-uniformity-pwm');
    if (uPwm) uPwm.addEventListener('input', (e) => T('c-uniformity-pwm-val', e.target.value));
    document.getElementById('c-uniformity-start')?.addEventListener('click', startUniformity);
    document.getElementById('c-uniformity-stop')?.addEventListener('click', stopUniformity);
    document.getElementById('c-stop-all')?.addEventListener('click', () => App.emergencyStop());

    // IMU buttons
    document.getElementById('c-btn-zero-imu')?.addEventListener('click', () => {
        App.send({ type: 'zero_imu' }); App.addLog('Zero IMU sent', 'info');
    });
    document.getElementById('c-btn-save-cal')?.addEventListener('click', () => {
        App.send({ type: 'save_imu_cal' }); App.addLog('Save IMU cal sent', 'info');
    });
    document.getElementById('c-btn-load-cal')?.addEventListener('click', () => {
        App.send({ type: 'load_imu_cal' }); App.addLog('Load IMU cal sent', 'info');
    });

    // Motor cal buttons
    document.getElementById('c-btn-auto-cal')?.addEventListener('click', startAutoCal);
    document.getElementById('c-btn-save-mcal')?.addEventListener('click', () => {
        App.send({ type: 'save_motor_cal' }); App.addLog('Save motor cal sent', 'info');
    });
    document.getElementById('c-btn-load-mcal')?.addEventListener('click', () => {
        App.send({ type: 'load_motor_cal' }); App.addLog('Load motor cal sent', 'info');
    });
    document.getElementById('c-btn-set-mcal')?.addEventListener('click', () => {
        // Manual override inputs are in external-row order; firmware
        // expects internal order. Permute rows 2/3 on the way out.
        const gainsFwd = [1, 1, 1, 1];
        const gainsRev = [1, 1, 1, 1];
        for (let i = 0; i < 4; i++) {
            gainsFwd[ROW_TO_INT[i]] = parseFloat(document.getElementById('c-mf-' + i).value) || 1.0;
            gainsRev[ROW_TO_INT[i]] = parseFloat(document.getElementById('c-mr-' + i).value) || 1.0;
        }
        App.send({ type: 'set_motor_gains', gainsFwd, gainsRev });
        App.addLog('Manual gains sent', 'info');
    });

    // Ground-truth camera buttons
    const gtBtn = document.getElementById('c-btn-gt-calibrate');
    const gtRefresh = document.getElementById('c-btn-gt-refresh');
    gtBtn?.addEventListener('click', () => {
        gtBtn.disabled = true;
        gtBtn.textContent = 'Calibrating…';
        setGtStatus('running', 'Running calibrate_homography.py — this takes ~5 s.');
        if (!App.send({ type: 'calibrate_camera' })) {
            // WS dropped — no result will come. Restore UI so the button
            // doesn't stick in "Calibrating…" forever.
            gtBtn.disabled = false;
            gtBtn.textContent = 'Recalibrate';
            setGtStatus('fail', 'Not connected to server. Reconnect and try again.');
        }
    });
    gtRefresh?.addEventListener('click', () => {
        App.send({ type: 'get_camera_calibration' });
    });
    // Populate on first load
    App.send({ type: 'get_camera_calibration' });

    // Subscribe to events
    App.on('state', updateCalSensors);
    App.on('motorCalResult', handleMotorCalResult);
    App.on('cameraCalibration', updateGtCal);
    App.on('cameraCalibrationResult', handleGtCalResult);

    App.on('robotLog', (msg) => {
        if (motorCalLogListener) motorCalLogListener(msg);
    });

    App.on('robotInfo', (info) => {
        if (info.motorGainsFwd) {
            updateMotorGainsUI(info.motorGainsFwd, info.motorGainsRev || [1,1,1,1]);
        }
    });

    App.on('ack', (msg) => {
        if (msg.cmd === 'save_imu_cal') App.addLog('IMU cal save: ' + (msg.success ? 'OK' : 'FAILED'), msg.success ? 'info' : 'error');
        else if (msg.cmd === 'load_imu_cal') App.addLog('IMU cal load: ' + (msg.success ? 'OK' : 'FAILED'), msg.success ? 'info' : 'error');
        else if (msg.cmd === 'save_motor_cal') {
            App.addLog('Motor cal save: ' + (msg.success ? 'OK' : 'FAILED'), msg.success ? 'info' : 'error');
            if (msg.gainsFwd) updateMotorGainsUI(msg.gainsFwd, msg.gainsRev);
        }
        else if (msg.cmd === 'load_motor_cal') {
            App.addLog('Motor cal load: ' + (msg.success ? 'OK' : 'FAILED'), msg.success ? 'info' : 'error');
            if (msg.gainsFwd) updateMotorGainsUI(msg.gainsFwd, msg.gainsRev);
        }
        else if (msg.cmd === 'set_motor_gains') {
            App.addLog('Motor gains applied', 'info');
            if (msg.gainsFwd) updateMotorGainsUI(msg.gainsFwd, msg.gainsRev);
        }
        else if (msg.cmd === 'set_openloop_cal') {
            const verdict = msg.ok
                ? 'OK'
                : 'FAILED' + (msg.error ? ' (' + msg.error + ')' : '');
            App.addLog('Open-loop cal save: ' + verdict, msg.ok ? 'info' : 'error');
        }
    });

    initOpenloopCalPanel();
}

// ============================================
// Tier-0 Open-Loop Calibration Panel
// ============================================
//
// Six directions, one row each. For each: the operator clicks Drive,
// the robot runs at the base PWM for the configured duration, the
// operator measures the displacement (cm for linear, degrees for
// yaw), and types it in. When Save is clicked, all six measurements
// are converted to m/s / rad/s and sent to firmware via
// set_openloop_cal.
//
// Drive is implemented without new firmware commands: we fan out
// 4 motor_test commands per 100 ms, multiplying the base PWM by the
// direction's sign vector. Calibration mode in firmware stays active
// because we refresh the motor_test commands well inside the 500 ms
// auto-clear window. Stops on duration elapsed or on any click of the
// top-bar E-STOP.

// Mirror of the firmware openloop_motor_table.cpp — see that file for
// the sign-convention rationale. Order: L1, R1, L2, R2 (internal).
const OL_DIR_LIST = [
    { key: 'fwd',      label: 'Forward (+X)',   signs: [+1, +1, +1, +1], kind: 'linear' },
    { key: 'back',     label: 'Backward (-X)',  signs: [-1, -1, -1, -1], kind: 'linear' },
    { key: 'strafe_l', label: 'Strafe L (+Y)',  signs: [+1, -1, -1, +1], kind: 'linear' },
    { key: 'strafe_r', label: 'Strafe R (-Y)',  signs: [-1, +1, +1, -1], kind: 'linear' },
    { key: 'yaw_ccw',  label: 'Yaw CCW (+ω)',   signs: [-1, +1, -1, +1], kind: 'yaw' },
    { key: 'yaw_cw',   label: 'Yaw CW (-ω)',    signs: [+1, -1, +1, -1], kind: 'yaw' },
];

let _olActiveInterval = null;

function _stopOpenloopDrive() {
    if (_olActiveInterval) {
        clearInterval(_olActiveInterval);
        _olActiveInterval = null;
    }
    App.send({ type: 'stop' });
}

// `dir.signs` is indexed in INTERNAL motor order [L1,R1,L2,R2] (mirrors
// the firmware table), but `motor_test` takes a WIRE-order index
// [L1,R1,R2,L2] — the firmware applies its own EXT_TO_INT={0,1,3,2}
// on receipt. Remap here so the wire index we send resolves to the
// correct internal motor. Without this, strafe rows produce yaw.
const INT_TO_EXT_MOTOR = [0, 1, 3, 2];

// Returns a Promise that resolves when the drive duration has elapsed
// and the stop has been sent, so the auto-cal orchestrator can await
// completion. Manual callers (the Drive button) still work by ignoring
// the return value.
//
// The 20 ms tick matches the firmware's 50 Hz main loop. Anything
// slower bumps into the motor-driver slew-rate limiter
// (MOTOR_MAX_PWM_STEP=15 per setMotorSpeed call). At 100 ms the PWM
// needs ~1 s to ramp 0→100, which biases the first auto-cal by ~50 %
// and — in a fwd→back chain — prevents the reverse torque from
// overcoming the forward coast, so "back" would drift forward.
const OL_TICK_MS = 20;

function _driveOpenloopDirection(dir, pwm, durationMs) {
    if (_olActiveInterval) {
        App.addLog('Open-loop drive already running; ignoring', 'error');
        return Promise.reject(new Error('drive already running'));
    }
    App.addLog(`Open-loop drive: ${dir.label} pwm=${pwm} for ${durationMs} ms`, 'info');
    const start = Date.now();
    return new Promise((resolve) => {
        const tick = () => {
            for (let i = 0; i < 4; i++) {
                App.send({ type: 'motor_test', motor: INT_TO_EXT_MOTOR[i], pwm: dir.signs[i] * pwm });
            }
            if (Date.now() - start >= durationMs) {
                _stopOpenloopDrive();
                resolve();
            }
        };
        tick();  // fire immediately
        _olActiveInterval = setInterval(tick, OL_TICK_MS);
    });
}

function _olRowMeasuredId(key) { return `c-ol-meas-${key}`; }
function _olRowSpeedId(key)    { return `c-ol-speed-${key}`; }

// ---- Tier-0 auto-cal ---------------------------------------------------
//
// Drives each direction in pairs (fwd↔back, strafe-L↔strafe-R, yaw-CCW↔
// yaw-CW) so the robot returns to near-origin between pairs, snapshots
// GT before/after each drive, and auto-fills the measured input.
// Linear rows fill in cm (world Δ rotated into body frame); yaw rows
// fill in degrees (normalized Δθ).

const AUTOCAL_PWM = 100;
const AUTOCAL_DRIVE_MS = 700;
// Settle must cover: (a) firmware's 500 ms calibration-mode timeout
// before PID can brake, (b) the PID's ~200–400 ms brake from
// end-of-drive velocity to rest, (c) margin. An earlier 500 ms value
// snapshotted while the robot was still coasting at ~0.4 m/s, which
// bled residual forward momentum into the next drive and made back
// runs undershoot by 80 %.
const AUTOCAL_SETTLE_MS = 1500;
const AUTOCAL_POS_TOL_M = 0.05;       // same as experiment preflight
const AUTOCAL_HEAD_TOL_DEG = 5;       // same as experiment preflight
const AUTOCAL_DRIFT_TOL_M = 0.10;     // between pairs — re-centre if over
const AUTOCAL_ORDER = ['fwd', 'back', 'strafe_l', 'strafe_r', 'yaw_ccw', 'yaw_cw'];

let _pendingCalSnapshot = null;

function _installCalSnapshotListener() {
    if (_installCalSnapshotListener._installed) return;
    // Event name is the camelCase-renamed version emitted by app.js
    // (`cal_snapshot_result` → `calSnapshotResult`), matching the
    // pattern used by experimentSnapshotResult, motorCalResult, etc.
    App.on('calSnapshotResult', (msg) => {
        if (_pendingCalSnapshot && msg.runId === _pendingCalSnapshot.runId) {
            clearTimeout(_pendingCalSnapshot.timeoutHandle);
            _pendingCalSnapshot.resolve(msg);
            _pendingCalSnapshot = null;
        }
    });
    _installCalSnapshotListener._installed = true;
}

function _requestCalSnapshot(label) {
    _installCalSnapshotListener();
    const runId = `cal_${Date.now()}_${Math.random().toString(36).slice(2, 6)}`;
    return new Promise((resolve, reject) => {
        const timeoutHandle = setTimeout(() => {
            if (_pendingCalSnapshot && _pendingCalSnapshot.runId === runId) {
                _pendingCalSnapshot = null;
                reject(new Error(`cal_snapshot(${label}) timed out after 10 s`));
            }
        }, 10000);
        _pendingCalSnapshot = { runId, resolve, timeoutHandle };
        const sent = App.send({ type: 'cal_snapshot', label, runId });
        if (!sent) {
            clearTimeout(timeoutHandle);
            _pendingCalSnapshot = null;
            reject(new Error('WebSocket not open; cal_snapshot dropped'));
        }
    });
}

function _sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

function _setAutocalStatus(text, kind = 'info') {
    const el = document.getElementById('c-ol-autocal-status');
    if (!el) return;
    el.textContent = text;
    el.style.color = kind === 'error' ? 'var(--ak-error, #ef4444)'
        : kind === 'success' ? 'var(--ak-success, #10b981)'
            : 'var(--ak-text-muted)';
}

function _setRowPending(key, text) {
    const el = document.getElementById(_olRowSpeedId(key));
    if (el) el.textContent = text;
}

// Body-frame Δ from two world-frame poses, rotated by -theta_before.
// Returns { dxBody, dyBody } in meters.
function _worldDeltaToBody(before, after) {
    const dxW = after.x - before.x;
    const dyW = after.y - before.y;
    const c = Math.cos(before.theta);
    const s = Math.sin(before.theta);
    return {
        dxBody:  c * dxW + s * dyW,
        dyBody: -s * dxW + c * dyW,
    };
}

function _signedLinearFromDelta(dirKey, d) {
    switch (dirKey) {
    case 'fwd':      return  d.dxBody;
    case 'back':     return -d.dxBody;
    case 'strafe_l': return  d.dyBody;
    case 'strafe_r': return -d.dyBody;
    default: throw new Error(`not a linear direction: ${dirKey}`);
    }
}

// Normalize a radian angle into (-π, π]. Guards against ±180° wraparound
// when Δθ = after.theta - before.theta crosses the seam.
function _normalizeAngleRad(a) {
    while (a >   Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
}

async function _autocalMeasureOne(dir) {
    _setRowPending(dir.key, 'snap before…');
    const before = await _requestCalSnapshot(`autocal_${dir.key}_before`);
    if (!before.ok) throw new Error(`snapshot before ${dir.key}: ${before.error}`);
    console.log(`[autocal] ${dir.key} before: x=${before.x.toFixed(3)} y=${before.y.toFixed(3)} θ=${before.thetaDeg.toFixed(1)}°`);

    _setRowPending(dir.key, 'driving…');
    await _driveOpenloopDirection(dir, AUTOCAL_PWM, AUTOCAL_DRIVE_MS);
    await _sleep(AUTOCAL_SETTLE_MS);

    _setRowPending(dir.key, 'snap after…');
    const after = await _requestCalSnapshot(`autocal_${dir.key}_after`);
    if (!after.ok) throw new Error(`snapshot after ${dir.key}: ${after.error}`);
    console.log(`[autocal] ${dir.key} after:  x=${after.x.toFixed(3)} y=${after.y.toFixed(3)} θ=${after.thetaDeg.toFixed(1)}°`);

    if (dir.kind === 'yaw') {
        // Yaw: the measured input is in degrees. Expected sign per
        // direction is handled the same way as linear — CCW wants a
        // positive Δθ, CW wants a negative Δθ.
        const dThetaRad = _normalizeAngleRad(after.theta - before.theta);
        const signedRad = dir.key === 'yaw_ccw' ? dThetaRad : -dThetaRad;
        const signedDeg = signedRad * 180 / Math.PI;
        console.log(
            `[autocal] ${dir.key} Δθworld=${(dThetaRad * 180 / Math.PI).toFixed(1)}° ` +
            `signed=${signedDeg.toFixed(1)}°`,
        );
        if (!Number.isFinite(signedDeg) || signedDeg <= 0) {
            throw new Error(
                `${dir.key}: angular displacement ${signedDeg.toFixed(1)}° ` +
                `is non-positive — wrong direction? (Δθ=${(dThetaRad*180/Math.PI).toFixed(1)}°)`,
            );
        }
        const input = document.getElementById(_olRowMeasuredId(dir.key));
        if (input) input.value = signedDeg.toFixed(1);
        _setRowPending(dir.key, `measured ${signedDeg.toFixed(1)}°`);
        return { before, after, deg: signedDeg };
    }

    // Linear
    const delta = _worldDeltaToBody(before, after);
    const signedM = _signedLinearFromDelta(dir.key, delta);
    console.log(
        `[autocal] ${dir.key} Δworld=(${(after.x-before.x).toFixed(3)}, ${(after.y-before.y).toFixed(3)}) ` +
        `Δbody=(${delta.dxBody.toFixed(3)}, ${delta.dyBody.toFixed(3)}) signed=${(signedM*100).toFixed(1)}cm`,
    );
    if (!Number.isFinite(signedM) || signedM <= 0) {
        throw new Error(
            `${dir.key}: body-frame displacement ${(signedM * 100).toFixed(1)} cm ` +
            `is non-positive — wrong direction? (world Δ=(${(after.x-before.x).toFixed(3)}, ${(after.y-before.y).toFixed(3)}), ` +
            `before θ=${before.thetaDeg.toFixed(1)}°, after θ=${after.thetaDeg.toFixed(1)}°)`,
        );
    }

    const cm = signedM * 100;
    const input = document.getElementById(_olRowMeasuredId(dir.key));
    if (input) input.value = cm.toFixed(1);
    _setRowPending(dir.key, `measured ${cm.toFixed(1)} cm`);
    return { before, after, cm };
}

async function _runAutocal(btn) {
    const prevLabel = btn.textContent;
    btn.disabled = true;
    btn.textContent = 'Running auto-cal…';

    // The Save handler divides measured displacement by the UI's
    // `Drive duration` field to derive speed. Auto-cal drives for a
    // fixed internal AUTOCAL_DRIVE_MS regardless of that field's
    // value, so pin the field to match before we start. Otherwise
    // a stale default (e.g. 3 s) would turn a correct 0.5 m/s cal
    // into 0.12 m/s, producing a worse tier-0 overshoot than before.
    const durEl = document.getElementById('c-ol-dur');
    if (durEl) durEl.value = (AUTOCAL_DRIVE_MS / 1000).toFixed(1);

    try {
        _setAutocalStatus('Preflight snapshot…', 'info');
        const pre = await _requestCalSnapshot('autocal_preflight');
        if (!pre.ok) throw new Error(`preflight: ${pre.error}`);
        if (Math.abs(pre.x) > AUTOCAL_POS_TOL_M || Math.abs(pre.y) > AUTOCAL_POS_TOL_M) {
            throw new Error(
                `robot not at origin: GT=(${pre.x.toFixed(3)}, ${pre.y.toFixed(3)}) m ` +
                `exceeds ±${AUTOCAL_POS_TOL_M} m — place robot at (0, 0) facing +X`,
            );
        }
        if (Math.abs(pre.thetaDeg) > AUTOCAL_HEAD_TOL_DEG) {
            throw new Error(
                `robot not facing +X: θ=${pre.thetaDeg.toFixed(1)}° ` +
                `exceeds ±${AUTOCAL_HEAD_TOL_DEG}° — re-orient robot`,
            );
        }
        _setAutocalStatus(`Preflight ok (${pre.x.toFixed(3)}, ${pre.y.toFixed(3)}, ${pre.thetaDeg.toFixed(1)}°)`, 'info');

        for (let i = 0; i < AUTOCAL_ORDER.length; i++) {
            const dir = OL_DIR_LIST.find(d => d.key === AUTOCAL_ORDER[i]);
            if (!dir) continue;

            // Between pairs (after back, after strafe_r, after yaw_cw),
            // check drift so we don't start the next direction already
            // far from origin. yaw pair check is included — small linear
            // creep during strafe can push the robot toward the edge.
            const atPairBoundary = (i === 2 || i === 4);
            if (atPairBoundary) {
                _setAutocalStatus('Drift check between pairs…', 'info');
                const mid = await _requestCalSnapshot(`autocal_mid_${i}`);
                if (!mid.ok) throw new Error(`drift-check snapshot: ${mid.error}`);
                if (Math.abs(mid.x) > AUTOCAL_DRIFT_TOL_M || Math.abs(mid.y) > AUTOCAL_DRIFT_TOL_M) {
                    throw new Error(
                        `robot drifted to (${mid.x.toFixed(3)}, ${mid.y.toFixed(3)}) ` +
                        `after pair ${i / 2} — re-centre and restart auto-cal`,
                    );
                }
            }

            _setAutocalStatus(`Measuring ${dir.label}…`, 'info');
            await _autocalMeasureOne(dir);
        }

        _setAutocalStatus('Auto-cal complete — review values and Save Open-Loop Cal.', 'success');
    } catch (err) {
        App.send({ type: 'stop' });  // belt-and-braces: robot halts on any failure
        _setAutocalStatus(`Aborted: ${err.message}`, 'error');
        App.addLog(`Auto-cal aborted: ${err.message}`, 'error');
    } finally {
        btn.disabled = false;
        btn.textContent = prevLabel;
    }
}

function initOpenloopCalPanel() {
    const tbody = document.getElementById('c-ol-rows');
    if (!tbody) return;

    tbody.innerHTML = '';
    for (const dir of OL_DIR_LIST) {
        const tr = document.createElement('tr');
        tr.style.borderTop = '1px solid var(--ak-border)';
        const unit = dir.kind === 'yaw' ? '°' : 'cm';
        tr.innerHTML = `
            <td style="padding:4px 6px;">${dir.label}</td>
            <td style="text-align:center;padding:4px 6px;">
                <button class="btn-secondary" data-ol-dir="${dir.key}" style="padding:2px 8px;font-size:0.8em;">Drive</button>
            </td>
            <td style="text-align:right;padding:4px 6px;">
                <input type="number" id="${_olRowMeasuredId(dir.key)}" step="0.1"
                       style="width:70px;text-align:right;" placeholder="${unit}">
                <span style="color:var(--ak-text-muted);margin-left:2px;">${unit}</span>
            </td>
            <td style="text-align:right;padding:4px 6px;color:var(--ak-text-muted);font-family:var(--font-mono, monospace);">
                <span id="${_olRowSpeedId(dir.key)}">—</span>
            </td>
        `;
        tbody.appendChild(tr);
    }

    // Drive-button handlers (delegated)
    tbody.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-ol-dir]');
        if (!btn) return;
        const dir = OL_DIR_LIST.find(d => d.key === btn.dataset.olDir);
        if (!dir) return;
        const pwm = Number(document.getElementById('c-ol-pwm').value) || 0;
        const durS = Number(document.getElementById('c-ol-dur').value) || 0.7;
        if (pwm < 30 || pwm > 255) {
            App.addLog('Base PWM must be 30–255', 'error');
            return;
        }
        _driveOpenloopDirection(dir, pwm, durS * 1000);
    });

    document.getElementById('c-btn-ol-refresh')?.addEventListener('click', () => {
        App.send({ type: 'get_info' });
    });

    document.getElementById('c-btn-ol-autocal')?.addEventListener('click', (e) => {
        _runAutocal(e.currentTarget);
    });

    document.getElementById('c-btn-ol-save')?.addEventListener('click', () => {
        const pwm = Number(document.getElementById('c-ol-pwm').value) || 0;
        const durS = Number(document.getElementById('c-ol-dur').value) || 0.7;
        const speeds = {};
        for (const dir of OL_DIR_LIST) {
            const measRaw = document.getElementById(_olRowMeasuredId(dir.key)).value;
            const meas = Number(measRaw);
            if (!Number.isFinite(meas) || meas <= 0) {
                App.addLog(`Measured value for ${dir.label} missing or non-positive`, 'error');
                return;
            }
            // Convert measured → speed in the firmware's units.
            //   Linear: cm / s → m / s
            //   Yaw:    deg / s → rad / s
            const speed = (dir.kind === 'yaw')
                ? (meas * Math.PI / 180) / durS
                : (meas / 100) / durS;
            speeds[dir.key] = speed;
            const el = document.getElementById(_olRowSpeedId(dir.key));
            if (el) el.textContent = speed.toFixed(3) + (dir.kind === 'yaw' ? ' rad/s' : ' m/s');
        }
        App.send({ type: 'set_openloop_cal', basePwm: pwm, speeds });
        App.addLog(`Open-loop cal sent (pwm=${pwm}, duration=${durS} s)`, 'info');
    });

    // Receive openloopCal state via robotInfo broadcasts.
    App.on('robotInfo', (msg) => {
        const olc = msg.openloopCal;
        const statusEl = document.getElementById('c-ol-status');
        if (!olc) {
            if (statusEl) statusEl.textContent = 'not supported by firmware';
            return;
        }
        if (statusEl) {
            statusEl.textContent = olc.valid
                ? `loaded (basePwm=${olc.basePwm})`
                : 'NOT CALIBRATED — tier-0 trajectories will reject';
            statusEl.style.color = olc.valid ? 'var(--ak-success, #10b981)' : 'var(--ak-error, #ef4444)';
        }
        // Update the computed-speed column with the actually-stored values.
        for (const dir of OL_DIR_LIST) {
            const v = olc.speeds?.[dir.key];
            const el = document.getElementById(_olRowSpeedId(dir.key));
            if (el && typeof v === 'number') {
                el.textContent = v > 0
                    ? v.toFixed(3) + (dir.kind === 'yaw' ? ' rad/s' : ' m/s')
                    : '—';
            }
        }
        const pwmEl = document.getElementById('c-ol-pwm');
        if (pwmEl && olc.basePwm > 0 && pwmEl.value === '100') {
            // Seed the PWM input from the stored cal so re-opening the
            // page shows what's currently live. Only overwrite the
            // default — respect whatever the user has typed in.
            pwmEl.value = olc.basePwm;
        }
    });
}

document.addEventListener('DOMContentLoaded', init);
