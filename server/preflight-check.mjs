import WebSocket from 'ws';

const URL = 'ws://localhost:3000/ws';
const WINDOW_MS = 3000;

const ws = new WebSocket(URL);
const states = [];
let info = null;
let infoRequested = false;

const pass = (msg) => console.log(`  ✓ ${msg}`);
const fail = (msg) => { console.log(`  ✗ ${msg}`); process.exitCode = 1; };
const warn = (msg) => console.log(`  ! ${msg}`);
const note = (msg) => console.log(`    ${msg}`);

ws.on('open', () => {
    console.log(`connected to ${URL}`);
    // Request NVS/firmware info
    ws.send(JSON.stringify({ type: 'get_info' }));
    infoRequested = true;

    setTimeout(() => {
        ws.close();
        report();
    }, WINDOW_MS);
});

ws.on('message', (raw) => {
    let m;
    try { m = JSON.parse(raw.toString()); } catch { return; }
    if (m.type === 'state') states.push(m);
    else if (m.type === 'robotInfo' || m.type === 'info') info = m;
});

ws.on('error', (err) => {
    console.log(`WS error: ${err.message}`);
    process.exit(2);
});

function report() {
    console.log(`captured ${states.length} state broadcasts in ${WINDOW_MS} ms\n`);

    if (states.length === 0) {
        fail('server not broadcasting state — is it running and connected to a robot?');
        process.exit(1);
    }

    const rate = (states.length / (WINDOW_MS / 1000)).toFixed(1);
    note(`broadcast rate: ${rate} Hz (expected ~10 Hz)`);

    const s = states[states.length - 1];
    const prev = states[0];

    console.log('\n── Server → Robot link ──');
    if (s.connected) pass(`robot connected @ ${s.robotIp}`);
    else fail(`robot NOT connected @ ${s.robotIp}`);

    console.log('\n── Sensor stream freshness ──');
    if (!s.sensors) {
        fail('no sensor data in state broadcast');
    } else {
        const ageMs = Date.now() - s.sensors.serverReceivedAt;
        if (ageMs < 500) pass(`last sensor packet ${ageMs} ms old (fresh)`);
        else fail(`last sensor packet ${ageMs} ms old (stale — robot may be disconnected)`);

        // Encoder signs of life — any wheel changed across the window?
        if (prev.sensors && s.sensors) {
            const deltas = s.sensors.enc.map((v, i) => v - prev.sensors.enc[i]);
            const anyChanged = deltas.some(d => d !== 0);
            if (anyChanged) note(`enc deltas [${deltas.join(', ')}] (wheels can move)`);
            else note(`enc deltas all zero — robot at rest`);
            note(`enc snapshot  [${s.sensors.enc.join(', ')}]`);
        }
    }

    console.log('\n── IMU (BNO055) ──');
    if (!s.sensors?.imu) {
        fail('no IMU block in sensors');
    } else {
        const imu = s.sensors.imu;
        const cal = s.sensors.cal;
        note(`yaw=${imu.yaw.toFixed(2)}°  gyroZ=${imu.gyroZ.toFixed(4)} rad/s  accel=(${imu.accelX.toFixed(3)}, ${imu.accelY.toFixed(3)})`);
        note(`cal  sys=${cal.sys}  gyro=${cal.gyro}  accel=${cal.accel}  mag=${cal.mag}  (0-3, higher=better)`);

        if (s.sensors.imuStuck === true) {
            fail('IMU STUCK flag set by firmware watchdog — power-cycle the robot');
        } else {
            pass('IMU watchdog flag clear (not stuck)');
        }

        if (cal.gyro >= 2 && cal.sys >= 1) pass(`IMU calibration acceptable (sys ${cal.sys}/3, gyro ${cal.gyro}/3)`);
        else warn(`IMU calibration low (sys ${cal.sys}/3, gyro ${cal.gyro}/3) — fig-8 warm-up recommended`);

        // Lockup detection: check yaw variation over the window
        const yaws = states.map(st => st.sensors?.imu?.yaw).filter(v => v !== undefined);
        const gzs = states.map(st => st.sensors?.imu?.gyroZ).filter(v => v !== undefined);
        const yawRange = yaws.length ? Math.max(...yaws) - Math.min(...yaws) : 0;
        const gzRange = gzs.length ? Math.max(...gzs) - Math.min(...gzs) : 0;
        if (yawRange === 0 && gzRange === 0 && yaws.length > 5) {
            warn(`yaw and gyroZ identical across ${yaws.length} frames — possible lockup (watchdog may fire soon)`);
        } else {
            note(`yaw range over window: ${yawRange.toFixed(3)}°,  gyroZ range: ${gzRange.toFixed(4)} rad/s  (non-zero = sensor live)`);
        }
    }

    console.log('\n── ESP32 heap ──');
    if (s.sensors?.heap) {
        const { free, min } = s.sensors.heap;
        note(`free=${(free / 1024).toFixed(1)} KB  low-water=${(min / 1024).toFixed(1)} KB`);
        if (min > 30 * 1024) pass('heap headroom healthy (low-water > 30 KB)');
        else if (min > 10 * 1024) warn(`heap low-water ${(min / 1024).toFixed(1)} KB — monitor during campaign`);
        else fail(`heap low-water ${(min / 1024).toFixed(1)} KB — risk of OOM crash`);
    } else {
        warn('no heap telemetry in state');
    }

    console.log('\n── Firmware info (get_info response) ──');
    if (!info) {
        warn('get_info response not received within window');
    } else {
        note(`firmware=${info.firmware}  uptime=${(info.uptime / 1000).toFixed(1)}s  rssi=${info.rssi} dBm  ntp=${info.ntpSynced ? 'synced' : 'NOT SYNCED'}`);

        // Motor gains (NVS)
        if (info.motorGainsFwd && info.motorGainsRev) {
            const gf = info.motorGainsFwd.map(Number);
            const gr = info.motorGainsRev.map(Number);
            note(`motor gains fwd [${gf.map(v => v.toFixed(3)).join(', ')}]`);
            note(`motor gains rev [${gr.map(v => v.toFixed(3)).join(', ')}]`);
            const all = [...gf, ...gr];
            const anyUnity = all.every(v => Math.abs(v - 1.0) < 0.001);
            if (anyUnity) fail('all motor gains are 1.000 — NVS not loaded or calibration not saved');
            else if (all.every(v => v > 0.7 && v < 1.3)) pass('motor gains within sane bounds (0.7–1.3)');
            else warn(`motor gain outside 0.7–1.3 range (suspicious): [${all.map(v => v.toFixed(3)).join(', ')}]`);
        } else {
            warn('motorGainsFwd/Rev missing from get_info');
        }

        // Open-loop (tier-0) cal
        if (info.openloopCal) {
            const olc = info.openloopCal;
            if (olc.valid) {
                pass(`tier-0 open-loop cal valid  (basePwm=${olc.basePwm})`);
                const sp = olc.speeds;
                note(`speeds  fwd=${sp.fwd.toFixed(3)}  back=${sp.back.toFixed(3)}  strafe_l=${sp.strafe_l.toFixed(3)}  strafe_r=${sp.strafe_r.toFixed(3)}  yaw_ccw=${sp.yaw_ccw.toFixed(3)}  yaw_cw=${sp.yaw_cw.toFixed(3)}`);
                note('handover baseline: fwd 0.497, back 0.504, strafe 0.373, yaw_ccw 2.02, yaw_cw 1.85');
            } else {
                fail('tier-0 open-loop cal INVALID — tier-0 trajectory runs will be rejected at arm time');
            }
        } else {
            warn('openloopCal block missing from get_info');
        }

        // Trajectory / experiment runner state
        if (info.trajState && info.trajState !== 'idle') {
            warn(`firmware trajectory state = '${info.trajState}' — expected 'idle'. Clear before arming.`);
        } else if (info.trajState === 'idle') {
            pass(`firmware trajectory state = idle (ready to arm)`);
        }

        if (info.imuAvailable === false) fail('firmware reports IMU not available');
    }

    console.log('\n── Experiment runner (server) ──');
    if (s.experiment) {
        note(`state=${s.experiment.state}  runPresent=${!!s.experiment.run}`);
        if (s.experiment.state !== 'idle' && !s.experiment.run?.aborted) {
            warn(`runner is in non-idle state — abort or complete before arming a fresh run`);
        } else {
            pass('runner state is idle');
        }
    }

    console.log('\ndone.');
}
