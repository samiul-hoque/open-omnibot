// ============================================
// Robot WebSocket Client
// ============================================

import WebSocket from 'ws';
import { EventEmitter } from 'events';
import { config } from '../config.js';

export class RobotClient extends EventEmitter {
    constructor() {
        super();
        this.ws = null;
        this.connected = false;
        this.reconnecting = false;
        this.reconnectAttempts = 0;
        this.lastSensorData = null;
        this.validationWarnings = 0;
        this.connectTimeout = null;
        this.pingInterval = null;
        this.pongReceived = true;
    }

    // Validate that a value is a finite number, return default if not
    validateNumber(value, defaultValue = 0, name = 'value') {
        const num = parseFloat(value);
        if (!Number.isFinite(num)) {
            const now = Date.now();
            if (now - (this._lastValidationWarnTime || 0) >= 10000) {
                console.warn(`Invalid ${name}: ${value}, using default ${defaultValue} (suppressing for 10s)`);
                this._lastValidationWarnTime = now;
            }
            return defaultValue;
        }
        return num;
    }

    // Validate encoder array
    validateEncoders(encoders) {
        if (!Array.isArray(encoders) || encoders.length !== 4) {
            if (this.validationWarnings < 10) {
                console.warn('Invalid encoder array, using zeros');
                this.validationWarnings++;
            }
            return [0, 0, 0, 0];
        }
        return encoders.map((v, i) => this.validateNumber(v, 0, `encoder[${i}]`));
    }

    // Validate velocities array
    validateVelocities(velocities) {
        if (!Array.isArray(velocities) || velocities.length !== 4) {
            return [0, 0, 0, 0];
        }
        return velocities.map((v, i) => this.validateNumber(v, 0, `velocity[${i}]`));
    }

    connect() {
        // Clean up any existing socket before reconnecting
        this.cleanup();

        const url = `ws://${config.robot.ip}:${config.robot.wsPort}${config.robot.wsPath}`;
        console.log(`Connecting to robot at ${url}...`);

        this.ws = new WebSocket(url, {
            perMessageDeflate: false,
            handshakeTimeout: 5000,
        });

        // Connection timeout — if open doesn't fire within 5s, give up and retry
        this.connectTimeout = setTimeout(() => {
            if (!this.connected) {
                console.error('Connection timeout — robot not reachable');
                this.ws.terminate();
            }
        }, 5000);

        this.ws.on('open', () => {
            clearTimeout(this.connectTimeout);
            this.connectTimeout = null;
            console.log('Connected to robot!');
            this.connected = true;
            this.reconnecting = false;
            this.reconnectAttempts = 0;
            this.pongReceived = true;
            this.emit('connected');
            this.startPingInterval();
        });

        this.ws.on('message', (data) => {
            try {
                const msg = JSON.parse(data.toString());
                this.handleMessage(msg);
            } catch (e) {
                console.error('Failed to parse message:', e);
            }
        });

        this.ws.on('close', () => {
            const wasConnected = this.connected;
            this.cleanup();
            if (wasConnected) {
                console.log('Disconnected from robot');
                this.emit('disconnected');
            }
            this.scheduleReconnect();
        });

        this.ws.on('error', (err) => {
            // Only log if it's not the expected ECONNREFUSED during reconnect
            if (err.code !== 'ECONNREFUSED') {
                console.error('WebSocket error:', err.message);
            }
        });
    }

    cleanup() {
        clearTimeout(this.connectTimeout);
        this.connectTimeout = null;
        clearInterval(this.pingInterval);
        this.pingInterval = null;
        this.connected = false;
        this.reconnecting = false;

        if (this.ws) {
            // Remove all listeners to prevent double-fire
            this.ws.removeAllListeners();
            try { this.ws.terminate(); } catch { /* socket already closed */ }
            this.ws = null;
        }
    }

    startPingInterval() {
        // Send app-level ping every 5s to detect dead connections
        this.pingInterval = setInterval(() => {
            if (!this.connected) {
                clearInterval(this.pingInterval);
                return;
            }
            if (!this.pongReceived) {
                console.error('Robot ping timeout — no pong received, reconnecting');
                this.cleanup();
                this.emit('disconnected');
                this.scheduleReconnect();
                return;
            }
            this.pongReceived = false;
            this.ping();
        }, 5000);
    }

    handleMessage(msg) {
        switch (msg.type) {
        case 'sensors': {
            // Fast-path sensor parsing: firmware data is trusted (validated
            // at source by snprintf formatting). Use direct access with
            // fallbacks instead of 22x validateNumber() calls.
            const imu = msg.imu || {};
            const enc = msg.enc;
            const vel = msg.vel;
            const now = Date.now();
            this.lastSensorData = {
                timestamp: msg.t || now,
                encoders: Array.isArray(enc) && enc.length === 4 ? enc : [0, 0, 0, 0],
                velocities: Array.isArray(vel) && vel.length === 4 ? vel : [0, 0, 0, 0],
                // Sensor-value defaults: use `??` (nullish coalesce) rather
                // than `||`, otherwise a legitimate exact-0 reading —
                // common for yaw during a held-heading strafe, or for
                // gyroZ at rest — collapses into the "missing field"
                // sentinel and becomes indistinguishable from "the
                // firmware didn't include this field at all". Downstream
                // odometry integrates these directly; preserving
                // exact-zero preserves physical signal.
                imu: {
                    yaw: imu.yaw ?? 0,
                    pitch: imu.pitch ?? 0,
                    roll: imu.roll ?? 0,
                    gyroZ: imu.gz ?? 0,
                    accelX: imu.ax ?? 0,
                    accelY: imu.ay ?? 0,
                },
                calibration: msg.cal || { sys: 0, gyro: 0, accel: 0, mag: 0 },
                robotUtc: msg.utc || null,
                receivedAt: now,
                firmwarePose: msg.pose ? {
                    x: msg.pose.x ?? 0,
                    y: msg.pose.y ?? 0,
                    theta: msg.pose.th ?? 0,
                } : null,
                // Firmware's IMU stuck-read watchdog — latched true when
                // BNO055 returns bit-identical gyro_z for ~1 s. Sticky
                // until the robot is power-cycled.
                imuStuck: msg.imuStuck === true,
                // ESP32 heap telemetry — `free` is current free bytes,
                // `min` is the low-water mark since boot. Long
                // downward drift of min = slow leak; short dip = a
                // transient allocation (e.g. large traj upload).
                heap: msg.heap ? {
                    free: Number(msg.heap.free) || 0,
                    min: Number(msg.heap.min) || 0,
                } : null,
                // PID debug block — present only when `set_debug` has
                // been sent. Contains per-wheel target / actual / error
                // / P / I / D / feedforward / pwm plus firmware FK
                // body-frame velocity. Logged to CSV for offline
                // steady-state bias analysis.
                dbg: msg.dbg || null,
            };
            this.emit('sensors', this.lastSensorData);
            break;
        }

        case 'ack':
            // Emit the full message so listeners (e.g. heading-hold UI)
            // can read extra fields like `enabled`, `success`, `gainsFwd`.
            // No existing listener depends on the old string-only signature.
            this.emit('ack', msg);
            break;

        case 'log':
            this.emit('log', msg.msg);
            break;

        case 'ping':
            this.ws.send(JSON.stringify({ type: 'pong' }));
            break;

        case 'pong':
            this.pongReceived = true;
            this.emit('pong');
            break;

        case 'info':
            this.emit('info', msg);
            break;

        case 'pong_cal':
            this.emit('pong_cal', msg);
            break;

        case 'motor_cal_result':
            this.emit('motor_cal_result', msg);
            break;

        case 'traj_done':
            this.emit('traj_done', msg);
            break;

        case 'traj_paused':
            this.emit('traj_paused', msg);
            break;

        case 'traj_progress':
            this.emit('traj_progress', msg);
            break;

        case 'self_test_result':
            this.emit('self_test_result', msg);
            break;

        case 'self_test_progress':
            this.emit('self_test_progress', msg);
            break;

        case 'self_test_complete':
            this.emit('self_test_complete', msg);
            break;

        default:
            console.log('Unknown message type:', msg.type);
        }
    }

    scheduleReconnect() {
        if (this.reconnecting) return;
        this.reconnecting = true;

        // Exponential backoff: 3s, 6s, 12s, ... capped at 30s
        const delay = Math.min(
            config.robot.reconnectInterval * Math.pow(2, this.reconnectAttempts),
            30000,
        );
        this.reconnectAttempts++;
        console.log(`Reconnecting in ${(delay / 1000).toFixed(1)}s (attempt ${this.reconnectAttempts})...`);
        setTimeout(() => {
            this.connect();
        }, delay);
    }

    safeSend(data) {
        if (!this.connected || !this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
        try {
            this.ws.send(JSON.stringify(data));
            return true;
        } catch (err) {
            console.error('Send failed:', err.message);
            return false;
        }
    }

    // Send velocity command (standard robotics convention, matches firmware)
    // vx: forward(+), vy: left(+), omega: CCW(+)
    setVelocity(vx, vy, omega) {
        return this.safeSend({ type: 'cmd', vx, vy, w: omega });
    }

    // Emergency stop
    stop() {
        return this.safeSend({ type: 'stop' });
    }

    // Reset encoders
    resetEncoders() {
        return this.safeSend({ type: 'reset_encoders' });
    }

    // Zero IMU orientation (set current heading as reference)
    zeroImu() {
        return this.safeSend({ type: 'zero_imu' });
    }

    // Direct motor test (calibration mode)
    motorTest(motor, pwm) {
        return this.safeSend({ type: 'motor_test', motor, pwm });
    }

    // Toggle IMU heading-hold. When enabled, firmware cancels yaw drift
    // during pure translation via a P-controller on gyro_z. Gain, deadzone,
    // and LPF alpha are optional — omitting them leaves the firmware's
    // current tuning untouched.
    setHeadingHold(enabled, gain, deadzone, alpha) {
        const msg = { type: 'set_heading_hold', enabled: !!enabled };
        if (typeof gain === 'number') msg.gain = gain;
        if (typeof deadzone === 'number') msg.deadzone = deadzone;
        if (typeof alpha === 'number') msg.alpha = alpha;
        return this.safeSend(msg);
    }

    // Request robot system info
    getInfo() {
        return this.safeSend({ type: 'get_info' });
    }

    // Save IMU calibration offsets to NVS
    saveImuCal() {
        return this.safeSend({ type: 'save_imu_cal' });
    }

    // Load IMU calibration offsets from NVS
    loadImuCal() {
        return this.safeSend({ type: 'load_imu_cal' });
    }

    // Start automated motor calibration
    startMotorCal() {
        return this.safeSend({ type: 'start_motor_cal' });
    }

    // Save motor calibration gains to NVS
    saveMotorCal() {
        return this.safeSend({ type: 'save_motor_cal' });
    }

    // Load motor calibration gains from NVS
    loadMotorCal() {
        return this.safeSend({ type: 'load_motor_cal' });
    }

    // Manually set motor calibration gains (per-direction)
    setMotorGains(gainsFwd, gainsRev) {
        return this.safeSend({ type: 'set_motor_gains', gainsFwd, gainsRev });
    }

    // Calibration ping for roundtrip latency measurement
    // ts = browser timestamp, tsServerFwd = server timestamp when forwarding
    pingCal(ts, tsServerFwd) {
        return this.safeSend({ type: 'ping_cal', ts, ts_server_fwd: tsServerFwd });
    }

    // Ping to check connection
    ping() {
        return this.safeSend({ type: 'ping' });
    }

    // --- Trajectory commands (Phase 2 refactor) ---

    // Send trajectory definition to firmware for autonomous execution.
    // Pause markers (`{kind: 'pause'}`) embed waypoints in the segment
    // list — firmware halts on each and holds position until trajResume().
    //
    // `mode` selects the firmware execution path:
    //   - undefined / 'closedloop': normal PID + IK path (default)
    //   - 'openloop':               tier-0 open-loop executor that uses
    //                               an NVS-calibrated 6-direction PWM
    //                               table. Required when the server's
    //                               localizationTier is 0.
    loadTrajectory(runId, segments, mode) {
        const wireSegs = segments.map(s => {
            switch (s.kind) {
            case 'translate': return { k: 't', vx: s.vx, vy: s.vy, d: s.distance };
            case 'yaw':       return { k: 'y', w: s.w, a: s.angle };
            case 'strafe_circle': return { k: 'c', s: s.speed, r: s.radius };
            case 'pause':     return { k: 'p' };
            default:
                console.warn(`Unknown segment kind: ${s.kind}`);
                return { k: '?' };
            }
        });
        const msg = { type: 'load_trajectory', runId, segments: wireSegs };
        if (mode === 'openloop') msg.mode = 'openloop';
        return this.safeSend(msg);
    }

    // Start a loaded & armed trajectory
    trajStart() {
        return this.safeSend({ type: 'traj_start' });
    }

    // Resume a trajectory halted at a SEG_PAUSE waypoint
    trajResume() {
        return this.safeSend({ type: 'traj_resume' });
    }

    // Abort a running trajectory
    trajAbort() {
        return this.safeSend({ type: 'traj_abort' });
    }

    // Reset firmware odometry to given pose (defaults to origin)
    resetOdom(x = 0, y = 0, theta = 0) {
        return this.safeSend({ type: 'reset_odom', x, y, theta });
    }

    // Set firmware odometry config (IMU weight)
    setOdomConfig({ imuWeight } = {}) {
        const msg = { type: 'set_odom_config' };
        if (typeof imuWeight === 'number') msg.imuWeight = imuWeight;
        return this.safeSend(msg);
    }

    // --- Self-test commands (Phase 4) ---

    runSelfTest(tests) {
        const msg = { type: 'start_self_test' };
        if (Array.isArray(tests)) msg.tests = tests;
        return this.safeSend(msg);
    }

    abortSelfTest() {
        return this.safeSend({ type: 'abort_self_test' });
    }

    // Get latest sensor data
    getSensors() {
        return this.lastSensorData;
    }

    // Check if connected
    isConnected() {
        return this.connected;
    }

    // Close connection
    disconnect() {
        this.cleanup();
    }
}
