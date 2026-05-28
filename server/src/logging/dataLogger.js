// ============================================
// Data Logger
// ============================================
//
// Logs sensor data and pose estimates to CSV files
// for later analysis in your thesis.
//
// Features:
// - Event column: tags rows with calibration, command, and state-change events
// - JSON metadata sidecar: session info, config, firmware, events summary
// - Idle suppression: drops to 1 Hz when robot is stationary (configurable)
// - Missing sensor distinction: empty string for unavailable IMU vs 0 for real zeros
//

import { writeFileSync, mkdirSync, existsSync, createWriteStream } from 'fs';
import { config } from '../config.js';

export class DataLogger {
    constructor() {
        this.enabled = config.logging.enabled;
        this.directory = config.logging.directory;
        this.prefix = config.logging.prefix;

        this.filename = null;
        this.metaFilename = null;
        this.headerWritten = false;
        this.rowCount = 0;
        this.startTime = null;
        this.errorCount = 0;
        this.lastError = null;

        // Async append stream — opened on start(), closed on stop().
        // Using createWriteStream keeps the 50 Hz sensor loop non-blocking
        // (the previous appendFileSync blocked the event loop on every row).
        this._stream = null;

        // Event queue — written on next sensor row, then cleared
        this._pendingEvents = [];

        // Metadata sidecar
        this._meta = null;

        // Idle suppression
        this._idleSuppress = config.logging.idleSuppress ?? true;
        this._idleSince = null;        // timestamp when idle started
        this._idleThresholdMs = 2000;  // how long before suppression kicks in
        this._idleLogIntervalMs = 1000; // log at 1 Hz during idle
        this._lastIdleLogTime = 0;
        this._isIdle = false;

        // Track last command for change detection
        this._lastCmd = null;
    }

    // Start a new log file
    start(sessionInfo = {}) {
        if (!this.enabled) return;

        try {
            // Create directory if needed
            if (!existsSync(this.directory)) {
                mkdirSync(this.directory, { recursive: true });
            }

            // Generate filename with timestamp. Experiment runs pass an
            // explicit `prefix` in sessionInfo (e.g. exp_straight_2m_0.40_tier1_rep1_...)
            // so the file is self-describing without relying on the tier suffix.
            const now = new Date();
            const timestamp = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
            const tierSuffix = `tier${config.localizationTier}`;
            const prefixOverride = typeof sessionInfo.prefix === 'string' && sessionInfo.prefix.length > 0
                ? sessionInfo.prefix
                : null;
            const sessionName = prefixOverride ?? `${this.prefix}_${tierSuffix}_${timestamp}`;

            this.filename = prefixOverride
                ? `${this.directory}/${prefixOverride}.csv`
                : `${this.directory}/${sessionName}.csv`;
            this.metaFilename = this.filename.replace(/\.csv$/, '.meta.json');
            this.headerWritten = false;
            this.rowCount = 0;
            this.startTime = Date.now();
            this.errorCount = 0;
            this._pendingEvents = [];
            this._idleSince = null;
            this._isIdle = false;
            this._lastIdleLogTime = 0;
            this._lastCmd = null;

            // Open the async append stream. Truncate if a file exists so a
            // restart of the same session prefix overwrites cleanly (matches
            // the pre-stream writeFileSync behaviour for the header).
            this._stream = createWriteStream(this.filename, { flags: 'w' });
            this._stream.on('error', (err) => {
                this.errorCount++;
                this.lastError = err;
                if (this.errorCount === 1 || this.errorCount % 100 === 0) {
                    console.error(`Log stream error (${this.errorCount} total): ${err.message}`);
                }
            });

            // Write initial metadata sidecar. `prefix` is consumed for the
            // filename and not persisted into the meta body.
            const { prefix: _prefix, ...metaExtra } = sessionInfo;
            this._meta = {
                session: sessionName,
                startTime: now.toISOString(),
                tier: config.localizationTier,
                robotIp: config.robot.ip,
                motorGainsFwd: config.physical.motorGainsFwd,
                motorGainsRev: config.physical.motorGainsRev,
                imuWeight: config.fusion.imuWeight,
                speed: config.control.speed,
                idleSuppress: this._idleSuppress,
                events: [],
                ...metaExtra,
            };
            this._writeMetadata();

            console.log(`Logging to: ${this.filename}`);
        } catch (err) {
            console.error(`Failed to start logging: ${err.message}`);
            this.enabled = false;
            this.lastError = err;
        }
    }

    // Queue an event tag for the next logged row
    logEvent(tag) {
        if (!this.enabled || !this.filename) return;
        this._pendingEvents.push(tag);

        // Also record in metadata
        if (this._meta) {
            this._meta.events.push({
                time: new Date().toISOString(),
                elapsed_ms: Date.now() - this.startTime,
                row: this.rowCount,
                event: tag,
            });
        }
    }

    // Log a data row
    log(data) {
        if (!this.enabled || !this.filename) return;

        const {
            sensorData,
            pose,
            firmwarePose,
            command,
        } = data;

        // --- Idle suppression ---
        const velocities = sensorData?.velocities || [0, 0, 0, 0];
        const cmd = command || { vx: 0, vy: 0, omega: 0 };
        const hasMotion =
            velocities.some(v => Math.abs(v) > 0.05) ||
            Math.abs(cmd.vx) > 0 || Math.abs(cmd.vy) > 0 || Math.abs(cmd.omega) > 0;

        const now = Date.now();

        if (this._idleSuppress) {
            if (hasMotion || this._pendingEvents.length > 0) {
                // Motion detected or event pending — always log
                if (this._isIdle) {
                    this._isIdle = false;
                    this.logEvent('idle_end');
                }
                this._idleSince = null;
            } else {
                // No motion
                if (this._idleSince === null) {
                    this._idleSince = now;
                }

                if (now - this._idleSince >= this._idleThresholdMs) {
                    if (!this._isIdle) {
                        this._isIdle = true;
                        this.logEvent('idle_start');
                        // Force log this row (with the idle_start event)
                    } else {
                        // Already idle — throttle to 1 Hz
                        if (now - this._lastIdleLogTime < this._idleLogIntervalMs) {
                            return; // skip this row
                        }
                    }
                }
            }
        }

        this._lastIdleLogTime = now;

        // --- Command change detection ---
        if (this._lastCmd === null ||
            cmd.vx !== this._lastCmd.vx ||
            cmd.vy !== this._lastCmd.vy ||
            cmd.omega !== this._lastCmd.omega) {
            if (this._lastCmd !== null && (cmd.vx !== 0 || cmd.vy !== 0 || cmd.omega !== 0)) {
                this.logEvent(`cmd:vx=${cmd.vx.toFixed(2)},vy=${cmd.vy.toFixed(2)},w=${cmd.omega.toFixed(2)}`);
            }
            this._lastCmd = { ...cmd };
        }

        // --- Determine sensor availability ---
        const imuAvailable = sensorData?.imu &&
            (sensorData.imu.yaw !== 0 || sensorData.imu.pitch !== 0 || sensorData.imu.roll !== 0 ||
             sensorData.imu.gyroZ !== 0 || sensorData.imu.accelX !== 0 || sensorData.imu.accelY !== 0 ||
             (sensorData.calibration && sensorData.calibration.sys > 0));

        // Helpers: return value or empty string for missing
        const v = (val) => val ?? 0;
        const imu = (val) => imuAvailable ? (val ?? 0) : '';
        // PID debug columns: empty string when set_debug is off so the
        // CSV stays compact for runs that don't need the diagnostics.
        const pidDbg = (val) => (val === undefined || val === null) ? '' : val;

        // --- Build event string ---
        const eventStr = this._pendingEvents.length > 0
            ? this._pendingEvents.join(';')
            : '';
        this._pendingEvents = [];

        // Build row object
        const row = {
            // Timing
            timestamp: now,
            elapsed_ms: now - this.startTime,
            robot_timestamp: v(sensorData?.timestamp),

            // Encoder counts
            enc_L1: v(sensorData?.encoders?.[0]),
            enc_R1: v(sensorData?.encoders?.[1]),
            enc_R2: v(sensorData?.encoders?.[2]),
            enc_L2: v(sensorData?.encoders?.[3]),

            // Wheel velocities (rad/s)
            vel_L1: v(sensorData?.velocities?.[0]),
            vel_R1: v(sensorData?.velocities?.[1]),
            vel_R2: v(sensorData?.velocities?.[2]),
            vel_L2: v(sensorData?.velocities?.[3]),

            // IMU data (empty string when IMU unavailable)
            imu_yaw: imu(sensorData?.imu?.yaw),
            imu_pitch: imu(sensorData?.imu?.pitch),
            imu_roll: imu(sensorData?.imu?.roll),
            imu_gyro_z: imu(sensorData?.imu?.gyroZ),
            imu_accel_x: imu(sensorData?.imu?.accelX),
            imu_accel_y: imu(sensorData?.imu?.accelY),

            // Calibration
            cal_sys: v(sensorData?.calibration?.sys),
            cal_gyro: v(sensorData?.calibration?.gyro),
            cal_accel: v(sensorData?.calibration?.accel),
            cal_mag: v(sensorData?.calibration?.mag),

            // Pose estimate (server-computed)
            pose_x: v(pose?.x),
            pose_y: v(pose?.y),
            pose_theta: v(pose?.theta),
            pose_theta_deg: v(pose?.thetaDeg),
            pose_tier: v(pose?.tier),

            // Firmware-computed pose (ESP32 onboard odometry)
            fw_pose_available: firmwarePose ? 1 : 0,
            fw_pose_x: v(firmwarePose?.x),
            fw_pose_y: v(firmwarePose?.y),
            fw_pose_theta: v(firmwarePose?.theta),

            // Command
            cmd_vx: v(command?.vx),
            cmd_vy: v(command?.vy),
            cmd_omega: v(command?.omega),

            // PID debug per-wheel diagnostics (empty unless set_debug
            // was sent). Internal firmware order [L1, R1, L2, R2] —
            // NOT the external CSV wheel-velocity order [L1, R1, R2, L2]
            // so fields are explicitly labeled with the internal index
            // to keep the convention clear.
            dbg_tgt_i0: pidDbg(sensorData?.dbg?.pid?.[0]?.tgt),
            dbg_tgt_i1: pidDbg(sensorData?.dbg?.pid?.[1]?.tgt),
            dbg_tgt_i2: pidDbg(sensorData?.dbg?.pid?.[2]?.tgt),
            dbg_tgt_i3: pidDbg(sensorData?.dbg?.pid?.[3]?.tgt),
            dbg_act_i0: pidDbg(sensorData?.dbg?.pid?.[0]?.act),
            dbg_act_i1: pidDbg(sensorData?.dbg?.pid?.[1]?.act),
            dbg_act_i2: pidDbg(sensorData?.dbg?.pid?.[2]?.act),
            dbg_act_i3: pidDbg(sensorData?.dbg?.pid?.[3]?.act),
            dbg_err_i0: pidDbg(sensorData?.dbg?.pid?.[0]?.err),
            dbg_err_i1: pidDbg(sensorData?.dbg?.pid?.[1]?.err),
            dbg_err_i2: pidDbg(sensorData?.dbg?.pid?.[2]?.err),
            dbg_err_i3: pidDbg(sensorData?.dbg?.pid?.[3]?.err),
            dbg_p_i0:   pidDbg(sensorData?.dbg?.pid?.[0]?.p),
            dbg_p_i1:   pidDbg(sensorData?.dbg?.pid?.[1]?.p),
            dbg_p_i2:   pidDbg(sensorData?.dbg?.pid?.[2]?.p),
            dbg_p_i3:   pidDbg(sensorData?.dbg?.pid?.[3]?.p),
            dbg_i_i0:   pidDbg(sensorData?.dbg?.pid?.[0]?.i),
            dbg_i_i1:   pidDbg(sensorData?.dbg?.pid?.[1]?.i),
            dbg_i_i2:   pidDbg(sensorData?.dbg?.pid?.[2]?.i),
            dbg_i_i3:   pidDbg(sensorData?.dbg?.pid?.[3]?.i),
            dbg_d_i0:   pidDbg(sensorData?.dbg?.pid?.[0]?.d),
            dbg_d_i1:   pidDbg(sensorData?.dbg?.pid?.[1]?.d),
            dbg_d_i2:   pidDbg(sensorData?.dbg?.pid?.[2]?.d),
            dbg_d_i3:   pidDbg(sensorData?.dbg?.pid?.[3]?.d),
            dbg_ff_i0:  pidDbg(sensorData?.dbg?.pid?.[0]?.ff),
            dbg_ff_i1:  pidDbg(sensorData?.dbg?.pid?.[1]?.ff),
            dbg_ff_i2:  pidDbg(sensorData?.dbg?.pid?.[2]?.ff),
            dbg_ff_i3:  pidDbg(sensorData?.dbg?.pid?.[3]?.ff),
            dbg_pwm_i0: pidDbg(sensorData?.dbg?.pid?.[0]?.pwm),
            dbg_pwm_i1: pidDbg(sensorData?.dbg?.pid?.[1]?.pwm),
            dbg_pwm_i2: pidDbg(sensorData?.dbg?.pid?.[2]?.pwm),
            dbg_pwm_i3: pidDbg(sensorData?.dbg?.pid?.[3]?.pwm),

            // Event marker
            event: eventStr,
        };

        if (!this._stream) return;

        // Write header if first row
        if (!this.headerWritten) {
            const header = Object.keys(row).join(',');
            this._stream.write(header + '\n');
            this.headerWritten = true;
        }

        // Write data row
        const values = Object.values(row).map(val => {
            if (val === '') return '';
            if (typeof val === 'number') return val.toFixed(6);
            return val;
        }).join(',');

        this._stream.write(values + '\n');
        this.rowCount++;

        // Auto-disable after enough stream errors (tracked in the 'error'
        // handler attached in start()).
        if (this.errorCount >= 1000) {
            console.error('Too many logging errors, disabling logger');
            this.enabled = false;
        }
    }

    // Get current status
    getStatus() {
        return {
            enabled: this.enabled,
            filename: this.filename,
            rowCount: this.rowCount,
            elapsed: this.startTime ? Date.now() - this.startTime : 0,
            errorCount: this.errorCount,
        };
    }

    // Stop logging and finalize metadata
    stop() {
        if (this.filename) {
            console.log(`Logging stopped. ${this.rowCount} rows written to ${this.filename}`);

            // Finalize metadata
            if (this._meta) {
                this._meta.endTime = new Date().toISOString();
                this._meta.rowCount = this.rowCount;
                this._meta.duration_ms = Date.now() - this.startTime;
                this._meta.duration_s = Math.round((Date.now() - this.startTime) / 1000);
                this._meta.errorCount = this.errorCount;
                this._writeMetadata();
            }
        }
        if (this._stream) {
            this._stream.end();
            this._stream = null;
        }
        this.filename = null;
        this.metaFilename = null;
        this._meta = null;
    }

    // Update the experiment block in the metadata sidecar and flush to disk.
    // Called by TrajectoryRunner at each lifecycle transition (armed, completed,
    // ground_truth_submitted, aborted).
    updateExperimentMeta(experimentBlock) {
        if (!this._meta) return;
        this._meta.experiment = experimentBlock;
        this._writeMetadata();
    }

    // Mark the current session as tainted by the firmware IMU stuck-read
    // watchdog firing at least once. Session-level sticky flag so
    // downstream analysis can filter runs with compromised fw_pose / IMU
    // columns without reading the event stream. Writes the sidecar on
    // first transition only.
    markImuStuck() {
        if (!this._meta) return;
        if (this._meta.imu_stuck_during_session) return;
        this._meta.imu_stuck_during_session = true;
        this._meta.imu_stuck_first_at = new Date().toISOString();
        this._writeMetadata();
    }

    // Write metadata sidecar to disk
    _writeMetadata() {
        if (!this.metaFilename || !this._meta) return;
        try {
            writeFileSync(this.metaFilename, JSON.stringify(this._meta, null, 2) + '\n');
        } catch (err) {
            console.error(`Failed to write metadata: ${err.message}`);
        }
    }
}
