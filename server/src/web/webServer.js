// ============================================
// Web UI Server
// ============================================
//
// HTTP static file server + WebSocket server for browser dashboard.
// Broadcasts sensor data and pose to connected browsers.
// Receives commands from browser and forwards to robot.
//

import http from 'http';
import fs from 'fs';
import fsp from 'fs/promises';
import path from 'path';
import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
import { WebSocketServer } from 'ws';
import { config } from '../config.js';
import { listTrajectories } from '../experiments/trajectories.js';
import { getManifest, readDoc, readSourceSlice, readImage } from './docsServer.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const PUBLIC_DIR = path.join(__dirname, 'public');

// Repo-root paths for the GT camera calibration tooling. webServer.js lives
// in server/src/web/, so the repo root is three levels up.
const REPO_ROOT = path.resolve(__dirname, '../../..');
const GT_PY = path.join(REPO_ROOT, 'evaluation/.venv/bin/python');
const GT_CALIBRATE = path.join(REPO_ROOT, 'evaluation/ground_truth/calibrate_homography.py');
const GT_CAL_JSON = path.join(REPO_ROOT, 'evaluation/ground_truth/calibration.json');
// Hardcode IPv4 loopback: Node's DNS resolver hands back ::1 first on some
// hosts, but the Python snapshot service binds 127.0.0.1 only, so "localhost"
// would ECONNREFUSED on IPv6-first systems. Matches the Python-side bind.
const GT_SNAPSHOT_URL = 'http://127.0.0.1:5055';

// MIME types for static files
const MIME_TYPES = {
    '.html': 'text/html',
    '.css': 'text/css',
    '.js': 'application/javascript',
    '.json': 'application/json',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
};

export class WebServer {
    constructor(dependencies = {}) {
        this.robot = dependencies.robot || null;
        this.localization = dependencies.localization || null;
        this.logger = dependencies.logger || null;
        this.getCommand = dependencies.getCommand || (() => ({ vx: 0, vy: 0, omega: 0 }));
        this.setCommand = dependencies.setCommand || (() => {});
        this.setTier = dependencies.setTier || (() => {});
        this.setImuWeight = dependencies.setImuWeight || (() => {});
        this.experimentRunner = dependencies.experimentRunner || null;

        this.server = null;
        this.wss = null;
        this.clients = new Set();
        this.broadcastInterval = null;
        this.heartbeatInterval = null;
        this.lastBroadcast = 0;
        // Explicit defaults for the two in-flight guards used by the
        // async experiment handlers. Reading `undefined` as falsy worked
        // by accident before; explicit init makes the invariant
        // (`false when no request is being processed`) readable at a
        // glance and immune to future linting rules that reject
        // implicit boolean coercion.
        this._snapshotInFlight = false;
        this._experimentStartInFlight = false;

        // Fan-out experiment runner events to all connected browsers.
        if (this.experimentRunner) {
            this.experimentRunner.on('armed', (e) => this._broadcast({ type: 'experiment_armed', ...e }));
            this.experimentRunner.on('started', (e) => this._broadcast({ type: 'experiment_started', ...e }));
            this.experimentRunner.on('tick', (e) => this._broadcast({ type: 'experiment_tick', ...e }));
            this.experimentRunner.on('paused', (e) => this._broadcast({ type: 'experiment_paused', ...e }));
            this.experimentRunner.on('completed', (e) => this._broadcast({ type: 'experiment_completed', ...e }));
            this.experimentRunner.on('aborted', (e) => this._broadcast({ type: 'experiment_aborted', ...e }));
            this.experimentRunner.on('stateChange', (e) => this._broadcast({ type: 'experiment_state', ...e }));
            this.experimentRunner.on('preflight_ok', (e) => this._broadcast({ type: 'experiment_preflight_ok', ...e }));
            this.experimentRunner.on('preflight_failed', (e) => this._broadcast({ type: 'experiment_preflight_failed', ...e }));
        }
    }

    _broadcast(payload) {
        const msg = JSON.stringify(payload);
        for (const client of this.clients) {
            if (client.readyState === 1) client.send(msg);
        }
    }

    start() {
        if (!config.web.enabled) {
            console.log('Web UI disabled in config');
            return;
        }

        // Create HTTP server for static files
        this.server = http.createServer((req, res) => {
            this.handleHttpRequest(req, res);
        });

        // Create WebSocket server
        this.wss = new WebSocketServer({
            server: this.server,
            path: '/ws',
        });

        this.wss.on('connection', (ws) => {
            this.handleWebSocketConnection(ws);
        });

        // Start server
        this.server.listen(config.web.port, () => {
            console.log(`Web UI available at http://localhost:${config.web.port}`);
        });

        // Start sensor broadcast interval
        this.broadcastInterval = setInterval(() => {
            this.broadcastState();
        }, config.web.broadcastInterval);

        // Start heartbeat interval (every 10 seconds)
        this.heartbeatInterval = setInterval(() => {
            this.checkHeartbeats();
        }, 10000);
    }

    checkHeartbeats() {
        for (const client of this.clients) {
            if (client.isAlive === false) {
                // Client didn't respond to last ping, terminate
                console.log('Browser client heartbeat timeout, disconnecting');
                client.terminate();
                this.clients.delete(client);
                continue;
            }

            // Mark as not alive, will be set to true when pong received
            client.isAlive = false;
            if (client.readyState === 1) {
                client.ping();
            }
        }
    }

    stop() {
        if (this.broadcastInterval) {
            clearInterval(this.broadcastInterval);
            this.broadcastInterval = null;
        }

        if (this.heartbeatInterval) {
            clearInterval(this.heartbeatInterval);
            this.heartbeatInterval = null;
        }

        // Close all WebSocket connections
        for (const client of this.clients) {
            client.close();
        }
        this.clients.clear();

        if (this.wss) {
            this.wss.close();
            this.wss = null;
        }

        if (this.server) {
            this.server.close();
            this.server = null;
        }
    }

    handleHttpRequest(req, res) {
        // Parse URL and normalize path
        let urlPath = req.url.split('?')[0];

        // Docs API routes — wired before static fallback.
        if (urlPath.startsWith('/api/docs/')) {
            this.handleDocsApi(req, res, urlPath);
            return;
        }

        // GT snapshot PNGs — files live at evaluation/snapshots/** outside
        // PUBLIC_DIR, so serve them here. Only .png under that root is
        // allowed; the ?path= value is normalised to reject traversal.
        if (urlPath === '/api/snapshot-image') {
            this._serveSnapshotImage(req, res);
            return;
        }

        if (urlPath === '/') {
            urlPath = '/index.html';
        }

        // Security: prevent directory traversal
        const safePath = path.normalize(urlPath).replace(/^(\.\.(\/|\\|$))+/, '');
        const filePath = path.join(PUBLIC_DIR, safePath);

        // Check if file exists and is within PUBLIC_DIR
        if (!filePath.startsWith(PUBLIC_DIR)) {
            res.writeHead(403);
            res.end('Forbidden');
            return;
        }

        fs.readFile(filePath, (err, data) => {
            if (err) {
                if (err.code === 'ENOENT') {
                    res.writeHead(404);
                    res.end('Not Found');
                } else {
                    res.writeHead(500);
                    res.end('Internal Server Error');
                }
                return;
            }

            const ext = path.extname(filePath);
            const mimeType = MIME_TYPES[ext] || 'application/octet-stream';

            res.writeHead(200, { 'Content-Type': mimeType });
            res.end(data);
        });
    }

    _serveSnapshotImage(req, res) {
        const url = new URL(req.url, 'http://localhost');
        const rel = url.searchParams.get('path') || '';
        // Only permit paths that look like `evaluation/snapshots/**/*.png` and
        // resolve inside the snapshots root after normalisation. No traversal.
        const snapshotsRoot = path.join(REPO_ROOT, 'evaluation', 'snapshots');
        const candidate = path.resolve(REPO_ROOT, rel);
        if (!candidate.startsWith(snapshotsRoot + path.sep) || !candidate.endsWith('.png')) {
            res.writeHead(403);
            res.end('Forbidden');
            return;
        }
        fs.readFile(candidate, (err, data) => {
            if (err) {
                res.writeHead(err.code === 'ENOENT' ? 404 : 500);
                res.end(err.code === 'ENOENT' ? 'Not Found' : 'Internal Server Error');
                return;
            }
            res.writeHead(200, {
                'Content-Type': 'image/png',
                'Content-Length': data.length,
                'Cache-Control': 'no-store',
            });
            res.end(data);
        });
    }

    handleDocsApi(req, res, urlPath) {
        const url = new URL(req.url, 'http://localhost');
        try {
            if (urlPath === '/api/docs/manifest') {
                const manifest = getManifest();
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify(manifest));
                return;
            }
            if (urlPath === '/api/docs/file') {
                const p = url.searchParams.get('path');
                const text = readDoc(p);
                res.writeHead(200, { 'Content-Type': 'text/markdown; charset=utf-8' });
                res.end(text);
                return;
            }
            if (urlPath === '/api/docs/source') {
                const p = url.searchParams.get('path');
                const start = url.searchParams.has('start') ? Number(url.searchParams.get('start')) : undefined;
                const end = url.searchParams.has('end') ? Number(url.searchParams.get('end')) : undefined;
                const slice = readSourceSlice(p, start, end);
                res.writeHead(200, { 'Content-Type': 'application/json' });
                res.end(JSON.stringify(slice));
                return;
            }
            if (urlPath === '/api/docs/image') {
                const p = url.searchParams.get('path');
                const img = readImage(p);
                res.writeHead(200, { 'Content-Type': img.mime, 'Cache-Control': 'public, max-age=3600' });
                res.end(img.data);
                return;
            }
            res.writeHead(404, { 'Content-Type': 'text/plain' });
            res.end('Not Found');
        } catch (err) {
            const msg = err && err.message ? err.message : 'error';
            const status = /not found|outside allowed|not markdown|extension not allowed/i.test(msg) ? 404 : 400;
            res.writeHead(status, { 'Content-Type': 'text/plain' });
            res.end(msg);
        }
    }

    handleWebSocketConnection(ws) {
        console.log('Browser connected to Web UI');
        this.clients.add(ws);

        // Mark client as alive for heartbeat
        ws.isAlive = true;

        // Handle pong responses for heartbeat
        ws.on('pong', () => {
            ws.isAlive = true;
        });

        // Send initial state + trajectory catalog. The catalog is one-shot
        // per connection (trajectories don't change at runtime) so we avoid
        // repeating it on the 10 Hz state broadcast.
        this.sendState(ws);
        if (ws.readyState === 1) {
            ws.send(JSON.stringify({
                type: 'trajectory_catalog',
                trajectories: listTrajectories(),
            }));
        }

        ws.on('message', (data) => {
            try {
                const msg = JSON.parse(data.toString());
                this.handleBrowserMessage(ws, msg);
            } catch (e) {
                console.error('Invalid message from browser:', e.message);
            }
        });

        ws.on('close', () => {
            console.log('Browser disconnected from Web UI');
            this.clients.delete(ws);
        });

        ws.on('error', (err) => {
            console.error('WebSocket error:', err.message);
            this.clients.delete(ws);
        });
    }

    handleBrowserMessage(ws, msg) {
        switch (msg.type) {
        case 'cmd':
            // Forward velocity command to robot
            if (this.robot && this.robot.isConnected()) {
                const vx = Number(msg.vx) || 0;
                const vy = Number(msg.vy) || 0;
                const w = Number(msg.w) || 0;
                this.robot.setVelocity(vx, vy, w);
                this.setCommand({ vx, vy, omega: w });
            }
            break;

        case 'stop':
            if (this.robot) {
                this.robot.stop();
                this.setCommand({ vx: 0, vy: 0, omega: 0 });
            }
            this._lastMotorTest = null;
            if (this.logger) this.logger.logEvent('stop');
            break;

        case 'resetPose':
            if (this.robot) {
                this.robot.resetEncoders();
            }
            if (this.localization) {
                this.localization.reset();
            }
            break;

        case 'zero_imu':
            if (this.robot && this.robot.isConnected()) {
                this.robot.zeroImu();
                // Reset IMU offset in fusion layer since yaw reference changed
                if (this.localization && typeof this.localization.resetImuOffset === 'function') {
                    this.localization.resetImuOffset();
                }
                if (this.logger) this.logger.logEvent('zero_imu');
            }
            break;

        case 'motor_test':
            if (this.robot && this.robot.isConnected()) {
                const motor = Number(msg.motor) || 0;
                const pwm = Number(msg.pwm) || 0;
                this.robot.motorTest(motor, pwm);
                // Only log on first occurrence or when motor/pwm changes
                const mtKey = `${motor}:${pwm}`;
                if (this.logger && this._lastMotorTest !== mtKey) {
                    this._lastMotorTest = mtKey;
                    this.logger.logEvent(`motor_test:m=${motor},pwm=${pwm}`);
                }
            }
            break;

        case 'get_info':
            if (this.robot && this.robot.isConnected()) {
                this.robot.getInfo();
            }
            break;

        case 'save_imu_cal':
            if (this.robot && this.robot.isConnected()) {
                this.robot.saveImuCal();
                if (this.logger) this.logger.logEvent('imu_cal_save');
            }
            break;

        case 'load_imu_cal':
            if (this.robot && this.robot.isConnected()) {
                this.robot.loadImuCal();
                if (this.logger) this.logger.logEvent('imu_cal_load');
            }
            break;

        case 'ping_cal':
            if (this.robot && this.robot.isConnected() && Number.isFinite(msg.ts)) {
                this.robot.pingCal(msg.ts, Date.now());
            }
            break;

        case 'start_motor_cal':
            if (this.robot && this.robot.isConnected()) {
                this.robot.startMotorCal();
                if (this.logger) this.logger.logEvent('motor_cal_start');
            }
            break;

        case 'save_motor_cal':
            if (this.robot && this.robot.isConnected()) {
                this.robot.saveMotorCal();
                if (this.logger) this.logger.logEvent('motor_cal_save');
            }
            break;

        case 'load_motor_cal':
            if (this.robot && this.robot.isConnected()) {
                this.robot.loadMotorCal();
                if (this.logger) this.logger.logEvent('motor_cal_load');
            }
            break;

        case 'set_motor_gains':
            if (this.robot && this.robot.isConnected()) {
                this.robot.setMotorGains(msg.gainsFwd || msg.gains, msg.gainsRev || msg.gains);
                if (this.logger) this.logger.logEvent('motor_gains_set');
            }
            break;

        case 'set_openloop_cal':
            // Tier-0 open-loop calibration from the dashboard. Forwards
            // { basePwm, speeds: {fwd, back, strafe_l, strafe_r,
            //   yaw_ccw, yaw_cw} } to firmware's NVS writer. Firmware
            // validates ranges + rejects partial cal; we just pass
            // through. Ack (ok:true|false) is broadcast from firmware
            // so every dashboard sees the state change.
            if (this.robot && this.robot.isConnected()) {
                this.robot.safeSend({
                    type: 'set_openloop_cal',
                    basePwm: msg.basePwm,
                    speeds: msg.speeds,
                });
                if (this.logger) this.logger.logEvent('openloop_cal_set');
            }
            break;

        case 'set_heading_hold':
            if (this.robot && this.robot.isConnected()) {
                this.robot.setHeadingHold(msg.enabled, msg.gain, msg.deadzone, msg.alpha);
                if (this.logger) this.logger.logEvent(`heading_hold:${msg.enabled ? 'on' : 'off'}`);
            }
            break;

        case 'set_debug':
            // Forward to robot; triggers per-wheel PID diagnostic block on
            // every sensor broadcast. Used by smoketest_straight_debug.mjs
            // to log integrator/FF internals for steady-state bias hunts.
            if (this.robot && this.robot.isConnected()) {
                this.robot.safeSend({
                    type: 'set_debug',
                    enabled: msg.enabled === true,
                    rate_divider: Number(msg.rate_divider) || 1,
                });
                if (this.logger) this.logger.logEvent(`debug:${msg.enabled ? 'on' : 'off'}`);
            }
            break;

        case 'setTier': {
            const tier = Number(msg.tier);
            // Tier 0 (open-loop baseline) is a valid selection. The
            // earlier bound `tier >= 1 && tier <= 2` silently dropped
            // dashboard tier-0 selections — exactly the silent-skip
            // failure mode called out in feedback_preflight_bug_check_storage.
            if (tier === 0 || tier === 1 || tier === 2) {
                this.setTier(tier);
            } else {
                console.warn(`Rejected setTier: invalid tier ${msg.tier}`);
            }
            break;
        }

        case 'setImuWeight': {
            const weight = Number(msg.weight);
            if (weight >= 0 && weight <= 1) {
                this.setImuWeight(weight);
            }
            break;
        }

        case 'listPorts':
            this.handleListPorts(ws);
            break;

        case 'getRobotIp':
            this.handleGetRobotIp(ws);
            break;

        case 'setRobotIp':
            this.handleSetRobotIp(ws, msg.ip);
            break;

        case 'experiment_arm':
            this._handleExperimentArm(ws, msg);
            break;

        case 'experiment_start':
            this._handleExperimentStart(ws);
            break;

        case 'experiment_abort':
            this._handleExperimentAction(ws, 'abort', msg.reason || 'manual');
            break;

        case 'experiment_ground_truth':
            this._handleExperimentGroundTruth(ws, msg);
            break;

        // --- Firmware odometry / trajectory proxies ---
        case 'reset_odom':
            if (this.robot) this.robot.resetOdom(msg.x, msg.y, msg.theta);
            break;

        case 'set_odom_config':
            if (this.robot) this.robot.setOdomConfig({ imuWeight: msg.imuWeight });
            break;

        case 'start_self_test':
            if (this.robot) this.robot.runSelfTest(msg.tests);
            break;

        case 'abort_self_test':
            if (this.robot) this.robot.abortSelfTest();
            break;

        case 'get_camera_calibration':
            this._sendCameraCalibration(ws);
            break;

        case 'calibrate_camera':
            this._handleCalibrateCamera(ws);
            break;

        case 'experiment_capture_snapshot':
            this._handleExperimentCaptureSnapshot(ws, msg);
            break;

        case 'cal_snapshot':
            this._handleCalSnapshot(ws, msg);
            break;

        default:
            console.log('Unknown browser message type:', msg.type);
        }
    }

    async _sendCameraCalibration(ws) {
        try {
            const data = await fsp.readFile(GT_CAL_JSON, 'utf-8');
            const cal = JSON.parse(data);
            if (ws && ws.readyState === 1) {
                ws.send(JSON.stringify({
                    type: 'camera_calibration',
                    calibratedAt: cal.calibrated_at || null,
                    p50mm: cal.reproj_error_p50_mm ?? null,
                    p95mm: cal.reproj_error_p95_mm ?? null,
                    inliers: cal.inliers ?? null,
                }));
            }
        } catch (err) {
            if (ws && ws.readyState === 1) {
                ws.send(JSON.stringify({
                    type: 'camera_calibration',
                    error: `could not read calibration.json: ${err.message}`,
                }));
            }
        }
    }

    // Spawn calibrate_homography.py, parse metrics, optionally nudge the
    // snapshot service to reload, broadcast the result. Guarded by a single
    // in-flight flag because the calibration script opens the RTSP stream
    // exclusively — two concurrent runs would race on the camera.
    async _handleCalibrateCamera(_ws) {
        if (this._cameraCalInFlight) {
            this._broadcast({
                type: 'camera_calibration_result',
                ok: false,
                error: 'calibration already running',
            });
            return;
        }
        this._cameraCalInFlight = true;
        console.log('[gt-cal] spawning calibrate_homography.py');
        let stdout = '';
        let stderr = '';
        const child = spawn(GT_PY, [GT_CALIBRATE, '--save-visual'], {
            cwd: REPO_ROOT,
        });
        // Safety timeout: the script normally completes in ~5s. If it hangs
        // (e.g. the camera stream wedges), kill the child so the in-flight
        // flag clears and the UI button re-enables.
        const killTimer = setTimeout(() => {
            console.warn('[gt-cal] subprocess exceeded 60s — sending SIGKILL');
            stderr += '\n[server] timed out after 60s, killed\n';
            child.kill('SIGKILL');
        }, 60_000);
        child.stdout.on('data', (d) => { stdout += d.toString(); });
        child.stderr.on('data', (d) => { stderr += d.toString(); });
        child.on('close', async (code) => {
            clearTimeout(killTimer);
            this._cameraCalInFlight = false;
            const ok = code === 0;
            let metrics = {};
            if (ok) {
                try {
                    const data = await fsp.readFile(GT_CAL_JSON, 'utf-8');
                    const cal = JSON.parse(data);
                    metrics = {
                        calibratedAt: cal.calibrated_at || null,
                        p50mm: cal.reproj_error_p50_mm ?? null,
                        p95mm: cal.reproj_error_p95_mm ?? null,
                        inliers: cal.inliers ?? null,
                    };
                    // Best-effort: tell snapshot service to reload the new homography.
                    this._postSnapshotReload().catch(() => {});
                } catch (err) {
                    stderr += `\n[server] failed to parse calibration.json: ${err.message}`;
                }
            }
            console.log(`[gt-cal] done (exit ${code})`);
            this._broadcast({
                type: 'camera_calibration_result',
                ok,
                exitCode: code,
                stdout: stdout.slice(-4000),  // cap to avoid flooding the browser
                stderr: stderr.slice(-2000),
                ...metrics,
            });
        });
        child.on('error', (err) => {
            clearTimeout(killTimer);
            this._cameraCalInFlight = false;
            console.error('[gt-cal] spawn failed:', err.message);
            this._broadcast({
                type: 'camera_calibration_result',
                ok: false,
                error: `spawn failed: ${err.message}`,
            });
        });
    }

    async _postSnapshotReload() {
        // POST /reload_calibration on the snapshot service. No-op if down.
        return new Promise((resolve) => {
            const req = http.request(
                `${GT_SNAPSHOT_URL}/reload_calibration`,
                { method: 'POST', timeout: 2000 },
                (res) => {
                    res.resume();
                    res.on('end', resolve);
                },
            );
            req.on('error', resolve);  // service probably not running — fine
            req.on('timeout', () => { req.destroy(); resolve(); });
            req.end();
        });
    }

    _handleExperimentArm(ws, msg) {
        if (!this.experimentRunner) {
            this._replyError(ws, 'experiment_error', 'runner unavailable');
            return;
        }
        try {
            this.experimentRunner.arm({
                trajectory: msg.trajectory,
                speed: Number(msg.speed),
                rep: Number(msg.rep) || 1,
                operatorNotes: String(msg.operatorNotes || ''),
                demo: msg.demo === true,
            });
        } catch (err) {
            this._replyError(ws, 'experiment_error', err.message);
        }
    }

    _handleExperimentAction(ws, action, ...args) {
        if (!this.experimentRunner) {
            this._replyError(ws, 'experiment_error', 'runner unavailable');
            return;
        }
        try {
            this.experimentRunner[action](...args);
        } catch (err) {
            this._replyError(ws, 'experiment_error', err.message);
        }
    }

    // Experiment start is the one action that's async — it runs a placement
    // preflight check (overhead-camera snapshot + tolerance comparison) before
    // issuing trajStart. On preflight failure we stay in armed state and
    // broadcast experiment_preflight_failed so the operator can reposition
    // the robot and click Start again without re-arming.
    async _handleExperimentStart(ws) {
        if (!this.experimentRunner) {
            this._replyError(ws, 'experiment_error', 'runner unavailable');
            return;
        }
        // In-flight guard: the preflight snapshot takes ~2.5 s (camera
        // pipeline content latency), so two rapid experiment_start messages
        // — e.g. from a double-click or two browser sessions — would each
        // await their own preflight concurrently, both pass, both try to
        // call runner.start(); the second one throws "cannot start in state
        // running". Reject the second request cleanly instead.
        if (this._experimentStartInFlight) {
            this._replyError(ws, 'experiment_error',
                'another experiment_start is already being processed');
            return;
        }
        this._experimentStartInFlight = true;
        try {
            // preflight() itself emits `preflight_ok` / `preflight_failed`
            // on the runner's event stream; the fan-out listeners set up
            // in this constructor broadcast those to all clients. The
            // return value is still what we branch on locally to decide
            // whether to proceed to start().
            const preflight = await this.experimentRunner.preflight();
            if (!preflight.ok) {
                return;
            }
            this.experimentRunner.start();
        } catch (err) {
            // Broadcast, not reply: a preflight or start failure leaves
            // the runner in whatever state the throw interrupted — usually
            // `armed` (if preflight threw before start(), the arm-timer
            // auto-aborts after 5 min) but possibly `running` or
            // `aborted` (if the throw came from inside start() after it
            // transitioned state). Either way, every connected dashboard
            // — not just the ws that clicked Start — needs to see the
            // error. `_replyError` would only reach the caller's ws.
            this._broadcast({
                type: 'experiment_error',
                error: err.message,
                stack: err.stack,
            });
        } finally {
            this._experimentStartInFlight = false;
        }
    }

    _handleExperimentGroundTruth(ws, msg) {
        if (!this.experimentRunner) {
            this._replyError(ws, 'experiment_error', 'runner unavailable');
            return;
        }
        try {
            const xMeas = Number(msg.xMeas);
            const yMeas = Number(msg.yMeas);
            const thetaDegMeas = Number(msg.thetaDegMeas);
            if (!Number.isFinite(xMeas) || !Number.isFinite(yMeas) || !Number.isFinite(thetaDegMeas)) {
                this._replyError(ws, 'experiment_error', 'ground truth values must be finite numbers');
                return;
            }
            this.experimentRunner.submitGroundTruth({
                xMeas,
                yMeas,
                thetaDegMeas,
                passFail: msg.passFail,
                notes: String(msg.notes || ''),
            });
        } catch (err) {
            this._replyError(ws, 'experiment_error', err.message);
        }
    }

    // Trigger a ground-truth snapshot against the aruco_detector HTTP service.
    // Valid only while the runner is in awaiting_ground_truth — the filename
    // path uses that run's (trajectory, runId) so snapshots group with
    // their run and don't accumulate in an unrelated bucket.
    _handleExperimentCaptureSnapshot(ws, msg) {
        if (!this.experimentRunner) {
            this._replyError(ws, 'experiment_error', 'runner unavailable');
            return;
        }
        if (this._snapshotInFlight) {
            this._broadcast({
                type: 'experiment_snapshot_result',
                ok: false,
                error: 'another snapshot is already in progress',
            });
            return;
        }
        const state = this.experimentRunner.getState();
        if (state !== 'awaiting_ground_truth') {
            this._replyError(ws, 'experiment_error',
                `snapshot allowed only in awaiting_ground_truth (state=${state})`);
            return;
        }
        const run = this.experimentRunner.getRun();
        if (!run) {
            this._replyError(ws, 'experiment_error', 'no active run');
            return;
        }
        this._snapshotInFlight = true;
        const label = String(msg.label || 'end');
        const params = new URLSearchParams({
            trajectory: String(run.trajectory || 'uncategorized'),
            run_id: String(run.runId || `run_${Date.now()}`),
            label,
        });
        const url = `${GT_SNAPSHOT_URL}/snapshot?${params.toString()}`;
        const req = http.get(url, { timeout: 8000 }, (res) => {
            let body = '';
            res.setEncoding('utf-8');
            res.on('data', (chunk) => { body += chunk; });
            res.on('end', () => {
                this._snapshotInFlight = false;
                let parsed;
                try { parsed = JSON.parse(body); }
                catch (err) {
                    this._broadcast({
                        type: 'experiment_snapshot_result',
                        ok: false,
                        error: `invalid JSON from snapshot service: ${err.message}`,
                    });
                    return;
                }
                const ok = res.statusCode === 200 && parsed.detected === true;
                this._broadcast({
                    type: 'experiment_snapshot_result',
                    ok,
                    label,
                    x: parsed.x ?? null,
                    y: parsed.y ?? null,
                    theta: parsed.theta ?? null,
                    thetaDeg: parsed.theta_deg ?? null,
                    sidePx: parsed.side_px ?? null,
                    imagePath: parsed.image_path ?? null,
                    error: ok ? null : (parsed.error || `snapshot service returned ${res.statusCode}`),
                });
            });
        });
        req.on('timeout', () => {
            req.destroy();
            this._snapshotInFlight = false;
            this._broadcast({
                type: 'experiment_snapshot_result',
                ok: false,
                error: 'snapshot service did not respond within 8s',
            });
        });
        req.on('error', (err) => {
            this._snapshotInFlight = false;
            this._broadcast({
                type: 'experiment_snapshot_result',
                ok: false,
                error: `snapshot service unreachable: ${err.message} (is aruco_detector.py running?)`,
            });
        });
    }

    // Tier-0 auto-cal snapshot. Runs outside the experiment state machine
    // so the cal page can fire snapshots at will between drive pulses.
    // Parallels _handleExperimentCaptureSnapshot but uses a separate
    // in-flight flag and a dedicated result message so the cal flow and
    // experiment flow can't interleave or clobber each other's UI state.
    _handleCalSnapshot(ws, msg) {
        if (this._calSnapshotInFlight) {
            this._broadcast({
                type: 'cal_snapshot_result',
                ok: false,
                error: 'another cal snapshot is already in progress',
            });
            return;
        }
        this._calSnapshotInFlight = true;
        const label = String(msg.label || 'autocal');
        const runId = String(msg.runId || `cal_${Date.now()}`);
        const params = new URLSearchParams({
            trajectory: 'calibration',
            run_id: runId,
            label,
        });
        const url = `${GT_SNAPSHOT_URL}/snapshot?${params.toString()}`;
        const req = http.get(url, { timeout: 8000 }, (res) => {
            let body = '';
            res.setEncoding('utf-8');
            res.on('data', (chunk) => { body += chunk; });
            res.on('end', () => {
                this._calSnapshotInFlight = false;
                let parsed;
                try { parsed = JSON.parse(body); }
                catch (err) {
                    this._broadcast({
                        type: 'cal_snapshot_result',
                        ok: false,
                        label,
                        runId,
                        error: `invalid JSON from snapshot service: ${err.message}`,
                    });
                    return;
                }
                const ok = res.statusCode === 200 && parsed.detected === true;
                this._broadcast({
                    type: 'cal_snapshot_result',
                    ok,
                    label,
                    runId,
                    x: parsed.x ?? null,
                    y: parsed.y ?? null,
                    theta: parsed.theta ?? null,
                    thetaDeg: parsed.theta_deg ?? null,
                    sidePx: parsed.side_px ?? null,
                    imagePath: parsed.image_path ?? null,
                    error: ok ? null : (parsed.error || `snapshot service returned ${res.statusCode}`),
                });
            });
        });
        req.on('timeout', () => {
            req.destroy();
            this._calSnapshotInFlight = false;
            this._broadcast({
                type: 'cal_snapshot_result',
                ok: false,
                label,
                runId,
                error: 'snapshot service did not respond within 8s',
            });
        });
        req.on('error', (err) => {
            this._calSnapshotInFlight = false;
            this._broadcast({
                type: 'cal_snapshot_result',
                ok: false,
                label,
                runId,
                error: `snapshot service unreachable: ${err.message} (is aruco_detector.py running?)`,
            });
        });
    }

    _replyError(ws, type, error) {
        if (ws.readyState === 1) {
            ws.send(JSON.stringify({ type, error }));
        }
    }

    handleGetRobotIp(ws) {
        if (ws.readyState === 1) {
            ws.send(JSON.stringify({
                type: 'robotIp',
                ip: config.robot.ip,
            }));
        }
    }

    async handleSetRobotIp(ws, ip) {
        if (!ip || typeof ip !== 'string') {
            if (ws.readyState === 1) {
                ws.send(JSON.stringify({ type: 'robotIpError', error: 'Invalid IP address' }));
            }
            return;
        }

        // Update config
        config.robot.ip = ip;
        console.log(`Robot IP changed to ${ip}`);

        // Disconnect and reconnect robot
        if (this.robot) {
            this.robot.disconnect();
            // Give it a moment then reconnect
            setTimeout(() => {
                this.robot.connect();
            }, 500);
        }

        // Notify all clients
        const msg = JSON.stringify({ type: 'robotIpChanged', ip: ip });
        for (const client of this.clients) {
            if (client.readyState === 1) {
                client.send(msg);
            }
        }
    }

    async handleListPorts(ws) {
        try {
            const { SerialPort } = await import('serialport');
            const ports = await SerialPort.list();
            if (ws.readyState === 1) {
                ws.send(JSON.stringify({
                    type: 'portList',
                    ports: ports.map(p => ({
                        path: p.path,
                        manufacturer: p.manufacturer,
                        pnpId: p.pnpId,
                        vendorId: p.vendorId,
                        productId: p.productId,
                    })),
                }));
            }
        } catch (err) {
            console.error('Failed to list ports:', err.message);
            if (ws.readyState === 1) {
                ws.send(JSON.stringify({
                    type: 'portList',
                    ports: [],
                }));
            }
        }
    }

    broadcastState() {
        if (this.clients.size === 0) return;

        const state = this.getStateObject();
        const message = JSON.stringify(state);

        for (const client of this.clients) {
            if (client.readyState === 1) { // WebSocket.OPEN
                client.send(message);
            }
        }
    }

    sendState(ws) {
        if (ws.readyState === 1) {
            ws.send(JSON.stringify(this.getStateObject()));
        }
    }

    getStateObject() {
        const sensors = this.robot ? this.robot.getSensors() : null;
        const pose = this.localization ? this.localization.getPose() : { x: 0, y: 0, theta: 0, thetaDeg: 0, tier: 1 };
        const logStatus = this.logger ? this.logger.getStatus() : { rowCount: 0, filename: null };
        const command = this.getCommand();

        return {
            type: 'state',
            timestamp: Date.now(),
            connected: this.robot ? this.robot.isConnected() : false,
            robotIp: config.robot.ip,
            sensors: sensors ? {
                enc: sensors.encoders,
                vel: sensors.velocities,
                imu: {
                    yaw: sensors.imu.yaw,
                    pitch: sensors.imu.pitch,
                    roll: sensors.imu.roll,
                    gyroZ: sensors.imu.gyroZ,
                    accelX: sensors.imu.accelX,
                    accelY: sensors.imu.accelY,
                },
                cal: sensors.calibration,
                robotUtc: sensors.robotUtc,      // Robot NTP time when sensor was read
                serverReceivedAt: sensors.receivedAt, // Server time when sensor data arrived
                imuStuck: sensors.imuStuck,      // Firmware watchdog — sticky until power-cycle
                heap: sensors.heap || null,      // {free, min} ESP32 heap telemetry
            } : null,
            pose: pose,
            firmwarePose: sensors?.firmwarePose || null,
            command: command,
            logging: logStatus,
            config: {
                tier: pose.tier,
                imuWeight: config.fusion.imuWeight,
            },
            experiment: this.experimentRunner ? {
                state: this.experimentRunner.getState(),
                run: this.experimentRunner.getRun(),
            } : null,
        };
    }

    // Broadcast calibration pong (latency measurement) to all browsers
    broadcastPongCal(msg) {
        const pongMsg = JSON.stringify({
            type: 'pong_cal',
            ts: msg.ts,                     // browser send time
            ts_server_fwd: msg.ts_server_fwd, // server forward time
            rt: msg.rt,                     // robot UTC time at receipt
            ntpSynced: msg.ntpSynced,       // robot NTP sync status
            ts_server_ret: Date.now(),      // server return time
        });

        for (const client of this.clients) {
            if (client.readyState === 1) {
                client.send(pongMsg);
            }
        }
    }

    // Broadcast robot info response to all browsers
    broadcastRobotInfo(info) {
        // Update per-direction motor gains from robot if present.
        // Validate bounds: gains outside [0.3, 3.0] indicate corruption.
        const validGain = (g) => Number.isFinite(g) && g >= 0.3 && g <= 3.0;
        if (info.motorGainsFwd && Array.isArray(info.motorGainsFwd) && info.motorGainsFwd.length === 4) {
            const nums = info.motorGainsFwd.map(Number);
            if (nums.every(validGain)) config.physical.motorGainsFwd = nums;
        }
        if (info.motorGainsRev && Array.isArray(info.motorGainsRev) && info.motorGainsRev.length === 4) {
            const nums = info.motorGainsRev.map(Number);
            if (nums.every(validGain)) config.physical.motorGainsRev = nums;
        }

        const infoMsg = JSON.stringify({
            ...info,
            type: 'robotInfo',
            timestamp: Date.now(),
        });

        for (const client of this.clients) {
            if (client.readyState === 1) {
                client.send(infoMsg);
            }
        }
    }

    // Broadcast motor calibration result to all browsers
    broadcastMotorCalResult(msg) {
        const resultMsg = JSON.stringify({
            type: 'motor_cal_result',
            success: msg.success,
            gainsFwd: msg.gainsFwd,
            gainsRev: msg.gainsRev,
            timestamp: Date.now(),
        });

        for (const client of this.clients) {
            if (client.readyState === 1) {
                client.send(resultMsg);
            }
        }
    }

    // Broadcast a robot log message to all browsers
    broadcastLog(msg) {
        const logMsg = JSON.stringify({
            type: 'robotLog',
            timestamp: Date.now(),
            msg: msg,
        });

        for (const client of this.clients) {
            if (client.readyState === 1) {
                client.send(logMsg);
            }
        }
    }

    // Broadcast a generic ack (from robot → server) to all browsers so
    // UI elements can react (e.g. the dashboard's heading-hold checkbox
    // syncs from the firmware's authoritative state).
    broadcastAck(msg) {
        const payload = JSON.stringify(msg);
        for (const client of this.clients) {
            if (client.readyState === 1) {
                client.send(payload);
            }
        }
    }

    // Broadcast self-test events to all browsers
    broadcastSelfTestResult(msg) {
        this._broadcast(msg);
    }

    broadcastSelfTestComplete(msg) {
        this._broadcast(msg);
    }

    // Broadcast connection status change
    broadcastConnectionStatus(connected) {
        const statusMsg = JSON.stringify({
            type: 'connectionStatus',
            timestamp: Date.now(),
            connected: connected,
        });

        for (const client of this.clients) {
            if (client.readyState === 1) {
                client.send(statusMsg);
            }
        }
    }
}
