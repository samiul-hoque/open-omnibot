// ============================================
// Web Server Unit Tests
// ============================================

import { describe, it, beforeEach, afterEach } from 'node:test';
import assert from 'node:assert';
import { EventEmitter } from 'events';
import { WebServer } from '../../src/web/webServer.js';

// ============================================
// Mock Classes
// ============================================

class MockRobotClient extends EventEmitter {
    constructor() {
        super();
        this.connected = false;
        this.lastCommand = null;
        this.sensors = null;
    }

    isConnected() {
        return this.connected;
    }

    getSensors() {
        return this.sensors;
    }

    setVelocity(vx, vy, omega) {
        this.lastCommand = { vx, vy, omega };
        return true;
    }

    stop() {
        this.lastCommand = { vx: 0, vy: 0, omega: 0, stopped: true };
        return true;
    }

    resetEncoders() {
        return true;
    }

    setSensors(data) {
        this.sensors = data;
    }

    setConnected(connected) {
        this.connected = connected;
    }
}

class MockLocalization {
    constructor() {
        this.pose = { x: 0, y: 0, theta: 0, thetaDeg: 0, tier: 1 };
        this.resetCalled = false;
    }

    getPose() {
        return { ...this.pose };
    }

    reset() {
        this.resetCalled = true;
        this.pose = { x: 0, y: 0, theta: 0, thetaDeg: 0, tier: 1 };
    }

    setPose(pose) {
        this.pose = { ...pose };
    }
}

class MockLogger {
    constructor() {
        this.status = { rowCount: 0, filename: null };
    }

    getStatus() {
        return { ...this.status };
    }

    setStatus(status) {
        this.status = { ...status };
    }

    logEvent() {}
}

// Minimal stand-in for TrajectoryRunner — just enough to unit-test the
// experiment_capture_snapshot state guard without spinning up the real runner.
class MockExperimentRunner extends EventEmitter {
    constructor() {
        super();
        this._state = 'idle';
        this._run = null;
    }

    getState() { return this._state; }
    setState(s) { this._state = s; }
    getRun() { return this._run ? { ...this._run } : null; }
    setRun(r) { this._run = r; }
}

// ============================================
// WebServer Tests
// ============================================

describe('WebServer', () => {
    let webServer;
    let mockRobot;
    let mockLocalization;
    let mockLogger;
    let currentCommand;
    let tierChanges;
    let imuWeightChanges;

    beforeEach(() => {
        mockRobot = new MockRobotClient();
        mockLocalization = new MockLocalization();
        mockLogger = new MockLogger();
        currentCommand = { vx: 0, vy: 0, omega: 0 };
        tierChanges = [];
        imuWeightChanges = [];
    });

    afterEach(async () => {
        if (webServer) {
            webServer.stop();
            webServer = null;
        }
        // Give server time to close
        await new Promise(resolve => setTimeout(resolve, 100));
    });

    describe('constructor', () => {
        it('should accept dependencies', () => {
            webServer = new WebServer({
                robot: mockRobot,
                localization: mockLocalization,
                logger: mockLogger,
                getCommand: () => currentCommand,
                setCommand: (cmd) => { currentCommand = cmd; },
                setTier: (tier) => tierChanges.push(tier),
                setImuWeight: (weight) => imuWeightChanges.push(weight),
            });

            assert.strictEqual(webServer.robot, mockRobot);
            assert.strictEqual(webServer.localization, mockLocalization);
            assert.strictEqual(webServer.logger, mockLogger);
        });

        it('should initialize with empty client set', () => {
            webServer = new WebServer({});
            assert.strictEqual(webServer.clients.size, 0);
        });

        it('should have null server before start', () => {
            webServer = new WebServer({});
            assert.strictEqual(webServer.server, null);
            assert.strictEqual(webServer.wss, null);
        });
    });

    describe('getStateObject', () => {
        it('should return complete state object when all dependencies present', () => {
            mockRobot.setConnected(true);
            mockRobot.setSensors({
                encoders: [100, 200, 300, 400],
                velocities: [1.0, 1.1, 1.2, 1.3],
                imu: { yaw: 10, pitch: 5, roll: 3, gyroZ: 0.5, accelX: 0.1, accelY: 0.2 },
                calibration: { sys: 3, gyro: 3, accel: 3, mag: 3 },
            });
            mockLocalization.setPose({ x: 1.5, y: 2.5, theta: 0.5, thetaDeg: 28.6, tier: 2 });
            mockLogger.setStatus({ rowCount: 100, filename: 'test.csv' });
            currentCommand = { vx: 0.1, vy: 0.2, omega: 0.3 };

            webServer = new WebServer({
                robot: mockRobot,
                localization: mockLocalization,
                logger: mockLogger,
                getCommand: () => currentCommand,
            });

            const state = webServer.getStateObject();

            assert.strictEqual(state.type, 'state');
            assert.strictEqual(state.connected, true);
            assert.deepStrictEqual(state.sensors.enc, [100, 200, 300, 400]);
            assert.deepStrictEqual(state.sensors.vel, [1.0, 1.1, 1.2, 1.3]);
            assert.strictEqual(state.sensors.imu.yaw, 10);
            assert.strictEqual(state.pose.x, 1.5);
            assert.strictEqual(state.pose.y, 2.5);
            assert.deepStrictEqual(state.command, currentCommand);
            assert.strictEqual(state.logging.rowCount, 100);
        });

        it('should handle missing robot', () => {
            webServer = new WebServer({
                localization: mockLocalization,
                logger: mockLogger,
                getCommand: () => currentCommand,
            });

            const state = webServer.getStateObject();

            assert.strictEqual(state.connected, false);
            assert.strictEqual(state.sensors, null);
        });

        it('should handle missing sensors', () => {
            mockRobot.setConnected(true);
            mockRobot.setSensors(null);

            webServer = new WebServer({
                robot: mockRobot,
                localization: mockLocalization,
                getCommand: () => currentCommand,
            });

            const state = webServer.getStateObject();

            assert.strictEqual(state.sensors, null);
        });

        it('should handle missing localization', () => {
            webServer = new WebServer({
                robot: mockRobot,
                getCommand: () => currentCommand,
            });

            const state = webServer.getStateObject();

            assert.strictEqual(state.pose.x, 0);
            assert.strictEqual(state.pose.y, 0);
        });
    });

    describe('handleBrowserMessage', () => {
        beforeEach(() => {
            mockRobot.setConnected(true);
            webServer = new WebServer({
                robot: mockRobot,
                localization: mockLocalization,
                logger: mockLogger,
                getCommand: () => currentCommand,
                setCommand: (cmd) => { currentCommand = cmd; },
                setTier: (tier) => tierChanges.push(tier),
                setImuWeight: (weight) => imuWeightChanges.push(weight),
            });
        });

        it('should handle cmd message', () => {
            webServer.handleBrowserMessage(null, {
                type: 'cmd',
                vx: 0.15,
                vy: 0.1,
                w: 0.5,
            });

            assert.strictEqual(mockRobot.lastCommand.vx, 0.15);
            assert.strictEqual(mockRobot.lastCommand.vy, 0.1);
            assert.strictEqual(mockRobot.lastCommand.omega, 0.5);
            assert.strictEqual(currentCommand.vx, 0.15);
            assert.strictEqual(currentCommand.omega, 0.5);
        });

        it('should handle stop message', () => {
            currentCommand = { vx: 0.15, vy: 0, omega: 0 };

            webServer.handleBrowserMessage(null, { type: 'stop' });

            assert.strictEqual(mockRobot.lastCommand.stopped, true);
            assert.strictEqual(currentCommand.vx, 0);
            assert.strictEqual(currentCommand.vy, 0);
            assert.strictEqual(currentCommand.omega, 0);
        });

        it('should handle resetPose message', () => {
            mockLocalization.pose = { x: 5, y: 10, theta: 1.5 };

            webServer.handleBrowserMessage(null, { type: 'resetPose' });

            assert.strictEqual(mockLocalization.resetCalled, true);
        });

        it('should handle setTier message for all valid tiers', () => {
            webServer.handleBrowserMessage(null, { type: 'setTier', tier: 0 });
            webServer.handleBrowserMessage(null, { type: 'setTier', tier: 1 });
            webServer.handleBrowserMessage(null, { type: 'setTier', tier: 2 });

            // Tier 0 (open-loop baseline) is valid alongside 1 and 2.
            // Regression for the silent-skip bug where the earlier
            // `tier >= 1 && tier <= 2` bound dropped dashboard tier-0
            // selections without any log — producing closed-loop runs
            // that looked like tier-0 runs in the UI.
            assert.deepStrictEqual(tierChanges, [0, 1, 2]);
        });

        it('should reject invalid tier values', () => {
            webServer.handleBrowserMessage(null, { type: 'setTier', tier: 4 });
            webServer.handleBrowserMessage(null, { type: 'setTier', tier: -1 });
            webServer.handleBrowserMessage(null, { type: 'setTier', tier: 'invalid' });

            assert.deepStrictEqual(tierChanges, []);
        });

        it('should handle setImuWeight message', () => {
            webServer.handleBrowserMessage(null, { type: 'setImuWeight', weight: 0.95 });

            assert.deepStrictEqual(imuWeightChanges, [0.95]);
        });

        it('should reject invalid IMU weight values', () => {
            webServer.handleBrowserMessage(null, { type: 'setImuWeight', weight: -0.1 });
            webServer.handleBrowserMessage(null, { type: 'setImuWeight', weight: 1.5 });

            assert.deepStrictEqual(imuWeightChanges, []);
        });

        it('should handle cmd with NaN values gracefully', () => {
            webServer.handleBrowserMessage(null, {
                type: 'cmd',
                vx: 'not a number',
                vy: undefined,
                w: null,
            });

            assert.strictEqual(mockRobot.lastCommand.vx, 0);
            assert.strictEqual(mockRobot.lastCommand.vy, 0);
            assert.strictEqual(mockRobot.lastCommand.omega, 0);
        });

        it('should not send commands when robot disconnected', () => {
            mockRobot.setConnected(false);

            webServer.handleBrowserMessage(null, {
                type: 'cmd',
                vx: 0.15,
                vy: 0,
                w: 0,
            });

            assert.strictEqual(mockRobot.lastCommand, null);
        });
    });

    describe('broadcastLog', () => {
        it('should format log message correctly', () => {
            webServer = new WebServer({});

            // Create a mock WebSocket client
            const messages = [];
            const mockWs = {
                readyState: 1, // WebSocket.OPEN
                send: (data) => messages.push(JSON.parse(data)),
                close: () => {},
            };

            webServer.clients.add(mockWs);
            webServer.broadcastLog('Test message');

            assert.strictEqual(messages.length, 1);
            assert.strictEqual(messages[0].type, 'robotLog');
            assert.strictEqual(messages[0].msg, 'Test message');
            assert.ok(messages[0].timestamp);
        });
    });

    describe('broadcastConnectionStatus', () => {
        it('should broadcast connection status', () => {
            webServer = new WebServer({});

            const messages = [];
            const mockWs = {
                readyState: 1,
                send: (data) => messages.push(JSON.parse(data)),
                close: () => {},
            };

            webServer.clients.add(mockWs);
            webServer.broadcastConnectionStatus(true);

            assert.strictEqual(messages.length, 1);
            assert.strictEqual(messages[0].type, 'connectionStatus');
            assert.strictEqual(messages[0].connected, true);
        });
    });

    describe('broadcastState', () => {
        it('should not broadcast when no clients connected', () => {
            webServer = new WebServer({
                robot: mockRobot,
                localization: mockLocalization,
            });

            // Should not throw
            webServer.broadcastState();
        });

        it('should broadcast to all connected clients', () => {
            mockRobot.setConnected(true);
            webServer = new WebServer({
                robot: mockRobot,
                localization: mockLocalization,
                getCommand: () => currentCommand,
            });

            const messages1 = [];
            const messages2 = [];

            const mockWs1 = {
                readyState: 1,
                send: (data) => messages1.push(JSON.parse(data)),
                close: () => {},
            };
            const mockWs2 = {
                readyState: 1,
                send: (data) => messages2.push(JSON.parse(data)),
                close: () => {},
            };

            webServer.clients.add(mockWs1);
            webServer.clients.add(mockWs2);
            webServer.broadcastState();

            assert.strictEqual(messages1.length, 1);
            assert.strictEqual(messages2.length, 1);
            assert.strictEqual(messages1[0].type, 'state');
            assert.strictEqual(messages2[0].type, 'state');
        });

        it('should skip clients with closed connections', () => {
            webServer = new WebServer({
                robot: mockRobot,
                localization: mockLocalization,
                getCommand: () => currentCommand,
            });

            const messages = [];
            const mockWsOpen = {
                readyState: 1,
                send: (data) => messages.push(JSON.parse(data)),
                close: () => {},
            };
            const mockWsClosed = {
                readyState: 3, // WebSocket.CLOSED
                send: () => { throw new Error('Should not be called'); },
                close: () => {},
            };

            webServer.clients.add(mockWsOpen);
            webServer.clients.add(mockWsClosed);
            webServer.broadcastState();

            assert.strictEqual(messages.length, 1);
        });
    });

    describe('stop', () => {
        it('should clean up all resources', () => {
            webServer = new WebServer({});

            const closeCalls = [];
            const mockWs = {
                readyState: 1,
                close: () => closeCalls.push(true),
            };

            webServer.clients.add(mockWs);
            webServer.broadcastInterval = setInterval(() => {}, 1000);

            webServer.stop();

            assert.strictEqual(webServer.clients.size, 0);
            assert.strictEqual(webServer.broadcastInterval, null);
            assert.strictEqual(closeCalls.length, 1);
        });
    });
});

// ============================================
// HTTP Handler Tests
// ============================================

describe('WebServer HTTP Handler', () => {
    let webServer;

    afterEach(() => {
        if (webServer) {
            webServer.stop();
            webServer = null;
        }
    });

    it('should return 404 for non-existent files', async () => {
        const writeHeadCalls = [];
        const mockRes = {
            writeHead: (status) => writeHeadCalls.push(status),
            end: () => {},
        };
        const mockReq = { url: '/nonexistent.html' };

        webServer = new WebServer({});
        webServer.handleHttpRequest(mockReq, mockRes);

        // Due to async fs.readFile, we need to wait
        await new Promise(resolve => setTimeout(resolve, 100));

        assert.ok(writeHeadCalls.includes(404));
    });

    it('should prevent directory traversal attacks', async () => {
        const writeHeadCalls = [];
        const mockRes = {
            writeHead: (status) => writeHeadCalls.push(status),
            end: () => {},
        };
        const mockReq = { url: '/../../../etc/passwd' };

        webServer = new WebServer({});
        webServer.handleHttpRequest(mockReq, mockRes);

        // Should return 403 or 404, not the actual file
        await new Promise(resolve => setTimeout(resolve, 100));

        assert.ok(writeHeadCalls.includes(403) || writeHeadCalls.includes(404));
    });
});

// ============================================
// /api/snapshot-image route
// ============================================

describe('snapshot-image route', () => {
    let webServer;

    afterEach(() => {
        if (webServer) {
            webServer.stop();
            webServer = null;
        }
    });

    function serveAndCapture(url) {
        const calls = { status: null, body: null };
        const mockRes = {
            writeHead: (status) => { calls.status = status; },
            end: (data) => { calls.body = data; },
        };
        webServer.handleHttpRequest({ url }, mockRes);
        // readFile is async; wait a tick
        return new Promise((r) => setTimeout(() => r(calls), 50));
    }

    it('rejects ../.. path traversal with 403', async () => {
        webServer = new WebServer({});
        const r = await serveAndCapture('/api/snapshot-image?path=' + encodeURIComponent('../../etc/passwd'));
        assert.strictEqual(r.status, 403);
    });

    it('rejects non-.png extensions with 403', async () => {
        webServer = new WebServer({});
        const r = await serveAndCapture('/api/snapshot-image?path=' + encodeURIComponent('evaluation/snapshots/notes.md'));
        assert.strictEqual(r.status, 403);
    });

    it('rejects paths outside evaluation/snapshots/ with 403', async () => {
        webServer = new WebServer({});
        const r = await serveAndCapture('/api/snapshot-image?path=' + encodeURIComponent('evaluation/ground_truth/calibration.json'));
        assert.strictEqual(r.status, 403);
    });

    it('returns 404 for a snapshot path that does not exist yet', async () => {
        webServer = new WebServer({});
        const r = await serveAndCapture('/api/snapshot-image?path=' + encodeURIComponent('evaluation/snapshots/nonexistent/run/end.png'));
        assert.strictEqual(r.status, 404);
    });
});

// ============================================
// experiment_capture_snapshot state guard
// ============================================

describe('experiment_capture_snapshot handler', () => {
    let webServer;
    let runner;
    let broadcasts;
    let replyErrors;

    beforeEach(() => {
        runner = new MockExperimentRunner();
        broadcasts = [];
        replyErrors = [];
        webServer = new WebServer({
            robot: new MockRobotClient(),
            localization: new MockLocalization(),
            logger: new MockLogger(),
            experimentRunner: runner,
            getCommand: () => ({ vx: 0, vy: 0, omega: 0 }),
        });
        // Capture broadcasts and direct replies instead of opening real sockets.
        webServer._broadcast = (msg) => broadcasts.push(msg);
        webServer._replyError = (_ws, type, error) => replyErrors.push({ type, error });
    });

    afterEach(() => {
        if (webServer) {
            webServer.stop();
            webServer = null;
        }
    });

    it('rejects when runner state is not awaiting_ground_truth', () => {
        runner.setState('idle');
        webServer.handleBrowserMessage(null, { type: 'experiment_capture_snapshot', label: 'end' });
        assert.strictEqual(replyErrors.length, 1);
        assert.match(replyErrors[0].error, /awaiting_ground_truth/);
        assert.strictEqual(broadcasts.length, 0);
    });

    it('rejects when no active run', () => {
        runner.setState('awaiting_ground_truth');
        runner.setRun(null);
        webServer.handleBrowserMessage(null, { type: 'experiment_capture_snapshot', label: 'end' });
        assert.strictEqual(replyErrors.length, 1);
        assert.match(replyErrors[0].error, /no active run/);
    });

    it('rejects a second snapshot while one is in flight', () => {
        runner.setState('awaiting_ground_truth');
        runner.setRun({ runId: 'exp_test_1', trajectory: 'straight_2m' });
        webServer._snapshotInFlight = true;  // simulate prior request mid-flight
        webServer.handleBrowserMessage(null, { type: 'experiment_capture_snapshot', label: 'end' });
        assert.strictEqual(broadcasts.length, 1);
        assert.strictEqual(broadcasts[0].ok, false);
        assert.match(broadcasts[0].error, /already in progress/);
    });
});

// ============================================
// calibrate_camera in-flight guard
// ============================================

describe('calibrate_camera in-flight guard', () => {
    let webServer;
    let broadcasts;

    beforeEach(() => {
        broadcasts = [];
        webServer = new WebServer({});
        webServer._broadcast = (msg) => broadcasts.push(msg);
    });

    afterEach(() => {
        if (webServer) {
            webServer.stop();
            webServer = null;
        }
    });

    it('broadcasts a failure when a calibration is already running', () => {
        webServer._cameraCalInFlight = true;
        webServer.handleBrowserMessage(null, { type: 'calibrate_camera' });
        assert.strictEqual(broadcasts.length, 1);
        assert.strictEqual(broadcasts[0].ok, false);
        assert.match(broadcasts[0].error, /already running/);
    });
});
