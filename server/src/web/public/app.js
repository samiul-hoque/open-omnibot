// ============================================
// Omni-2 Command Center — SPA Controller
// ============================================
// Single WebSocket, event bus, view routing, shared utilities.
// Each view module (dashboard.js, calibration.js, diagnostics.js)
// registers listeners via App.on() and sends commands via App.send().

/* global Theme */

const MAX_LOG_LINES = 200;

export const App = {
    ws: null,
    connected: false,
    currentView: 'dashboard',
    demoMode: false,

    // --- Event Bus ---
    _listeners: {},

    on(event, fn) {
        (this._listeners[event] = this._listeners[event] || []).push(fn);
    },

    emit(event, data) {
        (this._listeners[event] || []).forEach(fn => fn(data));
    },

    // --- WebSocket ---
    _reconnectAttempts: 0,
    _reconnectTimer: null,
    _infoInterval: null,

    connect() {
        if (this.demoMode) return;
        const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${proto}//${location.host}/ws`;

        this.ws = new WebSocket(url);

        this.ws.onopen = () => {
            this.connected = true;
            this._reconnectAttempts = 0;
            this.addLog('Connected to server', 'info');

            // Poll robot info every 2s
            clearInterval(this._infoInterval);
            this._infoInterval = setInterval(() => this.send({ type: 'get_info' }), 2000);
        };

        this.ws.onmessage = (e) => {
            try {
                const msg = JSON.parse(e.data);
                this._handleMessage(msg);
            } catch (err) {
                console.error('Parse error:', err);
            }
        };

        this.ws.onclose = () => {
            const wasConnected = this.connected;
            this.connected = false;
            clearInterval(this._infoInterval);
            if (wasConnected) {
                this._updateConnectionUI(false);
                this.addLog('Disconnected from server', 'error');
            }
            this._scheduleReconnect();
        };

        this.ws.onerror = () => {
            clearInterval(this._infoInterval);
        };
    },

    _scheduleReconnect() {
        if (this.demoMode) return;
        const delay = Math.min(3000 * Math.pow(1.5, this._reconnectAttempts), 10000);
        this._reconnectAttempts++;
        clearTimeout(this._reconnectTimer);
        this._reconnectTimer = setTimeout(() => this.connect(), delay);
    },

    send(data) {
        if (this.demoMode) {
            this.emit('demoSend', data);
            return true;
        }
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
            return true;
        }
        // WS not open — caller should restore any optimistic UI (disabled
        // buttons, "loading…" labels) since no response will ever arrive.
        // Warn so silent drops are at least visible in devtools — the
        // tier-selector click during a reconnect bounce was invisible
        // before this, and cost a whole experiment run.
        console.warn('[App.send] dropped message, WS not open:', data && data.type);
        return false;
    },

    _handleMessage(msg) {
        switch (msg.type) {
        case 'state':
            this._updateConnectionUI(!!msg.connected);
            this.emit('state', msg);
            break;
        case 'robotInfo':        this.emit('robotInfo', msg); break;
        case 'robotLog':
            this.addLog('[ROBOT] ' + msg.msg, 'info');
            this.emit('robotLog', msg);
            break;
        case 'connectionStatus':
            this._updateConnectionUI(msg.connected);
            this.emit('connectionStatus', msg);
            break;
        case 'pong_cal':         this.emit('pongCal', msg); break;
        case 'motor_cal_result': this.emit('motorCalResult', msg); break;
        case 'camera_calibration':        this.emit('cameraCalibration', msg); break;
        case 'camera_calibration_result': this.emit('cameraCalibrationResult', msg); break;
        case 'experiment_snapshot_result': this.emit('experimentSnapshotResult', msg); break;
        case 'cal_snapshot_result':        this.emit('calSnapshotResult', msg); break;
        case 'ack':              this.emit('ack', msg); break;
        case 'portList':         this.emit('portList', msg); break;
        case 'robotIp':          this.emit('robotIp', msg); break;
        case 'robotIpChanged':   this.addLog('Robot IP changed: ' + msg.ip, 'info'); break;
        case 'robotIpError':     this.emit('robotIpError', msg); break;
        case 'experiment_armed':
        case 'experiment_started':
        case 'experiment_tick':
        case 'experiment_completed':
        case 'experiment_aborted':
        case 'experiment_state':
        case 'experiment_error':
        case 'experiment_preflight_ok':
        case 'experiment_preflight_failed':
        case 'trajectory_catalog':
            this.emit(msg.type, msg);
            break;
        }
    },

    // --- View Switching ---
    switchView(viewName) {
        document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
        const el = document.getElementById('view-' + viewName);
        if (el) el.classList.add('active');
        document.querySelectorAll('.nav-item').forEach(n =>
            n.classList.toggle('active', n.dataset.view === viewName),
        );
        this.currentView = viewName;
        this.emit('viewChanged', viewName);
    },

    // --- Connection UI ---
    _updateConnectionUI(conn) {
        const dot = document.getElementById('conn-dot');
        const label = document.getElementById('conn-label');
        if (dot) dot.classList.toggle('connected', conn);
        if (label) label.textContent = conn ? 'UPLINK: STABLE' : 'UPLINK: OFFLINE';
    },

    // --- Shared Utilities ---
    setText(id, text) {
        const el = document.getElementById(id);
        if (el) el.textContent = text;
    },

    addLog(msg, level) {
        const output = document.getElementById('g-log-output');
        if (!output) return;
        const div = document.createElement('div');
        div.className = 'log-' + (level || 'info');
        const ts = new Date().toLocaleTimeString();
        div.textContent = '[' + ts + '] ' + msg;
        output.appendChild(div);
        if (output.children.length > MAX_LOG_LINES) {
            output.removeChild(output.firstChild);
        }
        output.scrollTop = output.scrollHeight;
    },

    // --- E-STOP (unified: keyboard + motor cleanup) ---
    _motorStopCallbacks: [],

    onEmergencyStop(fn) {
        this._motorStopCallbacks.push(fn);
    },

    emergencyStop() {
        this.send({ type: 'stop' });
        this._motorStopCallbacks.forEach(fn => fn());
        this.addLog('E-STOP activated', 'error');
    },

    // --- Robot Info (always-on, feeds nav bar) ---
    _setupRobotInfoHandler() {
        this.on('robotInfo', (info) => {
            this.setText('unit-firmware', 'v' + (info.firmware || '--'));
        });
    },
};

// --- Init ---
function init() {
    // Nav click handlers
    document.querySelectorAll('.nav-item').forEach(item => {
        item.addEventListener('click', () => App.switchView(item.dataset.view));
    });

    // Theme toggle
    const themeBtn = document.getElementById('theme-toggle');
    if (themeBtn && window.Theme) {
        const setIcon = () => { themeBtn.textContent = Theme.current() === 'light' ? '☾' : '☀'; };
        setIcon();
        themeBtn.addEventListener('click', () => {
            Theme.toggle();
            setIcon();
        });
    }

    // E-STOP button
    const estopBtn = document.getElementById('estop-btn');
    if (estopBtn) estopBtn.addEventListener('click', () => App.emergencyStop());

    // Spacebar E-STOP (global, except when in input)
    document.addEventListener('keydown', (e) => {
        if (e.code === 'Space' && e.target.tagName !== 'INPUT' && e.target.tagName !== 'SELECT' && e.target.tagName !== 'TEXTAREA') {
            e.preventDefault();
            App.emergencyStop();
        }
    });

    // beforeunload
    window.addEventListener('beforeunload', () => App.send({ type: 'stop' }));

    // Tier pills (segmented button group). Click or keyboard 0/1/2 ->
    // send setTier + mark the clicked pill `.pending`. The authoritative
    // `.active` class is applied in dashboard.js:applyTierVisibility
    // when the server's state broadcast confirms the switch. A dropped
    // message therefore stays visible as a pending pill + still-active
    // old pill, rather than a silent no-op.
    function requestTierSwitch(tier) {
        const pills = document.querySelectorAll('.tier-pill');
        pills.forEach(p => p.classList.remove('pending'));
        const target = document.querySelector(`.tier-pill[data-tier="${tier}"]`);
        if (target && !target.classList.contains('active')) target.classList.add('pending');
        App.send({ type: 'setTier', tier });
        App.emit('tierChanged', tier);
    }
    document.querySelectorAll('.tier-pill').forEach(btn => {
        btn.addEventListener('click', () => {
            const tier = Number(btn.dataset.tier);
            if (Number.isFinite(tier)) requestTierSwitch(tier);
        });
    });
    document.addEventListener('keydown', (e) => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') return;
        if (e.ctrlKey || e.metaKey || e.altKey) return;
        if (e.key === '0' || e.key === '1' || e.key === '2') {
            e.preventDefault();
            requestTierSwitch(Number(e.key));
        }
    });

    // IMU weight slider
    const imuWeight = document.getElementById('imu-weight');
    if (imuWeight) {
        imuWeight.addEventListener('input', (e) => {
            const w = Number(e.target.value) / 100;
            App.setText('imu-weight-val', w.toFixed(2));
            App.send({ type: 'setImuWeight', weight: w });
        });
    }

    // Demo mode
    const demoCheckbox = document.getElementById('demo-mode');
    if (demoCheckbox) {
        demoCheckbox.addEventListener('change', (e) => {
            App.demoMode = e.target.checked;
            if (App.demoMode) {
                if (App.ws) App.ws.close();
                App.addLog('Demo mode enabled', 'info');
            } else {
                App.connect();
            }
            App.emit('demoModeChanged', App.demoMode);
        });
    }

    // Robot info handler
    App._setupRobotInfoHandler();

    // Auto-connect
    App.connect();

    // Show initial view
    App.switchView('dashboard');
}

document.addEventListener('DOMContentLoaded', init);

// Expose on window for non-module consumers (docs.js)
window.App = App;
