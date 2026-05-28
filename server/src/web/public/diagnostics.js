// ============================================
// Diagnostics View — Latency, WiFi, Robot Info, Debug Log
// ============================================

import { App } from './app.js';

const T = App.setText.bind(App);

// --- Latency State ---
const LATENCY_HISTORY_MAX = 60;
const rtHistory = [];
const spHistory = [];
let rtStats = { min: Infinity, max: 0, sum: 0, count: 0 };
let spStats = { min: Infinity, max: 0, sum: 0, count: 0 };
let serverOffset = 0;
let robotOffset = 0;
let robotNtpSynced = false;
let autoPingEnabled = false;
let autoPingInterval = null;

// --- WiFi State ---
const RSSI_HISTORY_MAX = 60;
const rssiHistory = [];

// ============================================
// Robot Info
// ============================================

function formatUptime(ms) {
    const sec = Math.floor(ms / 1000);
    const min = Math.floor(sec / 60);
    const hr = Math.floor(min / 60);
    if (hr > 0) return hr + 'h ' + (min % 60) + 'm';
    if (min > 0) return min + 'm ' + (sec % 60) + 's';
    return sec + 's';
}

function updateRobotInfo(info) {
    T('g-info-firmware', info.firmware || '--');
    T('g-info-uptime', formatUptime(info.uptime || 0));
    T('g-info-ip', info.ip || '--');
    T('g-info-mac', info.mac || '--');
    T('g-info-heap', info.freeHeap ? (info.freeHeap / 1024).toFixed(0) + ' KB' : '--');
    T('g-info-imu', info.imuAvailable ? 'Available' : 'Not found');

    // WiFi
    T('g-wifi-ssid', (info.ssid || '--') + (info.apMode ? ' (AP)' : ''));
    const rssi = info.rssi || 0;
    T('g-wifi-rssi', rssi + ' dBm');

    // RSSI bar
    const pct = Math.max(0, Math.min(100, ((rssi + 90) / 60) * 100));
    const bar = document.getElementById('g-rssi-bar');
    if (bar) {
        bar.style.width = pct + '%';
        bar.style.background = pct > 60 ? 'var(--ak-success)' : pct > 30 ? 'var(--ak-warning)' : 'var(--ak-error)';
    }

    // RSSI sparkline
    rssiHistory.push(rssi);
    if (rssiHistory.length > RSSI_HISTORY_MAX) rssiHistory.shift();
    drawRssiSparkline();

    // NTP sync
    if (info.ntpSynced !== undefined) {
        robotNtpSynced = info.ntpSynced;
        updateRobotSyncUI();
    }
}

function updateRobotSyncUI() {
    const dot = document.getElementById('g-robot-sync-dot');
    const text = document.getElementById('g-robot-sync-text');
    if (dot) dot.className = 'cal-dot ' + (robotNtpSynced ? 'cal-3' : 'cal-1');
    if (text) text.textContent = robotNtpSynced ? 'NTP synced' : 'No NTP (AP mode?)';
}

// ============================================
// Latency
// ============================================

function sendPingCal() {
    App.send({ type: 'ping_cal', ts: Date.now() });
}

function handlePongCal(msg) {
    const now = Date.now();
    const rtt = now - msg.ts;

    // Per-hop
    const hopBtoS = msg.ts_server_fwd - msg.ts;
    const hopStoR = msg.rt ? (msg.rt - msg.ts_server_fwd) : null;
    const hopRtoS = msg.rt ? (msg.ts_server_ret - msg.rt) : null;
    const hopStoB = now - msg.ts_server_ret;

    T('g-hop-bs', hopBtoS + ' ms');
    T('g-hop-sr', hopStoR !== null ? hopStoR + ' ms' : '--');
    T('g-hop-rs', hopRtoS !== null ? hopRtoS + ' ms' : '--');
    T('g-hop-sb', hopStoB + ' ms');

    // Server offset
    serverOffset = Math.round((hopBtoS - hopStoB) / 2);
    T('g-offset-server', (serverOffset >= 0 ? '+' : '') + serverOffset + ' ms');

    // Robot offset
    if (msg.rt && msg.ntpSynced) {
        robotOffset = Math.round(msg.rt - (msg.ts + rtt / 2));
        T('g-offset-robot', (robotOffset >= 0 ? '+' : '') + robotOffset + ' ms');
    }

    // NTP status
    if (msg.ntpSynced !== undefined) {
        robotNtpSynced = msg.ntpSynced;
        updateRobotSyncUI();
    }

    // Roundtrip stats
    recordLatency(rtt, rtHistory, rtStats);
    rtStats = updateLatencyStats(rtHistory);
    updateLatencyUI('rt', rtt, rtStats);
    drawLatencySparkline('g-lat-rt-sparkline', rtHistory, 500);
}

function recordLatency(value, history, _stats) {
    history.push(value);
    if (history.length > LATENCY_HISTORY_MAX) history.shift();
}

function updateLatencyStats(history) {
    if (history.length === 0) return { min: Infinity, max: 0, sum: 0, count: 0 };
    let min = Infinity, max = 0, sum = 0;
    for (const v of history) {
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
    }
    return { min, max, sum, count: history.length };
}

function updateLatencyUI(prefix, current, stats) {
    const color = current < 50 ? 'var(--ak-success)' : current < 150 ? 'var(--ak-warning)' : 'var(--ak-error)';
    const valEl = document.getElementById('g-lat-' + prefix + '-val');
    if (valEl) {
        valEl.textContent = current + ' ms';
        valEl.style.color = color;
    }
    T('g-lat-' + prefix + '-min', stats.count > 0 ? stats.min + ' ms' : '--');
    T('g-lat-' + prefix + '-avg', stats.count > 0 ? Math.round(stats.sum / stats.count) + ' ms' : '--');
    T('g-lat-' + prefix + '-max', stats.count > 0 ? stats.max + ' ms' : '--');
}

function toggleAutoPing() {
    autoPingEnabled = !autoPingEnabled;
    const btn = document.getElementById('g-lat-auto-btn');
    if (autoPingEnabled) {
        autoPingInterval = setInterval(sendPingCal, 1000);
        if (btn) btn.textContent = 'Stop Auto';
    } else {
        clearInterval(autoPingInterval);
        autoPingInterval = null;
        if (btn) btn.textContent = 'Auto Ping';
    }
}

function resetLatencyStats() {
    rtHistory.length = 0;
    spHistory.length = 0;
    rtStats = { min: Infinity, max: 0, sum: 0, count: 0 };
    spStats = { min: Infinity, max: 0, sum: 0, count: 0 };
    ['rt', 'sp'].forEach(p => {
        const valEl = document.getElementById('g-lat-' + p + '-val');
        if (valEl) { valEl.textContent = '-- ms'; valEl.style.color = 'var(--ak-accent)'; }
        T('g-lat-' + p + '-min', '--');
        T('g-lat-' + p + '-avg', '--');
        T('g-lat-' + p + '-max', '--');
    });
    T('g-offset-server', '--');
    T('g-offset-robot', '--');
    ['bs', 'sr', 'rs', 'sb'].forEach(h => T('g-hop-' + h, '--'));
}

// ============================================
// Sensor Pipeline Latency
// ============================================

function updateSensorPipelineLatency(state) {
    if (!state.sensors) return;
    const now = Date.now();

    if (state.sensors.robotUtc && robotNtpSynced) {
        const spLatency = now - state.sensors.robotUtc;
        if (Math.abs(spLatency) < 5000) {
            recordLatency(Math.max(0, spLatency), spHistory, spStats);
            spStats = updateLatencyStats(spHistory);
            updateLatencyUI('sp', Math.max(0, spLatency), spStats);
            drawLatencySparkline('g-lat-sp-sparkline', spHistory, 200);
        }
    } else if (state.timestamp) {
        const spLatency = now - state.timestamp;
        if (spLatency >= 0 && spLatency < 5000) {
            recordLatency(spLatency, spHistory, spStats);
            spStats = updateLatencyStats(spHistory);
            updateLatencyUI('sp', spLatency, spStats);
            drawLatencySparkline('g-lat-sp-sparkline', spHistory, 200);
        }
    }
}

// ============================================
// Sparklines
// ============================================

function themeTokens() {
    return (window.Theme && window.Theme.tokens) ? window.Theme.tokens() : {};
}

function drawRssiSparkline() {
    const canvas = document.getElementById('g-rssi-sparkline');
    if (!canvas || rssiHistory.length < 2) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width = canvas.offsetWidth;
    const H = canvas.height = canvas.offsetHeight;
    if (W === 0 || H === 0) return;

    ctx.clearRect(0, 0, W, H);
    const minR = -90, maxR = -30;

    ctx.beginPath();
    ctx.strokeStyle = themeTokens().accent || '#3B82F6';
    ctx.lineWidth = 1.5;
    for (let i = 0; i < rssiHistory.length; i++) {
        const x = (i / (RSSI_HISTORY_MAX - 1)) * W;
        const y = H - ((rssiHistory[i] - minR) / (maxR - minR)) * H;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

function drawLatencySparkline(canvasId, history, maxY) {
    const canvas = document.getElementById(canvasId);
    if (!canvas || history.length < 2) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width = canvas.offsetWidth;
    const H = canvas.height = canvas.offsetHeight;
    if (W === 0 || H === 0) return;

    ctx.clearRect(0, 0, W, H);

    const t = themeTokens();

    // Threshold line at 50ms — warning-tinted, partially transparent
    const threshY = H - (50 / maxY) * H;
    ctx.beginPath();
    ctx.setLineDash([4, 4]);
    ctx.moveTo(0, threshY);
    ctx.lineTo(W, threshY);
    ctx.globalAlpha = 0.35;
    ctx.strokeStyle = t.warning || '#f59e0b';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.setLineDash([]);

    // Line
    ctx.beginPath();
    ctx.strokeStyle = t.accent || '#3B82F6';
    ctx.lineWidth = 1.5;
    for (let i = 0; i < history.length; i++) {
        const x = (i / (LATENCY_HISTORY_MAX - 1)) * W;
        const y = H - Math.min(history[i], maxY) / maxY * H;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

// ============================================
// Init
// ============================================

function init() {
    // Latency buttons
    document.getElementById('g-lat-auto-btn')?.addEventListener('click', toggleAutoPing);
    document.getElementById('g-lat-manual-btn')?.addEventListener('click', sendPingCal);
    document.getElementById('g-lat-reset-btn')?.addEventListener('click', resetLatencyStats);

    // Subscribe to events
    App.on('robotInfo', updateRobotInfo);
    App.on('pongCal', handlePongCal);
    App.on('state', updateSensorPipelineLatency);

    // Redraw sparklines when view becomes visible; stop auto-ping when leaving
    App.on('viewChanged', (view) => {
        if (view === 'diagnostics') {
            drawRssiSparkline();
            drawLatencySparkline('g-lat-rt-sparkline', rtHistory, 500);
            drawLatencySparkline('g-lat-sp-sparkline', spHistory, 200);
        } else {
            // Stop auto-ping to avoid unnecessary traffic while off-view
            if (autoPingInterval) {
                clearInterval(autoPingInterval);
                autoPingInterval = null;
                autoPingEnabled = false;
                const btn = document.getElementById('g-lat-auto-btn');
                if (btn) btn.textContent = 'Auto Ping';
            }
        }
    });

    // Redraw sparklines on theme change so they pick up new token colors
    if (window.Theme && typeof window.Theme.onChange === 'function') {
        window.Theme.onChange(() => {
            drawRssiSparkline();
            drawLatencySparkline('g-lat-rt-sparkline', rtHistory, 500);
            drawLatencySparkline('g-lat-sp-sparkline', spHistory, 200);
        });
    }
}

document.addEventListener('DOMContentLoaded', init);
