#!/usr/bin/env node
// Motor-gain recalibration wrapper.
//
// After the front-motor PWM/direction pin fix in config.h, each firmware
// motor index drives and reads the same physical wheel. The previously-
// persisted per-wheel gains in NVS were computed under the old crossed
// wiring and no longer describe the correct (motor, wheel) pairs.
//
// This script connects directly to the robot, kicks the automated motor-
// calibration state machine (`start_motor_cal`), waits for the result,
// and persists it to NVS (`save_motor_cal`). Run with the robot on a
// stand so the wheels can spin freely for the ~13s the state machine
// takes (6 steps × (400 ms ramp + 2000 ms measure) ≈ 15s with settles).
//
// Usage:
//   ROBOT_IP=robot.local node evaluation/scripts/recalibrate_motor_gains.mjs

import WebSocket from 'ws';

const ROBOT_IP = process.env.ROBOT_IP || 'robot.local';
const ROBOT_URL = `ws://${ROBOT_IP}:80/ws`;
const CAL_TIMEOUT_MS = 45_000;
const SAVE_TIMEOUT_MS = 5_000;

function send(ws, obj) { ws.send(JSON.stringify(obj)); }
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

function awaitMessage(ws, predicate, timeoutMs, label) {
    return new Promise((resolve, reject) => {
        const timeout = setTimeout(() => {
            ws.off('message', onMsg);
            reject(new Error(`Timeout waiting for ${label}`));
        }, timeoutMs);
        const onMsg = (raw) => {
            let msg;
            try { msg = JSON.parse(raw); } catch { return; }
            if (predicate(msg)) {
                clearTimeout(timeout);
                ws.off('message', onMsg);
                resolve(msg);
            }
        };
        ws.on('message', onMsg);
    });
}

async function main() {
    console.log(`[cal] Connecting to ${ROBOT_URL}`);
    const ws = new WebSocket(ROBOT_URL);
    await new Promise((resolve, reject) => {
        ws.once('open', resolve);
        ws.once('error', reject);
    });
    console.log('[cal] Connected. Make sure the robot is on a stand (wheels free).');

    // Surface firmware logs so the user can watch progress.
    ws.on('message', (raw) => {
        let msg;
        try { msg = JSON.parse(raw); } catch { return; }
        if (msg.type === 'log') console.log(`[robot] ${msg.msg}`);
        else if (msg.type === 'motor_cal_progress') {
            console.log(`[cal] step ${msg.step}/${msg.total}: ${msg.description}`);
        }
    });

    await sleep(400);

    console.log('[cal] Triggering auto-calibration (runs 6 PWM steps over ~15s)');
    send(ws, { type: 'start_motor_cal' });

    const result = await awaitMessage(ws,
        (m) => m.type === 'motor_cal_result',
        CAL_TIMEOUT_MS,
        'motor_cal_result');

    if (!result.success) {
        console.error('[cal] Calibration FAILED:', result);
        process.exit(1);
    }

    const fmt = (arr) => arr.map(v => v.toFixed(4)).join(', ');
    console.log(`[cal] Success. New gains:`);
    console.log(`        fwd: [${fmt(result.gainsFwd)}]`);
    console.log(`        rev: [${fmt(result.gainsRev)}]`);

    // Persist to NVS so the next boot uses them.
    console.log('[cal] Saving to NVS...');
    send(ws, { type: 'save_motor_cal' });
    const ack = await awaitMessage(ws,
        (m) => m.type === 'ack' && m.cmd === 'save_motor_cal',
        SAVE_TIMEOUT_MS,
        'save_motor_cal ack');

    if (ack.success) {
        console.log('[cal] Saved. Gains will auto-load on next boot.');
    } else {
        console.error('[cal] Save FAILED:', ack);
        process.exit(1);
    }

    ws.close();
}

main().catch(err => {
    console.error('[cal] FAILED:', err);
    process.exit(1);
});
