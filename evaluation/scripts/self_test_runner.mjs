#!/usr/bin/env node
// Self-test runner — triggers firmware self-test suite, collects results,
// writes results.json, and exits with CI-compatible exit code.
//
// Usage:
//   ROBOT_IP=robot.local node evaluation/scripts/self_test_runner.mjs
//   ROBOT_IP=robot.local node evaluation/scripts/self_test_runner.mjs motor_dir encoder_sanity

import WebSocket from 'ws';
import { mkdirSync, writeFileSync } from 'fs';

const ROBOT_IP = process.env.ROBOT_IP || 'robot.local';
const ROBOT_URL = `ws://${ROBOT_IP}:80/ws`;
const SUITE_TIMEOUT_MS = 120_000;

function send(ws, obj) { ws.send(JSON.stringify(obj)); }

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
    // Parse optional test names from CLI args
    const testNames = process.argv.slice(2).filter(a => !a.startsWith('-'));

    console.log(`Connecting to robot at ${ROBOT_URL}...`);
    const ws = new WebSocket(ROBOT_URL);

    await new Promise((resolve, reject) => {
        ws.on('open', resolve);
        ws.on('error', reject);
        setTimeout(() => reject(new Error('Connection timeout')), 5000);
    });
    console.log('Connected.');

    // Collect all results and logs
    const results = [];
    const logs = [];

    ws.on('message', (raw) => {
        let msg;
        try { msg = JSON.parse(raw); } catch { return; }
        if (msg.type === 'self_test_result') {
            results.push(msg);
            const icon = msg.pass ? '\u2713' : '\u2717';
            console.log(`  ${icon} ${msg.test}: ${msg.pass ? 'PASS' : 'FAIL'}`);
        }
        if (msg.type === 'log') {
            logs.push(msg.msg);
            console.log(`  LOG: ${msg.msg}`);
        }
    });

    // Start self-test suite
    const startMsg = { type: 'start_self_test' };
    if (testNames.length > 0) startMsg.tests = testNames;

    console.log(`\nStarting self-test suite${testNames.length ? ` (${testNames.join(', ')})` : ' (all tests)'}...`);
    send(ws, startMsg);

    // Wait for suite completion
    const complete = await awaitMessage(
        ws,
        (msg) => msg.type === 'self_test_complete',
        SUITE_TIMEOUT_MS,
        'self_test_complete',
    );

    console.log(`\n${'='.repeat(50)}`);
    console.log(`Suite: ${complete.pass ? 'ALL PASS' : 'SOME FAILED'}`);
    console.log(`  ${complete.tests_passed}/${complete.tests_run} passed in ${complete.duration_ms}ms`);
    console.log(`${'='.repeat(50)}`);

    // Save results
    const ts = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const outDir = `evaluation/experiments/self_test_${ts}`;
    mkdirSync(outDir, { recursive: true });

    const report = {
        timestamp: new Date().toISOString(),
        robotIp: ROBOT_IP,
        suite: complete,
        results,
        logs,
    };
    writeFileSync(`${outDir}/results.json`, JSON.stringify(report, null, 2) + '\n');
    console.log(`\nResults saved to ${outDir}/results.json`);

    ws.close();
    process.exit(complete.pass ? 0 : 1);
}

main().catch((err) => {
    console.error('Fatal:', err.message);
    process.exit(1);
});
