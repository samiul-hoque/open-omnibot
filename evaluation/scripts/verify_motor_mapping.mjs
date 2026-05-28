#!/usr/bin/env node
// Motor / encoder mapping verification.
//
// Runs each of the four motor_test indices (0..3, which the dashboard
// labels L1/R1/R2/L2 externally) one at a time, captures the sensor
// broadcast's enc[] deltas, and confirms that the motor commanded at
// position i produces motion on enc[i]. This is the automated version
// of the "jog and observe" diagnostic — cheaper to rerun and leaves
// a transcript that can be pasted into a session log.
//
// Requires: robot reachable at ROBOT_IP (default robot.local),
// wheels free to spin (bench stand), server NOT running (otherwise its
// 100 ms velocity keep-alive competes with this script's motor_test
// commands).
//
// Run: ROBOT_IP=robot.local node evaluation/scripts/verify_motor_mapping.mjs

import WebSocket from 'ws';

const ROBOT_IP = process.env.ROBOT_IP || 'robot.local';
const WS_URL = `ws://${ROBOT_IP}/ws`;
const PWM = 120;          // magnitude; safe on bench stand at this level
const SPIN_MS = 1500;     // motor-on duration per test
const COOLDOWN_MS = 800;  // between-tests settle
const KEEPALIVE_MS = 200; // re-issue motor_test to keep firmware watchdog happy

const MOTOR_LABELS = ['L1', 'R1', 'R2', 'L2']; // external wire order

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

async function main() {
  console.log(`Connecting to ${WS_URL}...`);
  const ws = new WebSocket(WS_URL);

  const latest = { enc: [0, 0, 0, 0], gotSample: false };

  ws.on('message', (raw) => {
    try {
      const msg = JSON.parse(raw.toString());
      if (msg.type === 'sensors' && Array.isArray(msg.enc) && msg.enc.length === 4) {
        latest.enc = msg.enc;
        latest.gotSample = true;
      }
    } catch { /* ignore */ }
  });

  await new Promise((resolve, reject) => {
    ws.once('open', resolve);
    ws.once('error', reject);
    setTimeout(() => reject(new Error('connect timeout')), 5000);
  });
  console.log('Connected.');

  // Wait for first sensor sample so the baseline is real, not [0,0,0,0]
  const waitStart = Date.now();
  while (!latest.gotSample && Date.now() - waitStart < 2000) {
    await sleep(50);
  }
  if (!latest.gotSample) {
    console.error('ERROR: no sensor broadcast received in 2s — is the robot powered up and connected?');
    ws.close();
    process.exit(2);
  }
  console.log(`Initial enc[]: [${latest.enc.join(', ')}]\n`);

  const results = [];

  for (let i = 0; i < 4; i++) {
    const label = MOTOR_LABELS[i];
    const startEnc = [...latest.enc];

    // Start motor i and keep-alive the firmware calibration-mode watchdog
    const sendCmd = () => ws.send(JSON.stringify({ type: 'motor_test', motor: i, pwm: PWM }));
    sendCmd();
    const keepAlive = setInterval(sendCmd, KEEPALIVE_MS);
    await sleep(SPIN_MS);
    clearInterval(keepAlive);

    // Stop and let the system settle
    ws.send(JSON.stringify({ type: 'motor_test', motor: i, pwm: 0 }));
    await sleep(COOLDOWN_MS);

    const endEnc = [...latest.enc];
    const deltas = endEnc.map((v, j) => v - startEnc[j]);
    const absDeltas = deltas.map(Math.abs);
    const maxIdx = absDeltas.indexOf(Math.max(...absDeltas));
    const pass = (maxIdx === i) && (absDeltas[i] > 100);

    results.push({ requested: i, label, deltas, absDeltas, maxIdx, pass });

    console.log(`Test ${i + 1}/4 — motor_test motor=${i} (UI label=${label})`);
    console.log(`  Δ enc[]: [${deltas.map((d) => String(d).padStart(7)).join(', ')}]`);
    console.log(`  Largest motion at enc[${maxIdx}] (${MOTOR_LABELS[maxIdx]})`);
    console.log(`  ${pass ? 'PASS' : 'FAIL'}` +
      (pass ? '' : ` — expected enc[${i}], got enc[${maxIdx}]`));
    console.log('');
  }

  // Final safety stop
  ws.send(JSON.stringify({ type: 'stop' }));
  await sleep(200);
  ws.close();

  const allPass = results.every((r) => r.pass);
  console.log('=== Summary ===');
  for (const r of results) {
    const mark = r.pass ? '\u2713' : '\u2717';
    const note = r.pass ? '' : ` (motion at ${MOTOR_LABELS[r.maxIdx]})`;
    console.log(`  ${r.label.padEnd(3)} (motor=${r.requested}): ${mark}${note}`);
  }
  console.log('');
  console.log(allPass
    ? 'ALL PASS — motor_test index → enc[] position mapping is consistent.'
    : 'FAILURES DETECTED — mapping is still off for at least one motor.');

  process.exit(allPass ? 0 : 1);
}

main().catch((err) => {
  console.error('FATAL:', err.message);
  process.exit(1);
});
