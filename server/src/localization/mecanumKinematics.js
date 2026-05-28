// ============================================
// Shared Mecanum Forward Kinematics
// ============================================
//
// Single source of truth for the FK formula used by all localization
// tiers.  Kept in sync with the firmware IK in mecanum.cpp and the
// Python offline replay in evaluation/scripts/capture.py.
//
//   dx = (r/4)     × ( ωL1 + ωR1 + ωR2 + ωL2)        forward
//   dy = (r/4)     × ( ωL1 − ωR1 + ωR2 − ωL2)        left-positive
//   dθ = (r/(4·L)) × (−ωL1 + ωR1 + ωR2 − ωL2)        CCW-positive
//
// ────────────────────────────────────────────────────────────────────
// INDEX ORDER CONVENTION — CRITICAL, READ BEFORE EDITING
// ────────────────────────────────────────────────────────────────────
// Two orderings exist in this codebase, do not conflate them:
//
//   WIRE / EXTERNAL order:  [L1, R1, R2, L2]   ← what robot broadcasts
//   FIRMWARE INTERNAL order: [L1, R1, L2, R2]  ← what the FK math and
//                                                all per-wheel config
//                                                arrays expect
//
// `mapEncoders()` below is the ONE place the permutation is applied.
// Once a caller has run raw encoders through `mapEncoders()`, the
// result is in INTERNAL order and all `config.physical.*` per-wheel
// arrays (encoderSigns, motorGainsFwd/Rev) index 1:1 against it.
//
// DO NOT re-apply the wire permutation inside encoderDeltas or mecanumFK.
// DO NOT index config.physical.encoderSigns against raw wire-order data.
// Past bugs in this area: gain-on-measurement double-correction
// (2026-04-18), vy sign flip (2026-04-15) — a misread of the convention
// is the common thread.
// ────────────────────────────────────────────────────────────────────
//

import { config } from '../config.js';

// Validate encoderMapping at load time — a bad permutation (e.g.
// [0,1,2,2]) would silently duplicate one wheel's encoder and drop
// another, causing divergent odometry that's very hard to debug.
const _map = config.physical.encoderMapping || [0, 1, 2, 3];
if (_map.length !== 4 || new Set(_map).size !== 4 || _map.some(i => i < 0 || i > 3)) {
    throw new Error(`Invalid encoderMapping ${JSON.stringify(_map)} — must be a permutation of [0,1,2,3]`);
}

/**
 * Map raw encoder array from wire order to firmware-internal order,
 * using config.physical.encoderMapping.
 *
 * Wire order from robot: [L1, R1, R2, L2]
 * Internal order:        [L1, R1, L2, R2]  (mapping [0,1,3,2])
 */
export function mapEncoders(rawEncoders) {
    return [rawEncoders[_map[0]], rawEncoders[_map[1]], rawEncoders[_map[2]], rawEncoders[_map[3]]];
}

/**
 * Compute wheel displacements in metres from encoder count deltas.
 *
 * Applies sign correction only. Motor gain compensation is NOT applied
 * here — gains live on the firmware's feedforward PWM side (see
 * pid_controller.cpp). Odometry must use true wheel motion; applying
 * gain would distort the FK body-frame integration.
 *
 * Input and output are in firmware-internal order [L1, R1, L2, R2].
 *
 * @param {number[]} prevEncoders  Previous encoder counts [L1, R1, L2, R2]
 * @param {number[]} encoders      Current encoder counts  [L1, R1, L2, R2]
 * @returns {{ dL1: number, dR1: number, dL2: number, dR2: number }}
 */
export function encoderDeltas(prevEncoders, encoders) {
    const signs = config.physical.encoderSigns;
    const mpc = config.physical.metersPerCount;

    return {
        dL1: (encoders[0] - prevEncoders[0]) * signs[0] * mpc,
        dR1: (encoders[1] - prevEncoders[1]) * signs[1] * mpc,
        dL2: (encoders[2] - prevEncoders[2]) * signs[2] * mpc,
        dR2: (encoders[3] - prevEncoders[3]) * signs[3] * mpc,
    };
}

/**
 * Mecanum forward kinematics: wheel displacements → body-frame motion.
 *
 * @param {{ dL1, dR1, dL2, dR2 }} d  Wheel displacements (metres)
 * @returns {{ dx: number, dy: number, dtheta: number }}  Body-frame deltas
 */
export function mecanumFK(d) {
    const r = config.physical.wheelRadius;
    const L = config.physical.lSum;

    const omegaL1 = d.dL1 / r;
    const omegaR1 = d.dR1 / r;
    const omegaR2 = d.dR2 / r;
    const omegaL2 = d.dL2 / r;

    const dx = (r / 4) * (omegaL1 + omegaR1 + omegaR2 + omegaL2);
    const dy = (r / 4) * (omegaL1 - omegaR1 + omegaR2 - omegaL2);
    const dtheta = (r / (4 * L)) * (-omegaL1 + omegaR1 + omegaR2 - omegaL2);

    return { dx, dy, dtheta };
}

/**
 * Transform body-frame displacement to world frame using midpoint heading.
 *
 * @param {number} dx_body   Body-frame forward displacement
 * @param {number} dy_body   Body-frame lateral displacement
 * @param {number} theta     Current world heading (rad)
 * @param {number} dtheta    Heading change this step (rad)
 * @returns {{ dx: number, dy: number }}  World-frame displacement
 */
export function bodyToWorld(dx_body, dy_body, theta, dtheta) {
    const mid = theta + dtheta / 2;
    const c = Math.cos(mid);
    const s = Math.sin(mid);
    return {
        dx: dx_body * c - dy_body * s,
        dy: dx_body * s + dy_body * c,
    };
}

/**
 * Normalize angle to [-π, π].
 */
export function normalizeAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
}
