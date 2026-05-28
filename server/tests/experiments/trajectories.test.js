// ============================================
// Trajectory catalog unit tests
// ============================================

import { describe, it } from 'node:test';
import assert from 'node:assert';

import {
    TRAJECTORIES,
    segmentDurationMs,
    segmentVelocityAt,
    trajectoryDurationMs,
    stripPauses,
    listTrajectories,
} from '../../src/experiments/trajectories.js';

// Helper: all non-pause motion segments in order.
function motionSegs(built) {
    return built.segments.filter(s => s.kind !== 'pause');
}

// Helper: pause indices in the flat segments array.
function pauseIndices(built) {
    const out = [];
    built.segments.forEach((s, i) => { if (s.kind === 'pause') out.push(i); });
    return out;
}

describe('trajectory catalog', () => {
    it('exposes the expected trajectories', () => {
        assert.deepStrictEqual(
            Object.keys(TRAJECTORIES).sort(),
            ['circle_0_5m_strafe', 'square_0m8_rotate', 'square_0m8_strafe', 'straight_2m', 'yaw_roundtrip'],
        );
    });

    it('straight_2m: leading start pause + 4 translate + 4 pause segments, 5 labels', () => {
        const t = TRAJECTORIES.straight_2m.build(0.4);
        // Leading pause (wp0_start) + 4 motion/pause pairs = 9 segments.
        assert.strictEqual(t.segments.length, 9);
        const motion = motionSegs(t);
        assert.strictEqual(motion.length, 4);
        for (const s of motion) {
            assert.deepStrictEqual(s, {
                kind: 'translate', vx: 0.4, vy: 0, w: 0, distance: 0.5,
            });
        }
        // Pauses at indices 0 (start), 2, 4, 6, 8.
        assert.deepStrictEqual(pauseIndices(t), [0, 2, 4, 6, 8]);
        // 5 waypoints, first one at the start pose before any motion.
        assert.strictEqual(t.waypoints.length, 5);
        assert.strictEqual(t.waypoints[0].label, 'wp0_start');
        assert.strictEqual(t.waypoints[0].segmentIdx, 0);
        for (const wp of t.waypoints) {
            assert.strictEqual(t.segments[wp.segmentIdx].kind, 'pause');
            assert.ok(typeof wp.label === 'string' && wp.label.length > 0);
        }
        // Motion duration only (pauses contribute 0). 2 m at 0.4 m/s = 5000 ms.
        assert.strictEqual(trajectoryDurationMs(t), 5000);
    });

    it('square_0m8_rotate: 13 labelled waypoints (initial + mid/corner/yawed × 4)', () => {
        const t = TRAJECTORIES.square_0m8_rotate.build(0.4);
        // 8 half-sides + 4 corner yaws = 12 motion segments.
        // 1 leading pause (wp00_initial) + 12 interleaved pauses = 13 pauses.
        // 12 + 13 = 25 total segments.
        assert.strictEqual(t.segments.length, 25);
        const motion = motionSegs(t);
        assert.strictEqual(motion.length, 12);
        assert.strictEqual(motion.filter(s => s.kind === 'translate').length, 8);
        assert.strictEqual(motion.filter(s => s.kind === 'yaw').length, 4);
        assert.strictEqual(pauseIndices(t).length, 13);
        assert.strictEqual(t.waypoints.length, 13);

        // First waypoint must be the pre-motion start snapshot.
        assert.strictEqual(t.waypoints[0].label, 'wp00_initial');
        assert.strictEqual(t.waypoints[0].segmentIdx, 0);
        // Final waypoint must be after the closing yaw so start vs end
        // closure error is directly comparable.
        assert.strictEqual(t.waypoints.at(-1).label, 'wp12_yawed_4');

        // 8 half-sides × 0.4 m / 0.4 m/s = 8000 ms.
        // 4 yaws × (π/2) / 0.5 rad/s ≈ 12 566 ms.
        const total = trajectoryDurationMs(t);
        const expected = 8 * 1000 + 4 * (Math.PI / 2 / 0.5) * 1000;
        assert.ok(Math.abs(total - expected) < 1, `total ${total} vs expected ${expected}`);
    });

    it('square_0m8_strafe: 9 labelled waypoints (initial + mid/corner × 4), no yaws', () => {
        const t = TRAJECTORIES.square_0m8_strafe.build(0.4);
        const motion = motionSegs(t);
        assert.strictEqual(motion.length, 8);
        for (const s of motion) {
            assert.strictEqual(s.kind, 'translate');
            assert.strictEqual(s.w, 0);
        }
        assert.strictEqual(pauseIndices(t).length, 9);
        assert.strictEqual(t.waypoints.length, 9);

        assert.strictEqual(t.waypoints[0].label, 'wp00_initial');
        assert.strictEqual(t.waypoints[0].segmentIdx, 0);
        assert.strictEqual(t.waypoints.at(-1).label, 'wp08_corner_4');

        assert.deepStrictEqual(motion.map(s => [s.vx, s.vy]), [
            [0.4, 0], [0.4, 0],
            [0, -0.4], [0, -0.4],
            [-0.4, 0], [-0.4, 0],
            [0, 0.4], [0, 0.4],
        ]);
    });

    it('circle_0_5m_strafe: polygonised arc with 4 pauses', () => {
        const t = TRAJECTORIES.circle_0_5m_strafe.build(0.4);
        const motion = motionSegs(t);
        // 8 segments per quarter × 4 quarters = 32 motion segments.
        assert.strictEqual(motion.length, 32);
        for (const s of motion) {
            assert.strictEqual(s.kind, 'translate');
            assert.strictEqual(s.w, 0);
        }
        assert.strictEqual(pauseIndices(t).length, 4);
        assert.strictEqual(t.waypoints.length, 4);

        // Motion duration ≈ 2π·r / speed regardless of polygon subdivision.
        const total = trajectoryDurationMs(t);
        const expected = 2 * Math.PI * 0.5 / 0.4 * 1000;
        assert.ok(Math.abs(total - expected) < 20, `total ${total} vs expected ${expected}`);
    });

    it('all trajectories keep waypoint.segmentIdx pointing at a pause segment', () => {
        for (const [name, def] of Object.entries(TRAJECTORIES)) {
            const t = def.build(0.3);
            for (const wp of t.waypoints) {
                assert.strictEqual(
                    t.segments[wp.segmentIdx]?.kind, 'pause',
                    `${name}: waypoint ${wp.label} idx ${wp.segmentIdx} is not a pause`,
                );
            }
        }
    });
});

describe('segmentVelocityAt', () => {
    it('returns constant velocity for translate segments', () => {
        const seg = { kind: 'translate', vx: 0.4, vy: 0, w: 0, distance: 2 };
        assert.deepStrictEqual(segmentVelocityAt(seg, 0), { vx: 0.4, vy: 0, w: 0 });
        assert.deepStrictEqual(segmentVelocityAt(seg, 2500), { vx: 0.4, vy: 0, w: 0 });
    });

    it('returns constant w for yaw segments, sign follows angle', () => {
        const seg = { kind: 'yaw', w: 0.5, angle: Math.PI / 2 };
        const v = segmentVelocityAt(seg, 500);
        assert.strictEqual(v.vx, 0);
        assert.strictEqual(v.vy, 0);
        assert.strictEqual(v.w, 0.5);

        const negSeg = { kind: 'yaw', w: 0.5, angle: -Math.PI / 2 };
        assert.strictEqual(segmentVelocityAt(negSeg, 500).w, -0.5);
    });

    it('rotates the velocity vector for strafe_circle', () => {
        const seg = { kind: 'strafe_circle', speed: 0.4, radius: 0.5 };
        const v0 = segmentVelocityAt(seg, 0);
        assert.ok(Math.abs(v0.vx) < 1e-9);
        assert.ok(Math.abs(v0.vy - 0.4) < 1e-9);
        assert.strictEqual(v0.w, 0);

        const tQuarter = (Math.PI / 2 / 0.8) * 1000;
        const v1 = segmentVelocityAt(seg, tQuarter);
        assert.ok(Math.abs(v1.vx - 0.4) < 1e-6);
        assert.ok(Math.abs(v1.vy) < 1e-6);
    });

    it('returns zero velocity for pause segments', () => {
        assert.deepStrictEqual(
            segmentVelocityAt({ kind: 'pause' }, 0),
            { vx: 0, vy: 0, w: 0 },
        );
        assert.deepStrictEqual(
            segmentVelocityAt({ kind: 'pause' }, 5000),
            { vx: 0, vy: 0, w: 0 },
        );
    });
});

describe('segmentDurationMs', () => {
    it('computes translate duration from speed magnitude (holonomic)', () => {
        const fwd = { kind: 'translate', vx: 0.4, vy: 0, w: 0, distance: 1 };
        assert.strictEqual(segmentDurationMs(fwd), 2500);
        const diag = { kind: 'translate', vx: 0.3, vy: 0.4, w: 0, distance: 1 };
        assert.strictEqual(segmentDurationMs(diag), 2000);
    });

    it('returns 0 for zero-velocity segments and pauses', () => {
        assert.strictEqual(segmentDurationMs({ kind: 'translate', vx: 0, vy: 0, w: 0, distance: 1 }), 0);
        assert.strictEqual(segmentDurationMs({ kind: 'yaw', w: 0, angle: 1 }), 0);
        assert.strictEqual(segmentDurationMs({ kind: 'strafe_circle', speed: 0, radius: 0.5 }), 0);
        assert.strictEqual(segmentDurationMs({ kind: 'pause' }), 0);
    });
});

describe('stripPauses', () => {
    it('removes all pause segments and clears waypoints', () => {
        const built = TRAJECTORIES.straight_2m.build(0.4);
        assert.ok(built.segments.some(s => s.kind === 'pause'), 'sanity: original has pauses');
        assert.ok(built.waypoints.length > 0, 'sanity: original has waypoints');

        const stripped = stripPauses(built);
        assert.strictEqual(stripped.segments.filter(s => s.kind === 'pause').length, 0);
        assert.deepStrictEqual(stripped.waypoints, []);
        // Original is not mutated.
        assert.ok(built.segments.some(s => s.kind === 'pause'));
        assert.ok(built.waypoints.length > 0);
    });

    it('preserves motion-segment order and total motion duration', () => {
        for (const key of Object.keys(TRAJECTORIES)) {
            const built = TRAJECTORIES[key].build(0.2);
            const stripped = stripPauses(built);
            const motion = built.segments.filter(s => s.kind !== 'pause');
            assert.deepStrictEqual(stripped.segments, motion, `${key}: motion order preserved`);
            assert.strictEqual(
                trajectoryDurationMs(stripped),
                trajectoryDurationMs(built),
                `${key}: pauses contribute 0 ms so durations match`,
            );
        }
    });
});

describe('listTrajectories', () => {
    it('returns {key, label, demoOnly} for every catalog entry', () => {
        const list = listTrajectories();
        assert.strictEqual(list.length, Object.keys(TRAJECTORIES).length);
        for (const entry of list) {
            assert.ok(typeof entry.key === 'string' && entry.key.length > 0);
            assert.ok(typeof entry.label === 'string' && entry.label.length > 0);
            assert.strictEqual(typeof entry.demoOnly, 'boolean');
            assert.ok(TRAJECTORIES[entry.key], `${entry.key} resolves in catalog`);
        }
    });
});
