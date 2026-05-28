// ============================================
// Trajectory catalog
// ============================================
//
// Each trajectory builder returns a single flat `segments` array that
// mixes motion segments (`translate` / `yaw` / `strafe_circle`) with
// explicit `pause` markers. The server uploads the whole list to the
// firmware once; the firmware halts on every pause segment and waits
// for `traj_resume` before continuing. The `waypoints` side-channel
// maps each pause's segment index back to its human-readable label
// (`wp1_0m5`, `wp3_corner_2`, …) so the server can attach the right
// label to each snapshot without the firmware knowing about labels at
// all.
//
// Segment kinds the firmware understands:
//   translate:      { kind: 'translate', vx, vy, w: 0, distance }
//   yaw:            { kind: 'yaw',       w, angle }
//   strafe_circle:  { kind: 'strafe_circle', speed, radius }
//                   (firmware hardcodes a full revolution; partial
//                    arcs are polygonised into translate segments,
//                    see quarterArcSegments below)
//   pause:          { kind: 'pause' }   ← zero-duration halt marker
//
// Trajectory builders take a `speed` parameter (m/s for translations,
// rad/s-independent for yaw) and return:
//   {
//     segments:  [...flat list, including pause markers...],
//     waypoints: [{ segmentIdx, label }, ...],  // one per pause
//   }
//
// World placement (camera usable workspace):
//   - X ∈ [−1.30, +1.10] m, Y ∈ [−0.50, +0.60] m. Origin is NOT
//     geometrically centred — the grid was rebuilt 2026-04-19 and the
//     painted origin ended up offset (−0.10, +0.05) from the grid's
//     geometric mid-point. X is the long axis (2.40 m total range).
//   - Each trajectory carries a `startHint` describing where the
//     operator must physically place the robot before clicking Start.
//     Odometry is zeroed on arm, so body-frame segments don't depend
//     on start pose — the hint is advisory UX only.

const YAW_SPEED = 0.5; // rad/s for in-place corner turns (~3.14 s per 90°)

// ---------------------------------------------------------------------------
// strafe_circle approximation — polygon of translate segments
// ---------------------------------------------------------------------------
// Firmware's strafe_circle is hardcoded to a full revolution, so partial
// arcs are approximated with N short translate segments whose velocity
// vector points along the tangent at each step's midpoint.
//
// Geometry: polygon vertices land *exactly* on the true circle at each
// segment boundary (double-angle identity on the chord + midpoint
// tangent direction). For r=0.5 m, N=8 per quarter, max chord-arc
// deviation is <1 mm.
const CIRCLE_SEGS_PER_QUARTER = 8;

function quarterArcSegments(speed, radius, startAngleRad) {
    const dTheta = (Math.PI / 2) / CIRCLE_SEGS_PER_QUARTER;
    const chord  = 2 * radius * Math.sin(dTheta / 2);
    const segs = [];
    for (let i = 0; i < CIRCLE_SEGS_PER_QUARTER; i++) {
        const midTheta = startAngleRad + (i + 0.5) * dTheta;
        segs.push({
            kind: 'translate',
            vx: speed * Math.sin(midTheta),
            vy: speed * Math.cos(midTheta),
            w: 0,
            distance: chord,
        });
    }
    return segs;
}

// ---------------------------------------------------------------------------
// Builder helper: take a list of motion-chunk + label pairs and flatten
// them into a single `segments` array with pause markers interleaved at
// the labelled boundaries. A null `label` means "no pause here" (used
// for trailing cleanup motions that shouldn't trigger a snapshot).
// ---------------------------------------------------------------------------
function buildWithPauses(chunks) {
    const segments = [];
    const waypoints = [];
    for (const chunk of chunks) {
        for (const s of chunk.segments) segments.push(s);
        if (chunk.label) {
            waypoints.push({ segmentIdx: segments.length, label: chunk.label });
            segments.push({ kind: 'pause' });
        }
    }
    return { segments, waypoints };
}

export const TRAJECTORIES = {
    straight_2m: {
        label: 'Straight 2 m',
        description: 'Forward 2 m along +x. Tests pure translation odometry. '
            + 'Snapshot at the start pose, then every 0.5 m waypoint (5 total).',
        startHint: {
            x: -1.0,
            y: 0.0,
            headingDeg: 0,
            text: 'Place the robot at world (−1.00, 0.00) facing +X (long axis). '
                + 'It will capture a start-pose snapshot, drive forward 2 m '
                + '(pausing for a snapshot every 0.5 m), and end at (+1.00, 0.00).',
        },
        build(speed) {
            const T = (dist) => ({ kind: 'translate', vx: speed, vy: 0, w: 0, distance: dist });
            // Leading empty chunk parks a pause as segment 0 so firmware
            // halts immediately on trajStart and we get a GT frame of the
            // true starting pose before any motion integrates drift.
            return buildWithPauses([
                { segments: [],       label: 'wp0_start' },
                { segments: [T(0.5)], label: 'wp1_0m5'   },
                { segments: [T(0.5)], label: 'wp2_1m0'   },
                { segments: [T(0.5)], label: 'wp3_1m5'   },
                { segments: [T(0.5)], label: 'wp4_2m0'   },
            ]);
        },
    },

    circle_0_5m_strafe: {
        label: 'Circle r=0.5 m (strafe)',
        description:
            'One full revolution of a 0.5 m radius circle with fixed heading. '
            + 'The robot strafes around the circle (no yaw). Snapshot at every '
            + '90° of arc (4 total). Each quarter is approximated as 8 short '
            + 'translate segments — see CIRCLE_SEGS_PER_QUARTER. Circle is '
            + 'centred at (0, +0.10) so the southern extreme (the start pose) '
            + 'sits 10 cm off the −Y edge of the post-2026-04-19 workspace; '
            + 'the northern extreme just kisses the +Y edge — acceptable but '
            + 'drift-sensitive, detection rate is already lowest near frame '
            + 'edges so use this variant as the primary closure-test.',
        startHint: {
            x: 0.0,
            y: -0.4,
            headingDeg: 90,
            text: 'Place the robot at world (0.00, −0.40) facing +Y. It will '
                + 'trace a 0.5 m radius circle centred at (0, +0.10), with 4 '
                + 'pauses every 90° of arc, and return to start.',
        },
        build(speed) {
            const r = 0.5;
            return buildWithPauses([
                { segments: quarterArcSegments(speed, r, 0),                  label: 'wp1_90deg'  },
                { segments: quarterArcSegments(speed, r, Math.PI / 2),        label: 'wp2_180deg' },
                { segments: quarterArcSegments(speed, r, Math.PI),            label: 'wp3_270deg' },
                { segments: quarterArcSegments(speed, r, 3 * Math.PI / 2),    label: 'wp4_360deg' },
            ]);
        },
    },

    yaw_roundtrip: {
        label: 'Yaw round-trip (±90° twice)',
        description:
            'Four consecutive in-place 90° yaws at YAW_SPEED: CCW, CW, CW, CCW. '
            + 'Exposes per-rotation sweep accuracy, CCW/CW directional asymmetry, '
            + 'return-to-start closure error, and any body translation under '
            + 'pure yaw from slip. Debug trajectory — not used for thesis.',
        startHint: {
            x: 0.0,
            y: 0.0,
            headingDeg: 0,
            text: 'Place the robot at world (0.00, 0.00) facing +X. It will '
                + 'capture a start-pose snapshot, then rotate in place: +90° '
                + 'CCW → back to 0° → −90° CW → back to 0°. Five snapshots.',
        },
        // Speed param is unused — yaws rotate at YAW_SPEED and ignore `speed`.
        // The signature is kept uniform with other trajectories so the runner's
        // speed-wrap wiring stays the same.
        build(_speed) {
            const Y_ccw = { kind: 'yaw', w: YAW_SPEED, angle:  Math.PI / 2 };
            const Y_cw  = { kind: 'yaw', w: YAW_SPEED, angle: -Math.PI / 2 };
            return buildWithPauses([
                { segments: [],      label: 'wp0_start'    },
                { segments: [Y_ccw], label: 'wp1_left90'   },
                { segments: [Y_cw],  label: 'wp2_back'     },
                { segments: [Y_cw],  label: 'wp3_right90'  },
                { segments: [Y_ccw], label: 'wp4_back2'    },
            ]);
        },
    },

    square_0m8_rotate: {
        label: 'Square 0.8 m (rotate at corners)',
        description:
            'Four 0.8 m straight segments separated by in-place 90° yaws. '
            + 'Snapshot at the start pose, at every mid-side, at every corner '
            + 'pre-yaw, and at every post-yaw heading (13 total). Square is '
            + 'placed with corners at (±0.4, −0.35) and (±0.4, +0.45) so it '
            + 'sits at the Y centroid of the post-2026-04-19 workspace and '
            + 'keeps 15 cm margin on every edge — replaces square_1m_rotate, '
            + 'which filled the Y axis and gave the robot no room to drift.',
        startHint: {
            x: 0.4,
            y: -0.35,
            headingDeg: 90,
            text: 'Place the robot at world (+0.40, −0.35) facing +Y. It will '
                + 'trace a 0.8 m square with corners at (±0.4, −0.35) and '
                + '(±0.4, +0.45), rotating 90° CCW at each corner, with 13 '
                + 'snapshots (initial, then mid-side / corner / post-yaw for '
                + 'each of the 4 sides).',
        },
        build(speed) {
            const T_half = { kind: 'translate', vx: speed, vy: 0, w: 0, distance: 0.4 };
            const Y      = { kind: 'yaw',       w: YAW_SPEED, angle: Math.PI / 2 };
            // Leading empty chunk parks a pause as segment 0 so firmware
            // halts immediately on trajStart — captures the true start pose
            // before any motion integrates drift. Trailing yaw is now also
            // labelled so `wp12_yawed_4` should read as the same pose as
            // `wp00_initial` if odometry + actuation were perfect, giving a
            // direct closure-error signal.
            return buildWithPauses([
                { segments: [],              label: 'wp00_initial'  },
                { segments: [T_half],        label: 'wp01_mid_1'    },
                { segments: [T_half],        label: 'wp02_corner_1' },
                { segments: [Y],             label: 'wp03_yawed_1'  },
                { segments: [T_half],        label: 'wp04_mid_2'    },
                { segments: [T_half],        label: 'wp05_corner_2' },
                { segments: [Y],             label: 'wp06_yawed_2'  },
                { segments: [T_half],        label: 'wp07_mid_3'    },
                { segments: [T_half],        label: 'wp08_corner_3' },
                { segments: [Y],             label: 'wp09_yawed_3'  },
                { segments: [T_half],        label: 'wp10_mid_4'    },
                { segments: [T_half],        label: 'wp11_corner_4' },
                { segments: [Y],             label: 'wp12_yawed_4'  },
            ]);
        },
    },

    square_0m8_strafe: {
        label: 'Square 0.8 m (holonomic strafe)',
        description:
            'Forward, strafe right, backward, strafe left — traces a 0.8 m '
            + 'square without rotating. Snapshot at the start pose, at every '
            + 'mid-side, and at every corner (9 total). Square is placed with '
            + 'corners at (±0.4, −0.35) and (±0.4, +0.45) so it sits at the Y '
            + 'centroid of the post-2026-04-19 workspace and keeps 15 cm '
            + 'margin on every edge — replaces square_1m_strafe.',
        startHint: {
            x: -0.4,
            y: -0.35,
            headingDeg: 90,
            text: 'Place the robot at world (−0.40, −0.35) facing +Y. It will '
                + 'trace a 0.8 m square holonomically (no rotation: fwd → '
                + 'right → back → left), with corners at (±0.4, −0.35) and '
                + '(±0.4, +0.45), and 9 snapshots (initial + mid-side + '
                + 'corner × 4).',
        },
        build(speed) {
            const fwd   = { kind: 'translate', vx:  speed, vy:  0,      w: 0, distance: 0.4 };
            const right = { kind: 'translate', vx:  0,     vy: -speed,  w: 0, distance: 0.4 };
            const back  = { kind: 'translate', vx: -speed, vy:  0,      w: 0, distance: 0.4 };
            const left  = { kind: 'translate', vx:  0,     vy:  speed,  w: 0, distance: 0.4 };
            // Leading empty chunk captures the true start pose (same pattern
            // as straight_2m and square_0m8_rotate). `wp08_corner_4` returns
            // to the start position — compare against `wp00_initial` for the
            // closure-error signal.
            return buildWithPauses([
                { segments: [],      label: 'wp00_initial'  },
                { segments: [fwd],   label: 'wp01_mid_1'    },
                { segments: [fwd],   label: 'wp02_corner_1' },
                { segments: [right], label: 'wp03_mid_2'    },
                { segments: [right], label: 'wp04_corner_2' },
                { segments: [back],  label: 'wp05_mid_3'    },
                { segments: [back],  label: 'wp06_corner_3' },
                { segments: [left],  label: 'wp07_mid_4'    },
                { segments: [left],  label: 'wp08_corner_4' },
            ]);
        },
    },
};

// Compute a segment's total duration (seconds) at the given speed.
// Pauses have no time budget — they hold until trajResume().
export function segmentDurationMs(seg) {
    switch (seg.kind) {
    case 'translate': {
        const v = Math.hypot(seg.vx, seg.vy);
        if (v <= 0) return 0;
        return (seg.distance / v) * 1000;
    }
    case 'yaw': {
        if (seg.w === 0) return 0;
        return (Math.abs(seg.angle) / Math.abs(seg.w)) * 1000;
    }
    case 'strafe_circle': {
        if (seg.speed <= 0) return 0;
        return (2 * Math.PI * seg.radius / seg.speed) * 1000;
    }
    case 'pause':
    default:
        return 0;
    }
}

// Sample a segment's body-frame velocity at elapsed time t (ms since
// the segment started). Returns { vx, vy, w }. Callers should stop the
// segment once t exceeds segmentDurationMs().
export function segmentVelocityAt(seg, elapsedMs) {
    switch (seg.kind) {
    case 'translate':
        return { vx: seg.vx, vy: seg.vy, w: 0 };
    case 'yaw':
        return { vx: 0, vy: 0, w: Math.sign(seg.angle) * Math.abs(seg.w) };
    case 'strafe_circle': {
        const omega = seg.speed / seg.radius;
        const theta = omega * (elapsedMs / 1000);
        return {
            vx: seg.speed * Math.sin(theta),
            vy: seg.speed * Math.cos(theta),
            w: 0,
        };
    }
    case 'pause':
    default:
        return { vx: 0, vy: 0, w: 0 };
    }
}

// Total motion duration of a built trajectory (pause segments contribute 0).
export function trajectoryDurationMs(built) {
    return built.segments.reduce((sum, seg) => sum + segmentDurationMs(seg), 0);
}

// Return a new built trajectory with all `pause` segments removed and the
// waypoint sidecar emptied. Used by the experiment runner's demo mode: the
// robot runs the motion end-to-end without halting for snapshots. Works on
// any trajectory shape, so future builders (including custom ones) opt in
// for free — no per-builder changes needed.
export function stripPauses(built) {
    const segments = built.segments.filter((s) => s.kind !== 'pause');
    return { segments, waypoints: [] };
}

// Trajectory catalog summary for UI consumption (dynamic dropdown).
// `demoOnly: true` marks trajectories that should only appear when the
// operator has demo mode enabled — useful for scratch/custom paths that
// shouldn't pollute the thesis set.
export function listTrajectories() {
    return Object.entries(TRAJECTORIES).map(([key, t]) => ({
        key,
        label: t.label,
        demoOnly: t.demoOnly === true,
    }));
}
