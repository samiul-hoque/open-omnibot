// ============================================
// Tier 1: Dead Reckoning Odometry
// ============================================
//
// Uses only encoder data to estimate robot pose.
// This will drift over time - that's expected and
// important to show in your thesis.
//

import { mapEncoders, encoderDeltas, mecanumFK, bodyToWorld, normalizeAngle } from './mecanumKinematics.js';

export class Odometry {
    // `tier` is carried on every getPose() so the broadcast/state path
    // can report which tier produced a pose. Tier 0 (open-loop) and
    // tier 1 (dead reckoning) both construct Odometry, so the class
    // alone can't disambiguate — the caller passes it in.
    constructor(tier = 1) {
        this.tier = tier;

        // Current pose estimate
        this.x = 0;           // meters
        this.y = 0;           // meters
        this.theta = 0;       // radians

        // Previous encoder counts for delta calculation
        this.prevEncoders = null;
        this.prevTimestamp = null;

        // For tracking total distance traveled
        this.totalDistance = 0;
    }

    // Reset pose to origin (or specified pose).
    //
    // Opens a 300 ms suppression window during which update() keeps
    // re-snapshotting the encoder values instead of computing deltas.
    // This absorbs stale sensor packets that arrive before the robot's
    // reset_encoders command has been processed — without it, the first
    // post-reset delta is (0 − staleValue), which flings the pose to a
    // random location.
    reset(x = 0, y = 0, theta = 0) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.prevEncoders = null;
        this.prevTimestamp = null;
        this.totalDistance = 0;
        this._suppressUntil = Date.now() + 300;
    }

    // Update pose based on new sensor data
    // Returns the updated pose
    update(sensorData) {
        if (!sensorData || !sensorData.encoders) {
            return this.getPose();
        }

        const encoders = mapEncoders(sensorData.encoders);
        const timestamp = sensorData.timestamp;

        // Suppression window: after reset(), keep re-snapshotting encoder
        // values until the robot has had time to process reset_encoders.
        if (this._suppressUntil && Date.now() < this._suppressUntil) {
            this.prevEncoders = [encoders[0], encoders[1], encoders[2], encoders[3]];
            this.prevTimestamp = timestamp;
            return this.getPose();
        }
        this._suppressUntil = null;

        // First reading - just store and return.
        if (this.prevEncoders === null) {
            this.prevEncoders = [encoders[0], encoders[1], encoders[2], encoders[3]];
            this.prevTimestamp = timestamp;
            return this.getPose();
        }

        // Encoder deltas → wheel displacements → body-frame motion
        const d = encoderDeltas(this.prevEncoders, encoders);
        const body = mecanumFK(d);

        // Guard against NaN/Inf from invalid encoder data or config
        if (!Number.isFinite(body.dx) || !Number.isFinite(body.dy) || !Number.isFinite(body.dtheta)) {
            this.prevEncoders[0] = encoders[0];
            this.prevEncoders[1] = encoders[1];
            this.prevEncoders[2] = encoders[2];
            this.prevEncoders[3] = encoders[3];
            this.prevTimestamp = timestamp;
            return this.getPose();
        }

        // Transform to world frame
        const world = bodyToWorld(body.dx, body.dy, this.theta, body.dtheta);

        // Update pose
        this.x += world.dx;
        this.y += world.dy;
        this.theta = normalizeAngle(this.theta + body.dtheta);

        // Track total distance for stats
        this.totalDistance += Math.sqrt(world.dx * world.dx + world.dy * world.dy);

        // Store for next iteration — in-place to avoid per-frame allocation.
        this.prevEncoders[0] = encoders[0];
        this.prevEncoders[1] = encoders[1];
        this.prevEncoders[2] = encoders[2];
        this.prevEncoders[3] = encoders[3];
        this.prevTimestamp = timestamp;

        return this.getPose();
    }

    getPose() {
        return {
            x: this.x,
            y: this.y,
            theta: this.theta,
            thetaDeg: this.theta * 180 / Math.PI,
            totalDistance: this.totalDistance,
            tier: this.tier,
        };
    }

    setPosition(x, y) {
        this.x = x;
        this.y = y;
    }

    normalizeAngle(angle) {
        return normalizeAngle(angle);
    }
}
