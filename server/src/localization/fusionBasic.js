// ============================================
// Tier 2: Encoder + IMU Fusion
// ============================================
//
// Uses encoder odometry for position (x, y) but
// replaces/corrects heading (theta) with IMU yaw.
//
// This is a simple complementary filter approach:
// - Position: from wheel odometry
// - Heading: blend of odometry change and IMU absolute heading
//

import { config } from '../config.js';
import { mapEncoders, encoderDeltas, mecanumFK, bodyToWorld, normalizeAngle } from './mecanumKinematics.js';

export class FusionBasic {
    constructor() {
        // Current pose estimate
        this.x = 0;           // meters
        this.y = 0;           // meters
        this.theta = 0;       // radians (from fusion)

        // Previous values
        this.prevEncoders = null;
        this.prevTimestamp = null;
        this.prevImuYaw = null;

        // IMU yaw offset (to align IMU frame with odometry frame)
        this.imuYawOffset = 0;
        this.imuInitialized = false;

        // Complementary filter weight
        // Higher = trust IMU more, Lower = trust odometry more
        this.imuWeight = config.fusion?.imuWeight ?? 0.98;

        // For tracking
        this.totalDistance = 0;
    }

    // Reset pose to origin.
    //
    // Opens a 300 ms suppression window — see odometry.js reset() for
    // the full explanation of why this is needed (stale encoder race).
    reset(x = 0, y = 0, theta = 0) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.prevEncoders = null;
        this.prevTimestamp = null;
        this.prevImuYaw = null;
        this.imuInitialized = false;
        this.totalDistance = 0;
        this._suppressUntil = Date.now() + 300;
    }

    // Update pose based on new sensor data
    update(sensorData) {
        if (!sensorData || !sensorData.encoders) {
            return this.getPose();
        }

        const encoders = mapEncoders(sensorData.encoders);
        const timestamp = sensorData.timestamp;
        const imuYawDeg = sensorData.imu?.yaw ?? null;

        // Suppression window: see odometry.js for the full explanation.
        if (this._suppressUntil && Date.now() < this._suppressUntil) {
            this.prevEncoders = [encoders[0], encoders[1], encoders[2], encoders[3]];
            this.prevTimestamp = timestamp;
            return this.getPose();
        }
        this._suppressUntil = null;

        // First reading - initialize.
        if (this.prevEncoders === null) {
            this.prevEncoders = [encoders[0], encoders[1], encoders[2], encoders[3]];
            this.prevTimestamp = timestamp;

            // Initialize IMU offset if available
            if (imuYawDeg !== null) {
                this.imuYawOffset = this.degToRad(imuYawDeg) - this.theta;
                this.imuInitialized = true;
            }

            return this.getPose();
        }

        // Encoder deltas → wheel displacements → body-frame motion
        const d = encoderDeltas(this.prevEncoders, encoders);
        const body = mecanumFK(d);

        // ----- Heading Fusion -----
        let newTheta;

        if (imuYawDeg !== null && this.imuInitialized) {
            // Get IMU heading in our reference frame
            const imuYawRad = this.degToRad(imuYawDeg) - this.imuYawOffset;
            const imuYawNorm = normalizeAngle(imuYawRad);

            // Odometry-predicted heading
            const odomTheta = normalizeAngle(this.theta + body.dtheta);

            // Complementary filter
            const angleDiff = normalizeAngle(imuYawNorm - odomTheta);
            newTheta = normalizeAngle(odomTheta + this.imuWeight * angleDiff);
        } else {
            // No IMU available, use pure odometry
            newTheta = normalizeAngle(this.theta + body.dtheta);

            // Try to initialize IMU if it becomes available
            if (imuYawDeg !== null && Number.isFinite(imuYawDeg) && !this.imuInitialized) {
                this.imuYawOffset = this.degToRad(imuYawDeg) - newTheta;
                this.imuInitialized = true;
            }
        }

        // ----- Position Update -----
        // Use fused heading for position transformation.
        // Normalize the heading delta so the midpoint rotation stays correct
        // when heading crosses ±π (raw diff would be ~±2π; true delta ~0).
        const headingDelta = normalizeAngle(newTheta - this.theta);
        const world = bodyToWorld(body.dx, body.dy, this.theta, headingDelta);

        // Update pose
        this.x += world.dx;
        this.y += world.dy;
        this.theta = newTheta;

        // Track total distance
        this.totalDistance += Math.sqrt(world.dx * world.dx + world.dy * world.dy);

        // Store for next iteration — in-place to avoid per-frame allocation.
        this.prevEncoders[0] = encoders[0];
        this.prevEncoders[1] = encoders[1];
        this.prevEncoders[2] = encoders[2];
        this.prevEncoders[3] = encoders[3];
        this.prevTimestamp = timestamp;
        this.prevImuYaw = imuYawDeg;

        return this.getPose();
    }

    getPose() {
        return {
            x: this.x,
            y: this.y,
            theta: this.theta,
            thetaDeg: this.theta * 180 / Math.PI,
            totalDistance: this.totalDistance,
            imuInitialized: this.imuInitialized,
            tier: 2,
        };
    }

    setPosition(x, y) {
        this.x = x;
        this.y = y;
    }

    setImuWeight(weight) {
        this.imuWeight = Math.max(0, Math.min(1, weight));
    }

    resetImuOffset() {
        this.imuYawOffset = 0;
        this.imuInitialized = false;
    }

    degToRad(deg) {
        return deg * Math.PI / 180;
    }

    normalizeAngle(angle) {
        return normalizeAngle(angle);
    }
}
