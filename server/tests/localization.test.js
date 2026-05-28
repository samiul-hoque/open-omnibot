// ============================================
// Localization Unit Tests
// ============================================

import { describe, it, beforeEach } from 'node:test';
import assert from 'node:assert';
import { Odometry } from '../src/localization/odometry.js';
import { FusionBasic } from '../src/localization/fusionBasic.js';

// ============================================
// Test Helpers
// ============================================

function createSensorData(encoders, imu = null) {
    return {
        timestamp: Date.now(),
        encoders: encoders,
        velocities: [0, 0, 0, 0],
        imu: imu || { yaw: 0, pitch: 0, roll: 0, gyroZ: 0, accelX: 0, accelY: 0 },
        calibration: { sys: 3, gyro: 3, accel: 3, mag: 3 },
    };
}

// Approximate equality for floating point
function assertApprox(actual, expected, tolerance = 0.001, message = '') {
    assert.ok(
        Math.abs(actual - expected) < tolerance,
        `${message} Expected ${expected}, got ${actual} (tolerance: ${tolerance})`,
    );
}

// ============================================
// Odometry Tests
// ============================================

describe('Odometry', () => {
    let odom;

    beforeEach(() => {
        odom = new Odometry();
    });

    describe('initialization', () => {
        it('should start at origin', () => {
            const pose = odom.getPose();
            assert.strictEqual(pose.x, 0);
            assert.strictEqual(pose.y, 0);
            assert.strictEqual(pose.theta, 0);
            assert.strictEqual(pose.tier, 1);
        });
    });

    describe('reset', () => {
        it('should reset to origin by default', () => {
            odom.x = 1;
            odom.y = 2;
            odom.theta = 1.5;
            odom.reset();

            const pose = odom.getPose();
            assert.strictEqual(pose.x, 0);
            assert.strictEqual(pose.y, 0);
            assert.strictEqual(pose.theta, 0);
        });

        it('should reset to specified pose', () => {
            odom.reset(1, 2, Math.PI / 2);

            const pose = odom.getPose();
            assert.strictEqual(pose.x, 1);
            assert.strictEqual(pose.y, 2);
            assertApprox(pose.theta, Math.PI / 2);
        });
    });

    describe('normalizeAngle', () => {
        it('should keep angles in [-pi, pi] range', () => {
            assertApprox(odom.normalizeAngle(0), 0);
            assertApprox(odom.normalizeAngle(Math.PI), Math.PI);
            assertApprox(odom.normalizeAngle(-Math.PI), -Math.PI);
        });

        it('should wrap angles greater than pi', () => {
            assertApprox(odom.normalizeAngle(Math.PI + 0.1), -Math.PI + 0.1);
            assertApprox(odom.normalizeAngle(2 * Math.PI), 0);
            assertApprox(odom.normalizeAngle(3 * Math.PI), Math.PI);
        });

        it('should wrap angles less than -pi', () => {
            assertApprox(odom.normalizeAngle(-Math.PI - 0.1), Math.PI - 0.1);
            assertApprox(odom.normalizeAngle(-2 * Math.PI), 0);
        });
    });

    describe('update', () => {
        it('should return current pose on first update', () => {
            const data = createSensorData([0, 0, 0, 0]);
            const pose = odom.update(data);

            assert.strictEqual(pose.x, 0);
            assert.strictEqual(pose.y, 0);
            assert.strictEqual(pose.theta, 0);
        });

        it('should handle null/undefined data gracefully', () => {
            const pose1 = odom.update(null);
            const pose2 = odom.update(undefined);
            const pose3 = odom.update({});

            assert.strictEqual(pose1.x, 0);
            assert.strictEqual(pose2.x, 0);
            assert.strictEqual(pose3.x, 0);
        });

        it('should update position for forward movement', () => {
            // Initialize with zero encoders
            odom.update(createSensorData([0, 0, 0, 0]));

            // All wheels move forward by same amount (1092 counts = 1 wheel revolution)
            // With r=0.04m, circumference = 0.251m, so 1 rev = 0.251m forward
            const countsPerRev = 1092;
            odom.update(createSensorData([countsPerRev, countsPerRev, countsPerRev, countsPerRev]));

            const pose = odom.getPose();
            // Robot should have moved forward in x
            assert.ok(pose.x > 0, 'Robot should have moved forward');
            assertApprox(pose.y, 0, 0.01, 'Robot should not have moved laterally');
            assertApprox(pose.theta, 0, 0.01, 'Robot should not have rotated');
        });

        it('should track total distance', () => {
            odom.update(createSensorData([0, 0, 0, 0]));
            odom.update(createSensorData([100, 100, 100, 100]));

            const pose = odom.getPose();
            assert.ok(pose.totalDistance > 0, 'Should track distance traveled');
        });

        // Regression guard for the FK sign flip fixed 2026-04-15 in
        // odometry.js:96. For strafe-left (body +y), IK produces wheel
        // speeds [+, -, +, -] in [L1,R1,R2,L2] order, and FK must invert
        // to positive y. Wire index order matches the firmware broadcast.
        it('should integrate strafe-left as +y', () => {
            odom.update(createSensorData([0, 0, 0, 0]));
            // Strafe-left wheel pattern, one wheel rev magnitude.
            const n = 1092;
            odom.update(createSensorData([+n, -n, +n, -n]));
            const pose = odom.getPose();
            assert.ok(pose.y > 0, `Strafe-left should move +y, got y=${pose.y}`);
            assertApprox(pose.x, 0, 0.01, 'Pure strafe should not move x');
            assertApprox(pose.theta, 0, 0.01, 'Pure strafe should not rotate');
        });

        it('should integrate strafe-right as -y', () => {
            odom.update(createSensorData([0, 0, 0, 0]));
            const n = 1092;
            odom.update(createSensorData([-n, +n, -n, +n]));
            const pose = odom.getPose();
            assert.ok(pose.y < 0, `Strafe-right should move -y, got y=${pose.y}`);
            assertApprox(pose.x, 0, 0.01, 'Pure strafe should not move x');
        });

        it('should integrate CCW wheel pattern as +theta', () => {
            odom.update(createSensorData([0, 0, 0, 0]));
            // CCW rotation: left side backward, right side forward.
            // Wire order [L1,R1,R2,L2]: [-, +, +, -].
            const n = 200;
            odom.update(createSensorData([-n, +n, +n, -n]));
            const pose = odom.getPose();
            assert.ok(pose.theta > 0, `CCW pattern should give +theta, got ${pose.theta}`);
        });
    });
});

// ============================================
// FusionBasic Tests
// ============================================

describe('FusionBasic', () => {
    let fusion;

    beforeEach(() => {
        fusion = new FusionBasic();
    });

    describe('initialization', () => {
        it('should start at origin', () => {
            const pose = fusion.getPose();
            assert.strictEqual(pose.x, 0);
            assert.strictEqual(pose.y, 0);
            assert.strictEqual(pose.theta, 0);
            assert.strictEqual(pose.tier, 2);
            assert.strictEqual(pose.imuInitialized, false);
        });
    });

    describe('reset', () => {
        it('should reset pose and IMU state', () => {
            fusion.x = 1;
            fusion.y = 2;
            fusion.imuInitialized = true;
            fusion.reset();

            const pose = fusion.getPose();
            assert.strictEqual(pose.x, 0);
            assert.strictEqual(pose.y, 0);
            assert.strictEqual(pose.imuInitialized, false);
        });
    });

    describe('IMU initialization', () => {
        it('should initialize IMU offset on first reading with IMU data', () => {
            const data = createSensorData([0, 0, 0, 0], { yaw: 45, pitch: 0, roll: 0 });
            fusion.update(data);

            assert.strictEqual(fusion.imuInitialized, true);
        });

        it('should work without IMU data (falls back to odometry)', () => {
            const data = {
                timestamp: Date.now(),
                encoders: [0, 0, 0, 0],
                imu: null,
            };
            fusion.update(data);

            const pose = fusion.getPose();
            assert.strictEqual(pose.imuInitialized, false);
        });
    });

    describe('setImuWeight', () => {
        it('should clamp weight to [0, 1]', () => {
            fusion.setImuWeight(0.5);
            assert.strictEqual(fusion.imuWeight, 0.5);

            fusion.setImuWeight(-0.5);
            assert.strictEqual(fusion.imuWeight, 0);

            fusion.setImuWeight(1.5);
            assert.strictEqual(fusion.imuWeight, 1);
        });
    });

    describe('degToRad', () => {
        it('should convert degrees to radians', () => {
            assertApprox(fusion.degToRad(0), 0);
            assertApprox(fusion.degToRad(90), Math.PI / 2);
            assertApprox(fusion.degToRad(180), Math.PI);
            assertApprox(fusion.degToRad(-90), -Math.PI / 2);
        });
    });

    describe('normalizeAngle', () => {
        it('should normalize angles to [-pi, pi]', () => {
            assertApprox(fusion.normalizeAngle(0), 0);
            assertApprox(fusion.normalizeAngle(2 * Math.PI), 0);
            assertApprox(fusion.normalizeAngle(-2 * Math.PI), 0);
        });
    });

    describe('update with sensor fusion', () => {
        it('should fuse encoder and IMU data', () => {
            // First reading initializes
            fusion.update(createSensorData([0, 0, 0, 0], { yaw: 0 }));

            // Second reading with IMU showing rotation
            const data = createSensorData([100, 100, 100, 100], { yaw: 10 });
            fusion.update(data);

            const pose = fusion.getPose();
            // With high IMU weight (0.98), heading should be close to IMU reading
            assert.ok(pose.imuInitialized, 'IMU should be initialized');
        });
    });
});
