// ============================================
// Robot Server Configuration
// ============================================
// To override values locally (e.g. robot IP), create src/config.local.js:
//   export const localConfig = { robot: { ip: '192.168.68.xxx' } };
// That file is gitignored and merges on top of these defaults.
// ============================================

import { existsSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __dirname = dirname(fileURLToPath(import.meta.url));

export const config = {
    // Robot connection
    robot: {
        ip: '192.168.68.1',  // Default — override in config.local.js
        wsPort: 80,
        wsPath: '/ws',
        reconnectInterval: 3000,  // ms
    },

    // Robot physical parameters (must match firmware)
    physical: {
        wheelRadius: 0.04,        // meters (80mm diameter)
        lx: 0.1175,               // meters (half track width)
        ly: 0.0953,               // meters (half wheelbase)
        countsPerWheelRev: 1092,  // encoder counts per wheel revolution
        // ────────────────────────────────────────────────────────────
        // INDEX ORDER — READ BEFORE EDITING ANY PER-WHEEL ARRAY BELOW
        // ────────────────────────────────────────────────────────────
        // Every per-wheel array on this object is indexed in FIRMWARE
        // INTERNAL ORDER: [L1, R1, L2, R2] = [rear-left, rear-right,
        // front-left, front-right]. The robot broadcasts encoders in
        // EXTERNAL WIRE ORDER [L1, R1, R2, L2]; `mapEncoders()` in
        // localization/mecanumKinematics.js converts wire→internal via
        // `encoderMapping` BEFORE any of these arrays are indexed. If
        // you add a new per-wheel config, keep it in internal order to
        // stay consistent with `encoderSigns`, `motorGainsFwd/Rev`, and
        // the firmware's `motorDirs`.
        // ────────────────────────────────────────────────────────────

        // Encoder sign correction in INTERNAL order [L1, R1, L2, R2].
        // Set to -1 for any motor mounted with reversed encoder wiring
        // so that all four read positive when the robot drives forward.
        encoderSigns: [1, 1, 1, 1],
        // Maps EXTERNAL wire-order broadcasts [L1, R1, R2, L2] into
        // INTERNAL order [L1, R1, L2, R2]. This is the ONE place the
        // wire→internal permutation is applied on the server.
        encoderMapping: [0, 1, 3, 2],
        // Per-motor per-direction feedforward gain factors in INTERNAL
        // order [L1, R1, L2, R2]. Updated from robot on connect. Gains
        // are applied on the FIRMWARE'S feedforward PWM side (see
        // firmware/pid_controller.cpp); the server stores them for
        // diagnostics/UI only — do NOT multiply them into odometry.
        motorGainsFwd: [1.0, 1.0, 1.0, 1.0],
        motorGainsRev: [1.0, 1.0, 1.0, 1.0],
    },

    // Control loop timing
    timing: {
        controlLoopInterval: 20,   // ms (50Hz)
        logInterval: 100,          // ms (10Hz logging)
        statusInterval: 1000,      // ms (1Hz status print)
    },

    // Data logging
    logging: {
        enabled: true,
        directory: './logs',
        prefix: 'robot_data',
        idleSuppress: true,  // drop to 1 Hz when robot stationary for >2s
    },

    // Localization tier selection
    // 1 = Encoders only
    // 2 = Complementary filter (IMU+encoder)
    localizationTier: 1,

    // Keyboard control parameters
    control: {
        speed: 0.60,              // m/s for forward/lateral movement
        turnSpeed: 0.30,          // rad/s for rotation
        keyReleaseTimeout: 300,   // ms - stop if no key received (increased for smoother control)
    },

    // Sensor fusion parameters
    fusion: {
        imuWeight: 0.98,          // Complementary filter weight (0-1, higher = trust IMU more)
    },

    // Web UI server
    web: {
        enabled: true,
        port: 3000,
        broadcastInterval: 100,   // ms (10Hz sensor broadcast to browsers)
    },
};

// Load local overrides (gitignored)
const localPath = join(__dirname, 'config.local.js');
if (existsSync(localPath)) {
    const { localConfig } = await import('./config.local.js');
    function merge(target, source) {
        for (const key of Object.keys(source)) {
            if (source[key] && typeof source[key] === 'object' && !Array.isArray(source[key])) {
                if (!target[key]) target[key] = {};
                merge(target[key], source[key]);
            } else {
                target[key] = source[key];
            }
        }
    }
    merge(config, localConfig);
}

// Derived values
config.physical.lSum = config.physical.lx + config.physical.ly;
config.physical.wheelCircumference = 2 * Math.PI * config.physical.wheelRadius;
config.physical.metersPerCount = config.physical.wheelCircumference / config.physical.countsPerWheelRev;
