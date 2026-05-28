# Server — Claude Code Context

## What This Is

Node.js control server for the Omni-2 robot. Connects to the ESP32 over WebSocket, runs multi-tier localization, serves a browser dashboard, and logs sensor data to CSV.

## Quick Start

```bash
npm install
npm start        # production
npm run dev      # auto-reload
```

Browser dashboard at `http://localhost:3000`. Calibration/diagnostics at `http://localhost:3000/calibration.html`. Keyboard control in terminal (WASD + QE).

## Local Config

Create `src/config.local.js` (gitignored) to override defaults:

```js
export const localConfig = {
    robot: { ip: '192.168.68.xxx' }
};
```

All defaults are in `src/config.js`. Deep-merged at startup.

## Architecture

```
index.js                 — Main entry, event wiring, keyboard control
├── robot/robotClient.js — WebSocket client to ESP32 (:80/ws)
├── localization/
│   ├── odometry.js      — Tier 1: dead reckoning (encoders only)
│   └── fusionBasic.js   — Tier 2: complementary filter (encoders + IMU)
├── logging/dataLogger.js — CSV logger to logs/
└── web/
    ├── webServer.js      — HTTP + WebSocket server for browser UI
    └── public/
        ├── index.html    — Main control dashboard (joystick, 2D map)
        └── calibration.html — Motor diagnostics, IMU cal, WiFi, latency
```

## Robot Protocol

**Server sends to robot:**
```json
{"type": "cmd", "vx": 0.2, "vy": 0.0, "w": 0.1}
{"type": "stop"}
{"type": "reset_encoders"}
{"type": "zero_imu"}
{"type": "motor_test", "motor": 0, "pwm": 128}
{"type": "get_info"}
{"type": "save_imu_cal"}
{"type": "load_imu_cal"}
{"type": "start_motor_cal"}
{"type": "save_motor_cal"}
{"type": "load_motor_cal"}
{"type": "set_motor_gains", "gainsFwd": [1,1,1,1], "gainsRev": [1,1,1,1]}
{"type": "set_heading_hold", "enabled": true, "gain": 1.0, "deadzone": 0.03, "alpha": 0.3}
{"type": "ping_cal", "ts": 123, "ts_server_fwd": 456}
```

The robot responds to `set_heading_hold` with a `{"type":"ack","cmd":"set_heading_hold",...}` that `webServer.broadcastAck()` forwards to all browsers so the dashboard checkbox can sync.

**Robot sends to server (50Hz):**
```json
{
  "type": "sensors",
  "t": 123456,
  "enc": [100, -100, -100, 100],
  "vel": [1.2, -1.2, -1.2, 1.2],
  "imu": {"yaw": 45.0, "pitch": 0.0, "roll": 0.0, "gz": 0.01, "ax": 0.0, "ay": 0.0},
  "cal": {"sys": 3, "gyro": 3, "accel": 3, "mag": 3}
}
```

Coordinates: vx = forward, vy = left, w = CCW.

## Physical Constants (must match firmware)

- Wheel radius: 0.04 m
- Half track width (Lx): 0.1175 m, half wheelbase (Ly): 0.0953 m
- Encoder: 1092 counts/wheel rev
- **Wire index order (READ CAREFULLY — this has bitten us multiple
  times)**: `msg.enc` / `msg.vel` arrive from the robot in EXTERNAL
  WIRE order `[L1, R1, R2, L2]`. The firmware internally uses
  `[L1, R1, L2, R2]` and permutes positions 2↔3 on the way out
  (`websocket_server.cpp:609/622`). **The server converts wire→internal
  exactly once** via `mapEncoders()` in `localization/mecanumKinematics.js`,
  and all downstream server code (encoderDeltas, FK, per-wheel config
  arrays in `config.physical.*`) works in INTERNAL order. Do not
  re-permute in `encoderDeltas` or `mecanumFK`, and do not index
  `encoderSigns`/`motorGainsFwd`/`motorGainsRev` against raw wire
  data. See the convention block at the top of `mecanumKinematics.js`.

## Kinematics sign note

The mecanum FK `vy` formula was wrong in firmware and both odometry
modules until 2026-04-15 — it had the sign of `vy` inverted relative
to the IK. Driving was unaffected (the motor path is IK-only), but
odometry integrated strafe on the wrong axis. Fixed in
`mecanum.cpp:64`, `odometry.js:96`, and `fusionBasic.js:112`. If you
touch any of those three formulas, keep them in sync:

```
dx = (r/4)     * ( L1 + R1 + R2 + L2)
dy = (r/4)     * ( L1 - R1 + R2 - L2)   ← positive = body left
dθ = (r/(4L)) * (-L1 + R1 + R2 - L2)   ← positive = CCW
```

## Motor-gain application (2026-04-18 — DO NOT REVERT)

Per-motor gains (`motorGainsFwd/Rev`) compensate per-motor PWM→speed
variance. They live on the **feedforward PWM** side, not on the
measurement side:

- Firmware `pid_controller.cpp`: multiplies TARGET by gain before
  `wheelSpeedToPWM()`. The PID loop closes on TRUE wheel rad/s.
- Firmware `sensors.cpp` (`readEncoders`, `readOdomEncoders`): returns
  TRUE rad/s with no gain applied.
- Server `mecanumKinematics.js` (`encoderDeltas`): sign correction only,
  no gain. Odometry must use true wheel motion — multiplying by gain
  here distorts FK integration.

Applying gain to the measurement side was the old scheme. It caused
22 cm/11° left-drift per 2 m of straight-line motion because it forced
`true_speed = target/gain` at steady state, creating asymmetric true
wheel speeds from symmetric targets. If you see a bug where per-wheel
calibration seems to make the robot worse instead of better, check
whether something slipped the gain back onto the measurement side.

## Localization Tiers

- **Tier 1** — Pure odometry. Drifts intentionally (thesis baseline).
- **Tier 2** — Complementary filter. Position from encoders, heading from IMU. Weight configurable (`imuWeight`, default 0.98).

All tier switching goes through `switchTier(tier)` in `index.js` — a single function that updates config, swaps the localization instance, logs a `tier_change` event to CSV, and syncs the webServer. Called by keyboard (1/2) and web UI alike.

## Ground-truth ingestion

Ground truth is captured **per run**, not streamed. The experiment
runner's `awaiting_ground_truth` phase accepts a single
`(xMeas, yMeas, thetaDegMeas)` from the operator — either typed in or
auto-filled from a snapshot of the overhead ArUco rig (see
`../evaluation/ground_truth/`) — and stores it under
`experiment.groundTruth` in the run's `meta.json` sidecar.

No GT values are written to CSV rows.

## Calibration Page

`calibration.html` is a standalone diagnostics page with:
- **Motor cards**: individual PWM sliders, run/stop, encoder + velocity display, lockup detection (|PWM|>50 but encoder stalled for 500ms)
- **Uniformity test**: run all 4 motors at same PWM, bar chart of velocity deviation, color-coded (green <10%, yellow 10-25%, red >25%)
- **IMU panel**: live yaw/pitch/roll/gyro/accel, calibration indicators (0-3 color-coded), contextual guidance text, zero/save/load buttons
- **WiFi panel**: SSID, RSSI bar + sparkline
- **Latency & Clock Sync**: NTP sync status for all 3 layers, clock offset estimation (NTP-style from roundtrip), per-hop breakdown (browser↔server↔robot), sensor pipeline latency, sparklines
- **Safety**: 500ms firmware timeout, E-STOP (button + spacebar), beforeunload stop, calibration mode prevents PID interference

Server proxies `motor_test`, `get_info`, `save_imu_cal`, `load_imu_cal`, `start_motor_cal`, `save_motor_cal`, `load_motor_cal`, `set_motor_gains`, `ping_cal` between browser and robot. `broadcastRobotInfo()`, `broadcastPongCal()`, and `broadcastMotorCalResult()` relay responses. Per-hop timestamps (`ts_server_fwd`, `ts_server_ret`) added for latency measurement. Sensor state includes `robotUtc` and `serverReceivedAt` for pipeline latency. Motor gains from `get_info` are stored in `config.physical.motorGainsFwd`/`motorGainsRev` and applied in all odometry tiers.

## Known TODOs

- `odometry.js` / `fusionBasic.js`: Extract shared mecanum kinematics module

## Deferred Work

- **UWB + tier-3 EKF**: removed from the runtime on 2026-04-14. The DWM1001 UWB reader, EKF fusion, anchor positioning, and related WebSocket/UI surfaces were partial implementations. Planned to return once the encoder-only and complementary-filter tiers are validated per the thesis experimental protocol. Re-introducing it will require: a fresh UWB reader, full EKF Jacobian, anchor calibration UI, and updates to the tier selector and logging columns.

## Testing

```bash
npm test
```

Tests cover localization tiers and web server.

## Key Rates

- Robot sensor stream: 20 Hz (firmware broadcastSensorData interval 50 ms)
- Motor command refresh: 10 Hz (100 ms keepalive in index.js)
- Web broadcast: 10 Hz
- Robot reconnect: exponential backoff 3s → 30s cap

## Data Logging

CSVs go to `logs/robot_data_tier{N}_{timestamp}.csv` (~40 columns). Started on robot connect, stopped on disconnect.

## Code Style

ES modules throughout. ESLint configured. Node.js conventions.
