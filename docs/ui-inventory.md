# Open Omnibot — UI & Interface Inventory

This document is a complete inventory of every user-facing interface, the controls within them, the data they display, and the backend functions they call. Use it as context for redesigning the user experience.

**Last updated:** 2026-03-26

---

## System Architecture (UI Perspective)

```
Browser (port 3000)                    Node.js Server                    ESP32 Firmware (port 80)
┌──────────────────┐                 ┌──────────────────┐              ┌──────────────────┐
│  index.html      │◄──── WS /ws ──►│  webServer.js    │◄── WS /ws ──►│ websocket_server │
│  calibration.html│                 │  robotClient.js  │              │ sensors/motors   │
│  joystick.js     │                 │  odometry.js     │              │ pid_controller   │
│  map.js          │                 │  fusionBasic.js   │              │ motor_calibration│
│                  │                 │  fusionEKF.js    │              │                  │
│                  │                 │  uwbReader.js    │              │ HTTP: / /update  │
│                  │                 │  dataLogger.js   │              │                  │
└──────────────────┘                 └──────────────────┘              └──────────────────┘
```

**Data flow:** Browser ↔ Server (WS on port 3000) ↔ Robot (WS on port 80). The server relays commands and broadcasts processed state at 10 Hz. The robot streams raw sensors at 20 Hz.

---

## 1. Main Dashboard (`/index.html`)

**Purpose:** Real-time robot control, trajectory visualization, and sensor monitoring.

### 1.1 Header

| Element | Type | Description |
|---------|------|-------------|
| Logo | Static | "Omni-2 Control" |
| Calibration link | Nav link | Opens `/calibration.html` |
| Connection indicator | Status dot | Green = connected, red = disconnected |
| Robot IP | Text + button | Shows current IP, "Change" opens IP modal |
| Tier selector | Dropdown | 1 (Encoders), 2 (Encoder+IMU), 3 (Full EKF) |
| IMU weight slider | Slider 0-100 | Only visible in Tier 2. Controls complementary filter weight |
| Demo Mode | Checkbox | Simulates robot locally without hardware |

**Backend calls:**
- Tier change → `{type: "setTier", tier: N}` → server updates `config.localizationTier`, calls `switchTier()`
- IMU weight → `{type: "setImuWeight", weight: 0.98}` → server calls `fusionBasic.setImuWeight()`
- Robot IP change → `{type: "setRobotIp", ip: "..."}` → server writes `config.local.js`, reconnects `robotClient`

### 1.2 Control Panel (Left Sidebar)

| Element | Type | Details | Message Sent |
|---------|------|---------|-------------|
| Virtual joystick | Canvas 180x180 | Touch/mouse, normalized ±1, 10 Hz emission | `{type: "cmd", vx, vy, w}` |
| Rotation slider | Range -100..100 | Auto-resets on release | Combined into `cmd.w` |
| Max Speed slider | Range 5-60 | Scales joystick output (0.05–0.60 m/s) | Local only |
| Max Turn slider | Range 5-60 | Scales rotation slider (0.05–0.60 rad/s) | Local only |
| STOP button | Button (red) | Clears all motion | `{type: "stop"}` |
| RESET button | Button | Resets pose to origin | `{type: "resetPose"}` |
| Zero IMU button | Button | Sets current heading as 0 | `{type: "zero_imu"}` |
| Init from UWB | Button | Sets pose from latest UWB position | `{type: "initPoseFromUwb"}` |

**Keyboard controls:** WASD (translate), QE (rotate), Space (stop). Commands sent at 100ms while held. Window blur = auto-stop.

**Server-side handling:**
- `cmd` → `robotClient.setVelocity(vx, vy, w)` → firmware PID loop
- `stop` → `robotClient.stop()` → firmware `stopAllMotors()`
- `resetPose` → `robotClient.resetEncoders()` + `localization.reset()`
- `zero_imu` → `robotClient.zeroImu()` + `localization.resetImuOffset()`
- `initPoseFromUwb` → reads `uwbReader.getLastPosition()`, calls `localization.reset(x, y)`

### 1.3 Map Panel (Center)

| Element | Type | Details |
|---------|------|---------|
| Position & trail map | Canvas, auto-sizing | 2D top-down view with grid, trail, robot icon |
| Orientation compass | Canvas 200x200 | Shows IMU yaw rotation |
| Zoom +/- buttons | Buttons | Scale ×1.5 per click (20–500 px/m range) |
| Clear Trail button | Button | Erases trajectory history |
| Center button | Button | Re-centers view on robot |
| UWB grid size | 2 number inputs + Apply | Sets anchor boundary rectangle (width, height in meters) |
| UWB calibration | 2 number inputs + Zero Here | XY offset to align UWB with odometry origin |

**Map renders (from `map.js` RobotMap class):**
- Grid: 10cm (fine) and 50cm (coarse) lines
- Origin: Red crosshair at (0,0)
- Robot: Green square with red heading arrow
- Trail: Cyan line (max 5000 points, >5mm movement threshold)
- UWB marker: Cyan circle with crosshair halo
- Anchor diamonds: Orange, with IDs and dashed boundary rectangle
- Scale bar: Bottom-right, adaptive (20cm/50cm/1m)
- Compass: Top-left with FWD arrow
- Legend: Top-right

**Data source:** All map data comes from the 10 Hz `state` broadcast — `pose.x`, `pose.y`, `pose.theta`, `uwb.position`.

### 1.4 Sensor Panel (Right Sidebar)

| Section | Fields | Source |
|---------|--------|--------|
| Pose | x (m), y (m), theta (deg) | `state.pose` (computed by server localization) |
| UWB | Status, position, quality, anchor distances | `state.uwb` from `uwbReader` |
| Encoders | L1, R1, R2, L2 (raw counts) | `state.sensors.enc` |
| Wheel Velocities | L1, R1, R2, L2 (rad/s) | `state.sensors.vel` |
| IMU | Yaw, pitch, roll (deg), gyro Z (deg/s), accel X/Y | `state.sensors.imu` |
| Calibration | Sys/Gyro, Accel/Mag (0-3) | `state.sensors.cal` |
| Command | vx, vy, omega | `state.command` |
| Logging | Row count | `state.logging.rowCount` |

**UWB controls:**
- Connect button → opens port selection modal → `{type: "listPorts"}` → `{type: "connectUwb", port: "..."}`
- Server uses `uwbReader.js` to read DWM1001 over serial

### 1.5 Debug Log Panel (Bottom)

- Max 200 lines, auto-scroll, clear button
- Sources: `[SYSTEM]`, `[ROBOT]`, `[UWB]`, `[UI]`, `[DEMO]`
- Color-coded: errors red, warnings orange, info cyan

### 1.6 Modals

| Modal | Trigger | Purpose |
|-------|---------|---------|
| Serial Port Selection | UWB Connect button | Lists available serial ports, click to connect |
| Robot IP Configuration | Header "Change" button | Text input with IP validation, triggers reconnect |

### 1.7 Demo Mode

Enabled via checkbox or `?demo=1` URL parameter. Generates simulated sensor data at 10 Hz locally in the browser — no server connection needed. Simulates encoders, IMU, UWB, kinematics with noise.

---

## 2. Calibration Page (`/calibration.html`)

**Purpose:** Hardware diagnostics, motor testing, IMU calibration, WiFi monitoring, latency measurement, motor gain calibration.

### 2.1 Header

| Element | Type | Description |
|---------|------|-------------|
| Logo | Static | "Omni-2 Calibration" |
| Dashboard link | Nav link | "← Dashboard" back to `/` |
| Connection indicator | Status dot | Same as main dashboard |
| E-STOP button | Button (red) | Stops all motors, sends `{type: "stop"}` |
| Calibration banner | Warning bar | "CALIBRATION MODE ACTIVE" — shown when any motor is running |

**E-STOP hotkey:** Spacebar (except when focused on input).

### 2.2 Robot Info Panel (Left Column)

| Field | Source |
|-------|--------|
| Firmware version | `robotInfo.firmware` |
| Uptime | `robotInfo.uptime` (formatted Xh Ym) |
| IP address | `robotInfo.ip` |
| MAC address | `robotInfo.mac` |
| Free heap | `robotInfo.freeHeap` (KB) |
| IMU status | `robotInfo.imuAvailable` → "Available" / "Not found" |

**Data source:** `get_info` polled every 2 seconds → firmware responds with system info → server relays as `robotInfo`.

**Firmware function:** `get_info` handler in `websocket_server.cpp:187` builds JSON with WiFi, NVS, heap, NTP status.

### 2.3 IMU Diagnostics Panel (Left Column)

| Element | Type | Details |
|---------|------|---------|
| Yaw/Pitch/Roll | Live display | From `state.sensors.imu`, 1-2 decimal places |
| Gyro Z | Live display | rad/s, 3 decimal places |
| Accel X/Y | Live display | m/s², 3 decimal places |
| Calibration indicators | 4 colored dots (S/G/A/M) | Level 0 (red) → 3 (green) |
| Guidance text | Dynamic | Auto-generated tips based on which components need calibration |
| Zero IMU | Button | `{type: "zero_imu"}` |
| Save Cal | Button | `{type: "save_imu_cal"}` |
| Load Cal | Button | `{type: "load_imu_cal"}` |

**Firmware functions:**
- `zero_imu` → `zeroIMU()` in `sensors.cpp` — stores current orientation as offset
- `save_imu_cal` → reads BNO055 offsets, writes to NVS namespace `"imu_cal"`
- `load_imu_cal` → reads NVS, calls `setIMUCalibrationOffsets()`
- `autoLoadIMUCalibration()` — called on boot, restores from NVS

### 2.4 WiFi Diagnostics Panel (Left Column)

| Element | Type | Details |
|---------|------|---------|
| SSID | Text | Network name + " (AP)" if fallback mode |
| RSSI | Text + bar | dBm value, color-coded bar (green > -60, yellow > -75, red) |
| RSSI sparkline | Canvas | 60-point history (30s at 2s polling interval) |
| Note | Static text | "WiFi credentials set at compile time" |

**Data source:** `robotInfo.ssid`, `robotInfo.rssi`, `robotInfo.apMode` from `get_info`.

### 2.5 Latency & Clock Sync Panel (Left Column)

| Element | Type | Details |
|---------|------|---------|
| Clock sync status | 3 indicators | Browser (always green), Server (green), Robot (green if NTP, orange if not) |
| Clock offsets | 2 values | Server vs browser (ms), Robot vs browser (ms) |
| Roundtrip latency | Large value + stats | Current, min/avg/max. Color: green <50ms, yellow <150ms, red |
| Roundtrip sparkline | Canvas | 60-point history with 50ms threshold line |
| Per-hop breakdown | 4 values | Browser→Server, Server→Robot, Robot→Server, Server→Browser |
| Sensor pipeline latency | Large value + stats | Robot sensor timestamp → browser receive. Uses NTP when available |
| Pipeline sparkline | Canvas | 60-point history, 200ms Y-axis |
| Auto Ping | Toggle button | Starts/stops 1 Hz ping_cal cycle |
| Ping Once | Button | Single ping_cal |
| Reset | Button | Clears all latency history and stats |

**Protocol:** `{type: "ping_cal", ts: browserTime}` → server adds `ts_server_fwd` → firmware adds `rt` (NTP UTC) → server adds `ts_server_ret` → browser computes per-hop deltas.

**Firmware function:** `ping_cal` handler echoes timestamps, adds `getUTCMillis()` if NTP synced.

### 2.6 Motor Diagnostics Panel (Right Column)

| Element | Type | Details |
|---------|------|---------|
| 4 motor cards | Per motor (L1, R1, R2, L2) | Each has PWM slider, Run/Stop toggle, encoder + velocity display |
| PWM slider | Range -255..255 | Per-motor, updates local state |
| Run/Stop button | Toggle | Starts 200ms interval sending `motor_test` |
| Encoder count | Live display | From `state.sensors.enc[i]` |
| Velocity | Live display | From `state.sensors.vel[i]` (rad/s) |
| Lockup warning | Conditional | Shows "POSSIBLE LOCKUP" if \|PWM\|>50 and encoder stalled >500ms |

**Uniformity test:**
| Element | Type | Details |
|---------|------|---------|
| PWM slider | Range -255..255 | Single slider for all 4 motors |
| Start/Stop | Buttons | Runs all motors (motor index 4), 200ms interval |
| Velocity bars | 4 horizontal bars | Deviation from mean. Green <10%, yellow <25%, red >25% |
| Summary | Text | "L1: 100% \| R1: 98% \| ..." |

**Stop All Motors:** Button at bottom — calls `emergencyStop()`.

**Firmware function:** `motor_test` handler in `websocket_server.cpp:168` — sets `calibrationMode = true`, calls `setMotorSpeed(motor, pwm)` directly (bypasses PID).

### 2.7 Motor Calibration Panel (Right Column)

| Element | Type | Details |
|---------|------|---------|
| Gain display | 8 values (4 motors × fwd/rev) | Grid layout. Color-coded: green <2% from 1.0, yellow <5%, red >5% |
| Progress bar | Animated bar + text | Shows step N/6 during calibration |
| Result display | Text box | Success message with gains, or failure message |
| Auto-Calibrate | Button | `{type: "start_motor_cal"}` — disables during run |
| Save | Button | `{type: "save_motor_cal"}` |
| Load | Button | `{type: "load_motor_cal"}` |
| Manual override | 8 number inputs (collapsible, 4×2 grid) | Step 0.001, range 0.8-1.2. Fwd + Rev per motor |
| Apply Manual | Button | `{type: "set_motor_gains", gainsFwd: [...], gainsRev: [...]}` |

**Firmware functions:**
- `start_motor_cal` → `startMotorCalibration()` in `motor_calibration.cpp` — state machine runs all motors at PWM [100, 160, 220] fwd+rev (6 steps). Each step: 400ms ramp-up (discarded) + 2000ms steady-state measurement. Computes separate forward and reverse gains as `group_mean / per_motor_count`. Broadcasts `motor_cal_result` with `gainsFwd` and `gainsRev` arrays.
- `save_motor_cal` → writes 8 gains to NVS namespace `"motor_cal"` (keys `gf_0..3`, `gr_0..3`)
- `load_motor_cal` → reads from NVS, applies via `setMotorGains(fwd, rev)`
- `set_motor_gains` → accepts `{gainsFwd: [4], gainsRev: [4]}`, applies directly
- `autoLoadMotorCalibration()` — called on boot, restores from NVS
- Gains applied in `sensors.cpp:readEncoders()` and `pid_controller.cpp:applyClosedLoopVelocity()` — forward gain when encoder delta >= 0, reverse gain when delta < 0

**Server-side:** Motor gains from `get_info` response stored in `config.physical.motorGainsFwd` and `config.physical.motorGainsRev`, applied directionally in `odometry.js`, `fusionBasic.js`, `fusionEKF.js` encoder delta calculations.

### 2.8 Debug Log Panel (Right Column)

Same behavior as main dashboard: max 200 lines, timestamped, color-coded (info=cyan, warn=yellow, error=red).

### 2.9 Safety Features

| Feature | Mechanism |
|---------|-----------|
| E-STOP button | Header, sends `stop` |
| Spacebar hotkey | Triggers E-STOP (skips if input focused) |
| Page unload | `beforeunload` handler sends `stop` |
| Firmware watchdog | 500ms timeout — motors stop if no command received |
| Calibration banner | Visual warning when PID is bypassed |

---

## 3. ESP32 Direct Endpoints (Port 80)

These are served by the robot itself, not the Node.js server.

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Status page: shows WebSocket URL, client count, firmware version, link to `/update` |
| `/update` | GET | Firmware upload form with JavaScript progress bar |
| `/update` | POST | Accepts `.bin` firmware upload (multipart). Stops motors, flashes, reboots |
| `/ws` | WebSocket | Raw sensor stream (20 Hz) and command interface |

**HTTP OTA usage:**
```bash
curl -F "firmware=@.pio/build/mhetesp32minikit/firmware.bin" http://robot.local/update
```

---

## 4. Complete WebSocket Protocol Reference

### 4.1 Browser → Server (port 3000 /ws)

| Message Type | Fields | Handler | Forwarded to Robot? |
|-------------|--------|---------|-------------------|
| `cmd` | `vx, vy, w` | `webServer.handleBrowserMessage` | Yes → `robotClient.setVelocity()` |
| `stop` | — | | Yes → `robotClient.stop()` |
| `resetPose` | — | | Yes (encoders) + server resets localization |
| `zero_imu` | — | | Yes + server resets IMU offset |
| `motor_test` | `motor (0-4), pwm` | | Yes → `robotClient.motorTest()` |
| `get_info` | — | | Yes → `robotClient.getInfo()` |
| `save_imu_cal` | — | | Yes |
| `load_imu_cal` | — | | Yes |
| `save_motor_cal` | — | | Yes |
| `load_motor_cal` | — | | Yes |
| `set_motor_gains` | `gainsFwd: [4], gainsRev: [4]` | | Yes |
| `start_motor_cal` | — | | Yes |
| `ping_cal` | `ts` | Server adds `ts_server_fwd` | Yes |
| `setTier` | `tier (1-3)` | Server-only | No |
| `setImuWeight` | `weight (0-1)` | Server-only | No |
| `initPoseFromUwb` | — | Server-only (reads UWB, resets pose) | No |
| `listPorts` | — | Server-only (serial port enumeration) | No |
| `connectUwb` | `port` | Server-only (opens serial) | No |
| `getRobotIp` | — | Server-only | No |
| `setRobotIp` | `ip` | Server-only (writes config, reconnects) | No |

### 4.2 Server → Browser (port 3000 /ws)

| Message Type | Frequency | Source | Contents |
|-------------|-----------|--------|----------|
| `state` | 10 Hz | Server (composed from robot sensors + localization) | `connected, robotIp, sensors{enc,vel,imu,cal,robotUtc}, pose{x,y,theta}, command{vx,vy,omega}, logging{rowCount}, config{tier,imuWeight}, uwb{...}` |
| `robotInfo` | On `get_info` response | Robot → server relay | `firmware, uptime, ip, mac, freeHeap, imuAvailable, ssid, rssi, apMode, ntpSynced, motorGainsFwd, motorGainsRev` |
| `robotLog` | On demand | Robot → server relay | `msg` (debug text) |
| `connectionStatus` | On connect/disconnect | Server | `connected: bool` |
| `pong_cal` | On ping response | Robot → server relay (adds `ts_server_ret`) | `ts, ts_server_fwd, rt, ntpSynced, ts_server_ret` |
| `motor_cal_result` | On calibration complete | Robot → server relay | `success, gainsFwd[4], gainsRev[4]` |
| `ack` | On command responses | Robot → server relay | `cmd, success, gains?` |
| `portList` | On `listPorts` request | Server | `ports[{path, manufacturer}]` |
| `uwbConnected` | On UWB serial open | Server | `port` |
| `uwbError` | On UWB error | Server | `error` |
| `initPoseResult` | On `initPoseFromUwb` | Server | `success, x, y` |
| `robotIp` | On `getRobotIp` | Server | `ip` |
| `robotIpChanged` | After IP change | Server (broadcast) | `ip` |
| `robotIpError` | On bad IP | Server | `error` |

### 4.3 Server → Robot (port 80 /ws)

| Message Type | Fields | Firmware Handler |
|-------------|--------|-----------------|
| `cmd` | `vx, vy, w` | Sets `cmdVx/Vy/Omega`, PID loop consumes at 50 Hz |
| `stop` | — | Clears commands, `stopAllMotors()` |
| `reset_encoders` | — | `resetAllEncoders()` |
| `zero_imu` | — | `zeroIMU()` |
| `ping` | — | Responds with `pong` |
| `ping_cal` | `ts, ts_server_fwd` | Responds with `pong_cal` + robot UTC timestamp |
| `motor_test` | `motor, pwm` | `setMotorSpeed()` directly, sets `calibrationMode=true` |
| `get_info` | — | Returns system info JSON |
| `save_imu_cal` | — | `getIMUCalibrationOffsets()` → NVS write |
| `load_imu_cal` | — | NVS read → `setIMUCalibrationOffsets()` |
| `save_motor_cal` | — | `getMotorGains()` → NVS write |
| `load_motor_cal` | — | NVS read → `setMotorGains()` |
| `set_motor_gains` | `gainsFwd[4], gainsRev[4]` | `setMotorGains(fwd, rev)` |
| `start_motor_cal` | — | `startMotorCalibration()` state machine |

### 4.4 Robot → Server (port 80 /ws)

| Message Type | Frequency | Contents |
|-------------|-----------|----------|
| `sensors` | 20 Hz | `t, enc[4], vel[4], imu{yaw,pitch,roll,gz,ax,ay}, cal{sys,gyro,accel,mag}, utc?` |
| `info` | On request | Full system info (firmware, uptime, IP, RSSI, heap, NTP, motorGainsFwd, motorGainsRev) |
| `pong` | On ping | — |
| `pong_cal` | On ping_cal | `ts, ts_server_fwd, rt?, ntpSynced` |
| `log` | On demand | `msg` (debug text) |
| `ack` | On save/load commands | `cmd, success, gains?` |
| `motor_cal_result` | On calibration complete | `success, gainsFwd[4], gainsRev[4]` |

---

## 5. Server-Side Processing (Non-UI)

These components run on the server and affect what the UI displays, but have no direct UI of their own.

| Component | File | What It Does | UI Impact |
|-----------|------|-------------|-----------|
| Odometry (Tier 1) | `odometry.js` | Dead reckoning from encoder deltas. Applies motor gains and encoder mapping. | `state.pose` |
| Fusion Basic (Tier 2) | `fusionBasic.js` | Complementary filter: position from encoders, heading from IMU. Weight configurable. | `state.pose` |
| Fusion EKF (Tier 3) | `fusionEKF.js` | Extended Kalman Filter with encoder + IMU + UWB. Partially implemented. | `state.pose` |
| UWB Reader | `uwbReader.js` | Reads DWM1001 over serial, parses position, computes anchor distances. | `state.uwb` |
| Data Logger | `dataLogger.js` | Writes CSV to `logs/` (~33 columns). Event column (col 33) for discrete events. JSON metadata sidecar (`.meta.json`). Idle suppression (1 Hz when stationary >2s). Empty strings for missing sensors. Motor test event deduplication. | `state.logging.rowCount` |
| Config | `config.js` | All defaults (robot IP, physical params, timing, localization tier). Merges `config.local.js`. | `state.config` |

---

## 6. NVS Persistence (Firmware Flash Storage)

| Namespace | Keys | Purpose | UI Trigger |
|-----------|------|---------|-----------|
| `imu_cal` | `valid` (bool), `offsets` (bytes) | BNO055 calibration offsets | Save/Load Cal buttons |
| `motor_cal` | `valid` (bool), `gf_0..3` (float), `gr_0..3` (float) | Per-motor per-direction encoder gain factors (8 values) | Save/Load in Motor Cal panel |

Both auto-load on boot via `autoLoadIMUCalibration()` and `autoLoadMotorCalibration()` in `setup()`.

---

## 7. Current User Journey

1. **Power on robot** → firmware connects to WiFi (or creates AP), starts WebSocket on :80, auto-loads calibration from NVS
2. **Start server** → `npm start` → connects to robot WebSocket, starts HTTP+WS on :3000, begins data logging
3. **Open dashboard** → browser connects to server WS, receives 10 Hz state broadcasts
4. **Drive robot** → joystick/keyboard → velocity commands → server → robot PID loop → motors
5. **Monitor** → live sensor data, 2D map trail, pose coordinates, UWB position
6. **Calibrate** → navigate to `/calibration.html` manually via link
7. **Test motors** → individual PWM sliders or uniformity test
8. **Calibrate IMU** → follow guidance text, save to flash when cal levels reach 3
9. **Calibrate motors** → Auto-Calibrate button, save gains to flash
10. **Check connectivity** → WiFi RSSI, latency pings, NTP sync status
11. **Flash new firmware** → HTTP OTA at robot's `/update` endpoint or via PlatformIO

---

## 8. Files Index

| File | Type | Purpose |
|------|------|---------|
| `server/src/web/public/index.html` | Browser UI | Main control dashboard |
| `server/src/web/public/calibration.html` | Browser UI | Calibration & diagnostics page |
| `server/src/web/public/joystick.js` | JS module | Virtual joystick component |
| `server/src/web/public/map.js` | JS module | 2D trajectory map component |
| `server/src/web/webServer.js` | Server | HTTP server + WS hub, message routing, state broadcast |
| `server/src/robot/robotClient.js` | Server | WebSocket client to ESP32, sensor parsing, command methods |
| `server/src/localization/odometry.js` | Server | Tier 1 dead reckoning |
| `server/src/localization/fusionBasic.js` | Server | Tier 2 encoder+IMU fusion |
| `server/src/localization/fusionEKF.js` | Server | Tier 3 EKF (partial) |
| `server/src/uwb/uwbReader.js` | Server | DWM1001 serial reader |
| `server/src/logging/dataLogger.js` | Server | CSV data logging |
| `server/src/config.js` | Server | Configuration defaults + local override |
| `server/src/index.js` | Server | Entry point, event wiring, keyboard control |
| `firmware/esp32-omni/src/websocket_server.cpp` | Firmware | WiFi, WS, NTP, OTA, message handlers, NVS |
| `firmware/esp32-omni/src/sensors.cpp` | Firmware | Encoder reading, IMU reading, motor gains |
| `firmware/esp32-omni/src/motors.cpp` | Firmware | PWM output via LEDC + MCP23017 |
| `firmware/esp32-omni/src/pid_controller.cpp` | Firmware | Feedforward + PID velocity control |
| `firmware/esp32-omni/src/mecanum.cpp` | Firmware | Inverse/forward kinematics |
| `firmware/esp32-omni/src/motor_calibration.cpp` | Firmware | Automated motor calibration state machine |
| `firmware/esp32-omni/src/main.cpp` | Firmware | Setup + main loop orchestration |
| `firmware/esp32-omni/src/config.h` | Firmware | All hardware constants and pin mappings |

---

## Keeping This Document Current

This file should be updated when any of the following change:

- UI panels, controls, or displays are added/removed/modified
- WebSocket message types are added or changed
- New HTTP endpoints are added
- Server-side processing components change (localization, logging)
- Firmware command handlers are added or modified
- NVS persistence namespaces change
- New HTML pages are added to `server/src/web/public/`

To check what has changed since this document was last updated, run:

```bash
# Changes to UI files
git log --oneline --since="2026-03-25" -- server/src/web/public/ server/src/web/webServer.js

# Changes to protocol/message handling
git log --oneline --since="2026-03-25" -- server/src/robot/robotClient.js firmware/esp32-omni/src/websocket_server.cpp

# Changes to server processing
git log --oneline --since="2026-03-25" -- server/src/localization/ server/src/index.js server/src/config.js

# All UI-relevant changes
git log --oneline --since="2026-03-25" -- server/ firmware/esp32-omni/src/
```

Replace the date with the "Last updated" date at the top of this file. If any of these show commits, read the changed files and update the relevant sections.
