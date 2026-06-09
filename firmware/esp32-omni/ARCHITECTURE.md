# ESP32-Omni Firmware — Claude Code Context

## What This Is

PlatformIO firmware for the Omni-2 mecanum-wheel robot. Runs on MH ET LIVE ESP32 MiniKit. Handles motor control, encoder/IMU reading, and WebSocket communication with the Node.js server.

## Quick Start

```bash
pio run                    # build
pio run -t upload          # flash via USB
pio run -t upload -e ota   # OTA upload (uses omni2.local)
pio run -t upload -e ota --upload-port 192.168.68.xxx  # OTA with explicit IP
pio test -e native         # run unit tests
```

### OTA Upload Methods

There are two OTA mechanisms — **ArduinoOTA** (PlatformIO integration) and **HTTP OTA** (browser/curl):

| Method | Command / URL | Firewall needed? |
|--------|--------------|-----------------|
| ArduinoOTA | `pio run -t upload -e ota` | Yes — ESP32 connects *back* to host on random TCP port |
| HTTP OTA | `curl -F "firmware=@.pio/build/mhetesp32minikit/firmware.bin" http://<robot-ip>/update` | No — push only, no callback |

**HTTP OTA** is the preferred method from WSL2 because it avoids the firewall callback issue entirely. Upload via browser at `http://<robot-ip>/update` or via curl.

**ArduinoOTA from WSL2** requires mirrored networking (`networkingMode=mirrored` in `~/.wslconfig`) **and** a Windows Firewall rule allowing inbound TCP 10000-60000 (the random port range espota.py uses). The Hyper-V firewall must also be disabled (`firewall=false` in `.wslconfig`). Without these, the ESP32 can't connect back through to WSL2.

## Local Config

Create `src/config.local.h` (gitignored) for WiFi credentials:

```cpp
#undef WIFI_SSID
#undef WIFI_PASSWORD

#define WIFI_SSID "your-network"
#define WIFI_PASSWORD "your-password"
```

All other config in `src/config.h`. Do NOT put credentials in config.h.

## Architecture

```
main.cpp              — setup(), loop(), timing orchestration
├── sensors.h/cpp     — PCNT quadrature encoders (x4), BNO055 IMU
├── motors.h/cpp      — LEDC PWM + MCP23017 direction control
├── mecanum.h/cpp     — Inverse/forward kinematics
├── pid_controller.h/cpp — Feedforward + per-wheel PID
├── motor_calibration.h/cpp — Automated per-motor gain calibration state machine
├── websocket_server.h/cpp — WiFi, AsyncWebSocket, JSON protocol, OTA
└── config.h          — All constants (pins, physical params, tuning)
```

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32 (MH ET LIVE MiniKit) |
| Motors | 4x DC with 42:1 gearbox |
| Encoders | 13 PPR, quadrature (1092 counts/wheel rev) |
| IMU | BNO055 @ I2C 0x29 (mounted upside-down) |
| GPIO Expander | MCP23017 @ I2C 0x20 (motor direction + standby) |
| Wheels | 80mm mecanum (radius 0.04m) |

## Motor/Encoder Mapping (internal firmware indices)

```
Index 0 = L1 (Rear Left)    — PWM:14, IN1/IN2: MCP7/6, EncA:35, EncB:34
Index 1 = R1 (Rear Right)   — PWM:25, IN1/IN2: MCP5/4, EncA:36, EncB:39
Index 2 = L2 (Front Left)   — PWM:26, IN1/IN2: MCP3/2, EncA:19, EncB:18
Index 3 = R2 (Front Right)  — PWM:27, IN1/IN2: MCP1/0, EncA:33, EncB:32
```

**External (wire/UI) index order is `[L1, R1, R2, L2]`** — different
from the internal order — so `msg.enc` / `msg.vel` place index 2 = R2
and index 3 = L2. This is handled in `websocket_server.cpp:609/622` by
permuting internal `counts[]` / `velocities[]` on output. The
calibration page's `MOTOR_NAMES` array follows the external order.

Chassis-side PWM harness is cross-routed on the front: GPIO 26 drives
the front-*left* motor and GPIO 27 drives the front-*right*, the
opposite of what the `MOTOR_L2_PWM=26`/`MOTOR_R2_PWM=27` naming
suggests. This was adjusted in `config.h` on 2026-04-15 (session notes
below) to make the software name match the physical wheel. The same
"swap" exists on the encoder side and was already corrected earlier in
commit `7b115fe`.

## Control Pipeline

```
WebSocket command {vx, vy, w}
  → IK → 4 target wheel speeds (rad/s)
  → Saturation (cap at MAX_WHEEL_SPEED=22 rad/s, scale proportionally)
  → Feedforward: target → base PWM  (PWM 255 = 22 rad/s per 2026-04-18 recal)
  → Rising-edge PID integrator reset on zero→non-zero command transition
  → (Optional) IMU heading-hold: if omega=0 & translating, override
    omega with filtered/dead-banded gain*gyro_z (default off)
  → Encoder delta → actual wheel velocity (gain-corrected per-direction)
  → PID correction per wheel (Kp=20, Ki=25, Kd=0.05)
  → Final PWM = feedforward + correction (clamped ±255)
  → Motor output via LEDC + MCP23017
```

**Axis convention:** `vx` = forward (+X, m/s), `vy` = left (+Y, m/s),
`omega` = CCW (rad/s). Commands go straight into the mecanum IK.

## WebSocket Protocol

**Robot broadcasts at 20Hz:**
```json
{
  "type": "sensors", "t": 123456,
  "enc": [L1, R1, R2, L2],
  "vel": [L1, R1, R2, L2],
  "imu": {"yaw": 45.0, "pitch": 0.0, "roll": 0.0, "gz": 0.01, "ax": 0.0, "ay": 0.0},
  "cal": {"sys": 3, "gyro": 3, "accel": 3, "mag": 3}
}
```

**Robot accepts:**
```json
{"type": "cmd", "vx": 0.2, "vy": 0.0, "w": 0.1}
{"type": "stop"}
{"type": "reset_encoders"}
{"type": "zero_imu"}
{"type": "ping"}
{"type": "motor_test", "motor": 0, "pwm": 128}
{"type": "get_info"}
{"type": "save_imu_cal"}
{"type": "load_imu_cal"}
{"type": "set_heading_hold", "enabled": true, "gain": 1.0, "deadzone": 0.03, "alpha": 0.3}
{"type": "ping_cal", "ts": 1234567890, "ts_server_fwd": 1234567890}
```

Motors auto-stop after 500ms with no command. Motors stop immediately on client disconnect.

### Calibration Mode

`motor_test` enters calibration mode which bypasses the PID loop for direct PWM control. Auto-clears after 500ms without a new `motor_test` command. Motor index 0-3 = individual, 4 = all.

### Motor Slew Rate Limiter

`setMotorSpeed()` ramps PWM by at most `MOTOR_MAX_PWM_STEP` (15) per call to protect gears from sudden direction changes. Full reversal takes ~340ms at 50Hz. `stopAllMotors()` bypasses the ramp for emergency stops.

### IMU Calibration Persistence

BNO055 calibration offsets are saved to ESP32 NVS (Preferences) via `save_imu_cal` and restored via `load_imu_cal`. Auto-loaded on boot if saved data exists (`autoLoadIMUCalibration()` called after `initIMU()`).

### Motor Gain Calibration

Per-motor per-direction encoder gain factors correct gearbox manufacturing variance and directional asymmetry. Each motor has separate forward and reverse gains (8 values total).

**Automated calibration** (`start_motor_cal`): State machine in `motor_calibration.cpp` runs all 4 motors at PWM [100, 160, 220] forward then reverse (6 steps). Each step: 400ms ramp-up (discarded from measurement) + 2000ms steady-state measurement. Computes `group_mean / per_motor_count` separately for forward and reverse steps.

**NVS persistence**: Namespace `"motor_cal"`, keys `gf_0..3` (forward) and `gr_0..3` (reverse). Auto-loaded on boot via `autoLoadMotorCalibration()`.

**Gain application (2026-04-18 — DO NOT REVERT)**: Gains are applied
**on the feedforward PWM side only**, never to encoder measurements.
`pid_controller.cpp:applyClosedLoopVelocity()` multiplies `target` by
the per-motor gain (chosen by target sign) before `wheelSpeedToPWM()`,
so a slower motor gets more FF PWM. `sensors.cpp:readEncoders()` and
`readOdomEncoders()` return TRUE rad/s with no gain applied — the PID
loop closes on true wheel speed, not gained units. Direction of the
gain in FF is picked by `target >= 0 ? gainsFwd : gainsRev`.

An earlier scheme applied the gain to the encoder measurement inside
the PID loop, which forced `true_speed = target/gain` at steady state
and produced systematic left-drift (22 cm per 2 m on this chassis).
If you find yourself moving gain back onto the measurement side to
"match what cal produced", you are re-introducing that bug — the cal
formula (`mean_counts / motor_counts`) is correct for FF compensation.

**Protocol**: `get_info` includes `motorGainsFwd` and `motorGainsRev` arrays. `set_motor_gains` accepts `{gainsFwd: [4], gainsRev: [4]}`.

## Timing

- Main loop: 1ms delay (watchdog-safe)
- Motor/PID update: 20ms (50Hz)
- Sensor broadcast: 50ms (20Hz)
- Velocity timeout: 500ms

## PID Tuning

- Kp=20.0, Ki=25.0, Kd=0.05
- Integral anti-windup: ±200 PWM
- Integral persists across stop/start (retains learned motor bias)
- Zero-target short circuit (no PID oscillation at standstill)

## Encoder Overflow Handling

PCNT counters are 16-bit. Overflow detection uses threshold at half range (16383). If delta exceeds threshold, accumulator adjusts by ±65536. Unit tests cover this in `test/test_overflow_logic/`.

## WiFi & NTP Behavior

1. Tries STA mode (credentials from config.local.h)
2. Falls back to AP mode ("Omni-Robot" / "12345678") if STA fails after 20 attempts
3. Power saving disabled (`WIFI_PS_NONE`) for stable WebSocket
4. ArduinoOTA enabled (hostname: "omni2", port 3232)
5. HTTP OTA enabled at `GET /update` (upload form) and `POST /update` (firmware binary)
6. SNTP sync after WiFi STA connects (pool.ntp.org, time.nist.gov) — waits up to 5s, continues in background if not ready
7. `getUTCMillis()` returns wall-clock UTC ms when synced, 0 otherwise
8. Sensor broadcasts include `"utc"` field when NTP is synced for cross-device latency measurement
9. `get_info` response includes `ntpSynced` and `ntpTime` fields

## HTTP Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Status page with WebSocket info and link to firmware update |
| `/ws` | WebSocket | Real-time sensor streaming and command interface |
| `/update` | GET | Firmware upload form (browser UI with progress bar) |
| `/update` | POST | Firmware upload handler (multipart `firmware` field, accepts `.bin`) |

HTTP OTA stops all motors on upload start, flashes via ESP32 `Update` library, and reboots on success. The reboot is deferred to the main loop so the HTTP response is sent first.

## Current Status

**Working (Tier 1 complete + Tier 2 stub):**
- Stable WebSocket with auto-reconnect handling.
- Per-wheel PID velocity control. Kp=20, Ki=25, Kd=0.05, integrator
  anti-windup at ±200 PWM. Rising-edge `resetPIDControllers()` on
  zero→non-zero command transitions kills cold-start overshoot.
- `MAX_WHEEL_SPEED = 22 rad/s` feedforward scaling (re-measured
  2026-04-18 via a 30-s PID hold at target 2.5 rad/s on a free-wheel
  stand: converged-steady-state PWM ≈ 29 → MAX = target × 255 / pwm =
  22 rad/s). The old 12 rad/s value was taken at PWM 255 (the top of
  the motor curve), which linearises terribly at typical operating
  targets 2–3 rad/s — FF demanded PWM 53 when the motor only needed
  PWM 29, forcing the integrator to claw back 24 PWM per wheel. That
  took >10 s to converge; segments are 3–5 s, so the loop never
  settled → chronic +5–7 % steady-state bias on straight drives and
  +10 % per 90° yaw. New value gives FF near the true operating PWM
  so the integrator makes tiny per-motor corrections and converges
  in ~580 ms. If you re-cal, DO IT AT THE OPERATING TARGET, not at
  full PWM. See `handover-2026-04-18-ff-recal.md`.
- Inter-motor velocity spread at PWM 160 (10-cycle bench campaign,
  re-measured 2026-04-15 after the front-motor pin fix): 6.66±1.62%
  with unity gains → 1.15±0.90% with calibrated gains (8.25× pairwise
  reduction — up from 3.60× pre-pin-fix because gains now attach to
  the correct motor/wheel pair). Full methodology and figure in
  the project's underlying undergraduate thesis.
- IMU heading-hold P controller (off by default) — when enabled and
  user commands pure translation, reads `gyro_z`, applies a one-pole
  IIR LPF, dead-zone at 0.03 rad/s, then feeds `gain*gz_filtered` into
  commanded omega. Reduces on-ground yaw drift during strafe 40–70%
  depending on gain (tested 2026-04-15). Default gain 1.0 was the
  stability sweet spot.

**Known issues (unfixed in firmware — they live in the chassis/UI):**
- On-ground strafe yaw drift of ~1–2°/s CCW on every non-W direction.
  Ground IMU experiments trace this to asymmetric wheel grip between
  robot sides (Q@0.3 rad/s achieves 102% of commanded yaw vs E@−0.3
  rad/s at 88%; same 16% asymmetry shows up as yaw during translation).
  A chassis reprint with adjustable motor heights was completed in
  April 2026 to address this.

**Not yet done:**
- Full Tier 2 (heading *tracking*, not just rate-null): the current
  heading-hold drives gyro_z → 0 but doesn't return to a reference
  heading after a disturbance. A yaw-integral term or a yaw-angle
  setpoint loop would give absolute heading tracking.
- UWB / Tier 3 EKF: removed from the runtime on 2026-04-14; planned
  re-integration as future work is a range-domain EKF with per-anchor
  calibration.

## Code Style

Arduino/C++ conventions, 2-space indent. Keep functions short. Hardware constants in config.h only.
