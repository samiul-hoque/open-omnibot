---
last-verified: 2026-04-09
sources:
  - server/src/web/public/calibration.js
  - server/src/web/public/diagnostics.js
  - server/src/logging/dataLogger.js
  - firmware/esp32-omni/src/websocket_server.cpp
---

# Diagnostics & Debug Tooling

## TL;DR

A separate Diagnostics view in the web UI exists because the main control dashboard hides too much when something goes wrong. Each panel was added in response to a specific class of bug we couldn't otherwise see: motor uniformity bars exposed gearbox variance, the IMU calibration indicators saved hours of "wave the robot until it works" guessing, the latency breakdown caught a WiFi power-save bug that looked like a control issue, and the wireless log stream replaced the USB serial cable for field testing. The CSV data logger has its own AI-friendly tweaks (metadata sidecar, idle suppression, event tagging) so post-hoc analysis doesn't require hand-counting columns.

## Why a separate diagnostics view at all

The control dashboard is optimised for **driving the robot**: big map, joystick, telemetry summary, E-STOP. Anything that doesn't fit that one job clutters the view, so when a bug happened we'd find ourselves opening a serial monitor in another window, watching log lines fly by, and trying to correlate them with what the robot was doing.

This worked exactly until we needed to debug something *while* the robot was rolling. Then:
- The serial cable was inconvenient (you can't drive a robot in a circle with a cable trailing it).
- Numeric data on a serial console is unreadable in real time — humans can't track 4 motor velocities, IMU calibration, and WiFi RSSI simultaneously.
- Every bug investigation required a colleague to drive while you watched.

The fix was a dedicated UI: every diagnostic that we'd previously eyeballed in serial output became a panel with the right visualization (bar chart, sparkline, color-coded indicator, etc.). The control dashboard stayed clean; the diagnostics view became the place to **look** at the robot without interfering with it.

## Panels and the bugs they exist for

Each panel below corresponds to a real failure mode we couldn't diagnose before adding it.

### Motor cards (per-PWM control + live encoder/velocity)

**File:** `server/src/web/public/calibration.js`

**Bug they exposed:** the very first symptom of mismatched gearmotors. Before the calibration page existed, all four motors were always commanded together by the PID. There was no way to drive *one* motor at a known PWM and observe its encoder rate. Once you have that, the disagreement becomes obvious within five seconds.

**What they show:**
- Per-motor PWM slider (-255 to +255)
- Current encoder count and velocity (rad/s)
- Run/stop button per motor
- "Lockup" warning if |PWM| > 50 but encoder hasn't moved for 500 ms — catches a stalled or disconnected motor

The lockup detector was added after a session where one of the motors had a wire come loose mid-test. Without the warning, the operator kept commanding higher PWM thinking the motor was sluggish, until the driver overheated.

### Uniformity test (bar chart of per-motor velocity at the same PWM)

**Bug it exposed:** the **106% encoder spread** that drove the development of automated motor calibration. See `11-motor-calibration.md`. Before the bar chart existed we had numeric tables; nobody noticed the spread until it was visualised side-by-side.

**Behaviour:** runs all four motors at the same PWM for ~2 seconds, normalises by the group mean, draws bars colour-coded green (<10% deviation), yellow (10–25%), red (>25%). After running automated calibration, all four bars should be green.

This is now part of the post-calibration verification ritual: if the bars aren't green after `start_motor_cal`, something is mechanically wrong (a gear is stripped, a wire is loose, the wheel is jammed) and the calibration won't help.

### IMU calibration indicators

**Bug they exposed:** the BNO055 was reporting absurd headings on first power-up because the chip wasn't fully calibrated, and there was no visible indication of calibration state. Operators would zero the IMU, drive a meter, see a 30° heading drift, and conclude the IMU was broken — when in reality the gyro hadn't finished its bias estimation.

**What they show:**
- Live `sys`/`gyro`/`accel`/`mag` calibration values (0-3) as colour-coded dots: red 0, orange 1, yellow 2, green 3.
- Contextual guidance text: "Wave the robot in a figure-8 to calibrate the magnetometer" — shown only when `mag < 3`.
- Save/load/zero buttons (see `13-imu.md` for the NVS persistence story).

The contextual text is the part that took the most iteration. Early versions just showed numbers; users had to look up what each number meant. Embedding the instructions in the UI cut "calibrate the IMU" from a 5-minute training-required task to a 30-second walk-up.

### Latency & clock sync panel

**Bug it exposed:** the **WiFi power-save stuttering** described in `20-communications.md` (TBD). When the symptom first appeared we thought it was a PID gain problem — the robot was producing exactly the kind of jerky motion that bad gains produce. Without per-hop latency we'd have spent days re-tuning the PID for a problem that had nothing to do with the PID.

**What it shows:**
- NTP sync status for browser, server, robot (three layers)
- Clock offset estimation (NTP-style from roundtrip timing)
- Per-hop latency: browser → server, server → robot, round-trip
- Sensor pipeline latency: timestamp at sensor read → timestamp at browser arrival
- Sparklines (60 samples, ~6 seconds at 10 Hz) for round-trip and pipeline latency
- A 50 ms threshold line on the sparkline so you can see at a glance whether you're inside or outside the budget

The per-hop breakdown was the key insight: when the WiFi was sleeping, the **server → robot** hop spiked into the hundreds of ms while every other hop was nominal. That immediately pointed at the radio, not the control loop.

**How the timestamps chain:**
- Browser stamps `ts` on outbound `ping_cal`
- Server stamps `ts_server_fwd` and forwards
- Robot stamps `ts_robot` and replies
- Server stamps `ts_server_ret`
- Browser computes deltas on receipt

For sensor pipeline latency, the robot includes its UTC wall clock (`utc` field) in every sensor packet (when NTP is synced); the browser subtracts it from its own wall clock to estimate end-to-end delay. See `22-time-sync.md` for why we needed NTP at all.

### WiFi panel (RSSI + sparkline)

**Bug it exposed:** intermittent disconnects in conference demo halls. RSSI alone wasn't enough — you needed the *trend* over 10–20 seconds to spot a roaming/handoff cycle. The sparkline made the pattern obvious.

### Robot info panel

Firmware version, MAC address, IP, free heap, uptime, NTP-synced wall clock, current motor gain values. Mostly used to confirm "yes, that OTA upload actually flashed the new firmware" (the firmware version field is the cheapest way to verify).

## Wireless log stream

**File:** `firmware/esp32-omni/src/websocket_server.cpp` — `wsLog()`

`wsLog(const char* fmt, ...)` is a printf-style helper that broadcasts a `{type: "log", msg: ...}` packet to **all** WebSocket clients. The browser appends each message to the on-page debug log with a timestamp.

Why it exists: USB serial is unavailable during field testing. The robot is wireless, on battery, possibly across the room. We wanted `Serial.printf` ergonomics without needing a cable. `wsLog` is now used everywhere a serial print would have been:

- Motor calibration progress ("step 3/6 — PWM 160 fwd")
- IMU calibration save/load confirmations
- WiFi state changes (connected, lost, AP fallback)
- OTA upload progress
- Errors and warnings

The browser-side log panel auto-scrolls, has 200-line history, and is colour-coded by level (info / warn / error). Same data is also written to the CSV log so post-mortem analysis doesn't require a screenshot.

## AI-friendly CSV data logging

**File:** `server/src/logging/dataLogger.js`

The data logger writes a CSV per session in `logs/robot_data_tier{N}_{timestamp}.csv`. Three features were added specifically to make the logs useful for **automated** analysis (Python scripts, Jupyter notebooks, future LLM-driven post-mortems) rather than only human-readable.

### 1. JSON metadata sidecar

Every CSV gets a `.meta.json` file alongside it containing:
- Column names and units
- Physical constants (wheel radius, gear ratio, encoder CPR)
- Localization tier in use
- Session start/stop timestamps
- Firmware version and motor gain values at session start

**Why:** Without a metadata file, an analysis script has to either hard-code column indices (which break when the schema changes) or parse the CSV header (which doesn't include units, constants, or tier info). The sidecar is the single source of truth: "given this CSV, what does column 17 represent and what units is it in?" is now a one-line lookup.

### 2. Idle suppression

When the robot is stationary for >2 seconds, the logger drops from 10 Hz to 1 Hz. Full rate resumes the moment a motion command arrives or any sensor reading changes.

**Why:** A typical demo session has long idle periods (setup, between-runs, talking to spectators). At 10 Hz constant, a 30-minute session was producing >15 MB CSVs that were 90% identical rows. After idle suppression the same session is ~2 MB with no loss of resolution during the parts you actually care about.

The 2-second threshold and 1 Hz fallback were tuned to be invisible: motion analysis cares about the moments around state transitions, not the steady states.

### 3. Empty strings for missing sensors

When a sensor is disconnected (e.g. UWB serial unplugged, IMU init failed), its columns are written as empty strings rather than zeros.

**Why:** Zero is a valid sensor reading. An analysis script that sees a 0 in the UWB column has no way to distinguish "robot is at the origin" from "no UWB data". An empty cell is unambiguous and pandas/numpy handle it natively as NaN.

### 4. Event column with deduplication

Column 33 is a free-text `event` field. Discrete events (`motor_test`, `motor_cal_step`, `imu_zero`, etc.) are tagged here. Continuous events like `motor_test` only emit a row when the parameters change — you don't get 200 identical `motor_test motor=0 pwm=128` rows during a slow PWM ramp.

**Why:** Lets analysis scripts find "all the rows where the user was running the motor test" without scanning a sensor pattern. Deduplication keeps the CSV slim while preserving the event timeline.

## Tuning / configuration

Diagnostics-side knobs are all in the web UI source:

| File | Knob | Default | Notes |
|---|---|---|---|
| `dataLogger.js` | `IDLE_THRESHOLD_MS` | 2000 | How long stationary before dropping to slow rate |
| `dataLogger.js` | `IDLE_LOG_HZ` | 1 | Slow-rate during idle |
| `dataLogger.js` | `ACTIVE_LOG_HZ` | 10 | Normal rate |
| `diagnostics.js` | `LATENCY_HISTORY_MAX` | 60 | Sparkline history depth (~6 s at 10 Hz) |
| `diagnostics.js` | `RSSI_HISTORY_MAX` | 60 | Same |
| `diagnostics.js` | latency threshold line | 50 ms | Visual budget marker |

## Known limitations

- **Diagnostics view is read-only.** You can't, for example, set a PID gain from the diagnostics view — you have to use the calibration page or modify firmware. Intentional separation; mixing read/write would tempt people to "tune things to fix the diagnostic indicator" which is a recipe for chasing symptoms.
- **No alerting.** If RSSI drops to -90 dBm or latency spikes to 500 ms, nothing tells you — you have to be looking at the panel. A future version could badge the nav item with a warning dot.
- **CSV idle suppression has no UI override.** If you want continuous high-rate logging during a stationary calibration session, you have to nudge the robot every 2 seconds. Manageable in practice but occasionally annoying.
- **Wireless log stream has no level filter on the browser side.** If the robot spams `wsLog`, the browser slows down. The 200-line history caps the impact but very chatty firmware can still hurt FPS.
- **No "replay" of a CSV session.** The CSV holds everything you'd need to drive the dashboard from a recorded session, but the player UI doesn't exist yet. Would be a high-value future addition.

## Source

- `server/src/web/public/calibration.js` — motor cards, uniformity test, IMU panel, manual gain editor
- `server/src/web/public/diagnostics.js` — latency panel, sparklines, robot info, RSSI
- `server/src/web/public/index.html` — view-calibration and view-diagnostics sections (the SPA shells for each panel)
- `server/src/logging/dataLogger.js` — CSV logger, metadata sidecar, idle suppression, event column
- `firmware/esp32-omni/src/websocket_server.cpp` — `wsLog`, `ping_cal` handler, `motor_test` handler
- Related: `11-motor-calibration.md`, `13-imu.md`, `22-time-sync.md`
