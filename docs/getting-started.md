# Getting Started

30-minute quickstart guide to get your Open Omnibot running.

## Prerequisites

- Hardware components (see [hardware/README.md](../hardware/README.md))
- PlatformIO installed
- Node.js 18+
- Python 3.8+ (for evaluation scripts)

## Step 1: Assemble Hardware (15 min)

1. 3D print or prepare chassis parts
2. Mount motors and wheels
3. Install motor drivers
4. Connect ESP32 and IMU
5. Wire power system

See [hardware/wiring/](../hardware/wiring/) for connection diagrams.

## Step 2: Flash Firmware (5 min)

```bash
cd firmware/esp32-omni

# Install dependencies and build
pio run

# Connect ESP32 via USB and upload
pio run -t upload

# Verify with serial monitor
pio device monitor
```

You should see "Open Omnibot Starting..." on successful boot.

> **Note:** UWB positioning is **not part of the current runtime** — it
> was removed 2026-04-14 and is planned future work. Localization runs
> on wheel encoders + IMU (Tiers 0–2). Do not set up DWM1001 anchors as
> part of getting started.

## Step 3: Start Server (5 min)

```bash
cd server

# Install dependencies
npm install

# Configure if needed (src/config.js)

# Start server
npm start
```

## Step 4: Test Basic Motion

With the server running, open the browser dashboard at
`http://localhost:3000` and use the joystick, or use keyboard control in
the server terminal (WASD + QE). The robot should respond immediately;
watch the live pose on the 2D map.

## Step 5: Verify Localization

1. Verify encoder counts and IMU readings in telemetry
2. Compare position estimates to physical position (drive a known
   distance, check the dashboard pose)

## Next Steps

- [Calibration Guide](calibration.md) - Improve accuracy
- [API Reference](api-reference.md) - Full command set
- [Troubleshooting](troubleshooting.md) - Common issues

## Quick Reference

| Task | Command |
|------|---------|
| Build firmware | `pio run` |
| Upload firmware | `pio run -t upload` |
| Serial monitor | `pio device monitor` |
| Start server | `npm start` |
| Run evaluation | `python evaluation/scripts/analyze_trajectory.py` |
