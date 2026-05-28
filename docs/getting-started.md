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

## Step 3: Configure UWB (5 min)

### Setup Anchors

1. Place 4 DWM1001 modules at room corners
2. Connect each to USB and configure as anchor:
   ```
   [Enter] [Enter]
   nma
   aps <x> <y> <z>
   ```
3. Record anchor positions for configuration

### Setup Tag

1. Connect the robot's DWM1001 module
2. Configure as tag:
   ```
   nmt
   ```

## Step 4: Start Server (5 min)

```bash
cd server

# Install dependencies
npm install

# Configure (edit .env or src/config.js)
# - Set serial port for ESP32
# - Set UWB anchor positions

# Start server
npm start
```

## Step 5: Test Basic Motion

With server running, test robot motion:

```bash
# Using curl or a WebSocket client
curl -X POST http://localhost:8080/command -d '{"vx": 0.1, "vy": 0, "omega": 0}'
```

Robot should move forward slowly.

## Step 6: Verify Localization

1. Check UWB ranges are being received
2. Verify IMU readings in telemetry
3. Compare position estimates to physical position

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
