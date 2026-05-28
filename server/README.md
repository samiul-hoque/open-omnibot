# Omni-2 Robot Control Server

A Node.js control server for the Omni-2 mecanum wheel robot with multi-tier sensor fusion localization.

## Features

- Real-time WebSocket communication with robot
- Multi-tier localization system for academic comparison
- Keyboard teleoperation for testing
- CSV data logging for analysis
- Mecanum wheel kinematics

## Requirements

- Node.js 18+
- Omni-2 robot running compatible firmware

## Installation

```bash
npm install
```

## Configuration

Edit `src/config.js` to configure:

```javascript
robot: {
    ip: 'robot.local',    // Your robot's IP address
    wsPort: 80,
    wsPath: '/ws',
}
```

### Key Configuration Options

| Option | Description | Default |
|--------|-------------|---------|
| `robot.ip` | Robot IP address | `robot.local` |
| `localizationTier` | Localization algorithm (1, 2, or 3) | `1` |
| `control.speed` | Movement speed (m/s) | `0.15` |
| `control.turnSpeed` | Rotation speed (rad/s) | `0.5` |
| `fusion.imuWeight` | IMU trust in fusion (0-1) | `0.98` |
| `logging.enabled` | Enable CSV logging | `true` |

## Usage

### Start the server

```bash
npm start
```

### Development mode (auto-reload)

```bash
npm run dev
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| `W` | Move forward |
| `S` | Move backward |
| `A` | Strafe left |
| `D` | Strafe right |
| `Q` | Rotate counter-clockwise |
| `E` | Rotate clockwise |
| `Space` | Emergency stop |
| `R` | Reset encoders and pose |
| `P` | Print status |
| `Ctrl+C` | Exit |

## Localization Tiers

The server implements a multi-tier localization strategy for academic comparison:

### Tier 1: Dead Reckoning (Encoders Only)

Pure wheel encoder-based odometry using mecanum wheel kinematics. Will drift over time due to wheel slip and accumulated errors.

```javascript
localizationTier: 1
```

### Tier 2: Encoder + IMU Fusion

Combines encoder odometry for position (X, Y) with IMU yaw for heading. Uses a complementary filter approach for more stable heading estimation.

```javascript
localizationTier: 2
```

### Tier 3: Full EKF with UWB (Planned)

Extended Kalman Filter with Ultra-Wideband positioning for absolute position correction. Not yet implemented.

## Data Logging

Sensor data is logged to CSV files in the `logs/` directory with the format:

```
robot_data_tier{N}_{timestamp}.csv
```

### Logged Fields

- Timestamps (server and robot)
- Encoder counts and velocities (4 wheels)
- IMU data (yaw, pitch, roll, gyro, accel)
- Calibration status
- Pose estimates (x, y, theta)
- Commands sent

## Project Structure

```
src/
├── index.js              # Main entry point
├── config.js             # Configuration
├── robot/
│   └── robotClient.js    # WebSocket client
├── localization/
│   ├── odometry.js       # Tier 1: Dead reckoning
│   └── fusionBasic.js    # Tier 2: Encoder + IMU fusion
└── logging/
    └── dataLogger.js     # CSV data logger
```

## Robot Communication Protocol

The server communicates with the robot via WebSocket JSON messages.

### Inbound (from robot)

```json
{
  "type": "sensors",
  "t": 12345,
  "enc": [100, 100, 100, 100],
  "vel": [1.0, 1.0, 1.0, 1.0],
  "imu": { "yaw": 0, "pitch": 0, "roll": 0, "gz": 0, "ax": 0, "ay": 0 },
  "cal": { "sys": 3, "gyro": 3, "accel": 3, "mag": 3 }
}
```

### Outbound (to robot)

```json
{ "type": "cmd", "vx": 0.1, "vy": 0, "w": 0 }
{ "type": "stop" }
{ "type": "reset_encoders" }
```

## License

MIT
