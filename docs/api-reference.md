# API Reference

Server API documentation for controlling the robot and accessing data.

## WebSocket API

Connect to `ws://localhost:8080` (or configured port).

### Events from Server

#### `position`
Current estimated robot position.

```json
{
  "event": "position",
  "data": {
    "x": 1.234,
    "y": 2.345,
    "theta": 0.785,
    "timestamp": 1234567890
  }
}
```

#### `telemetry`
Raw sensor data.

```json
{
  "event": "telemetry",
  "data": {
    "encoders": [100, 102, 99, 101],
    "imu": {
      "ax": 0.01,
      "ay": -0.02,
      "az": 9.81,
      "gx": 0.001,
      "gy": 0.002,
      "gz": 0.003
    },
    "uwb": [
      {"anchor": 0, "range": 1.523},
      {"anchor": 1, "range": 2.145}
    ],
    "timestamp": 1234567890
  }
}
```

#### `status`
System status updates.

```json
{
  "event": "status",
  "data": {
    "connected": true,
    "battery": 85,
    "mode": "running"
  }
}
```

### Events to Server

#### `command`
Send velocity command.

```json
{
  "event": "command",
  "data": {
    "vx": 0.1,
    "vy": 0.0,
    "omega": 0.0
  }
}
```

| Field | Type | Description | Unit |
|-------|------|-------------|------|
| vx | float | Forward velocity | m/s |
| vy | float | Lateral velocity | m/s |
| omega | float | Angular velocity | rad/s |

#### `stop`
Emergency stop.

```json
{
  "event": "stop"
}
```

#### `setMode`
Change operating mode.

```json
{
  "event": "setMode",
  "data": {
    "mode": "manual"
  }
}
```

Modes: `manual`, `autonomous`, `calibration`

## REST API

### GET /status

Get system status.

**Response:**
```json
{
  "connected": true,
  "uptime": 3600,
  "battery": 85,
  "mode": "manual",
  "position": {
    "x": 1.234,
    "y": 2.345,
    "theta": 0.785
  }
}
```

### GET /config

Get current configuration.

**Response:**
```json
{
  "serialPort": "COM3",
  "baudRate": 115200,
  "anchors": [
    {"id": 0, "x": 0, "y": 0, "z": 1.5},
    {"id": 1, "x": 3, "y": 0, "z": 1.5}
  ],
  "ekfParams": {
    "processNoise": 0.01,
    "measurementNoise": 0.1
  }
}
```

### POST /config

Update configuration.

**Request:**
```json
{
  "ekfParams": {
    "processNoise": 0.02
  }
}
```

### POST /log/start

Start data logging.

**Request:**
```json
{
  "filename": "experiment-001"
}
```

**Response:**
```json
{
  "status": "logging",
  "filename": "experiment-001.csv"
}
```

### POST /log/stop

Stop data logging.

**Response:**
```json
{
  "status": "stopped",
  "filename": "experiment-001.csv",
  "duration": 60.5,
  "samples": 6050
}
```

### POST /calibrate/imu

Start IMU calibration routine.

**Response:**
```json
{
  "status": "calibrating",
  "message": "Keep robot stationary for 10 seconds"
}
```

## Serial Protocol

ESP32 communicates with server via serial (USB or UART).

### Message Format

JSON messages terminated with newline:

```
{"type":"telemetry","enc":[100,102,99,101],"imu":[0.01,-0.02,9.81,0.001,0.002,0.003]}\n
```

### Message Types

#### Telemetry (ESP32 -> Server)
```json
{
  "type": "telemetry",
  "enc": [e1, e2, e3, e4],
  "imu": [ax, ay, az, gx, gy, gz],
  "uwb": [[anchor_id, range_mm], ...]
}
```

#### Command (Server -> ESP32)
```json
{
  "type": "cmd",
  "vx": 0.1,
  "vy": 0.0,
  "omega": 0.0
}
```

#### Stop (Server -> ESP32)
```json
{
  "type": "stop"
}
```

## Error Codes

| Code | Description |
|------|-------------|
| 1001 | Serial port not found |
| 1002 | Serial communication error |
| 2001 | UWB module not responding |
| 2002 | Invalid UWB data |
| 3001 | Configuration error |
| 3002 | Invalid command |
