# API Reference

The system has two WebSocket links and no REST control API:

```
Browser dashboard ◂—— ws/http (:3000) ——▸ Node.js server ◂—— ws (robot :80/ws) ——▸ ESP32 robot
```

All robot control flows through the server. The authoritative,
maintained protocol description is the **Robot Protocol** section of
the server architecture document (`server/ARCHITECTURE.md`); this page is a
summary.

## Server → Robot commands

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

| Field | Type | Description | Unit |
|-------|------|-------------|------|
| vx | float | Forward velocity (+X) | m/s |
| vy | float | Lateral velocity (+Y = left) | m/s |
| w | float | Angular velocity (+ = CCW) | rad/s |

Motors auto-stop after 500 ms without a command (firmware watchdog),
and stop immediately on client disconnect. `motor_test` enters a
calibration mode that bypasses the PID loop; it auto-clears after
500 ms without a new `motor_test`.

## Robot → Server sensor broadcast (20 Hz)

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

| Field | Type | Unit | Notes |
|-------|------|------|-------|
| `t` | int | ms | Firmware monotonic clock (`millis()`) |
| `enc` | int[4] | counts | Cumulative, **wire order `[L1, R1, R2, L2]`** |
| `vel` | float[4] | rad/s | Instantaneous wheel velocity, same order |
| `imu.yaw` | float | degrees | BNO055 Euler yaw (IMUPLUS mode, no magnetometer) |
| `imu.gz` | float | rad/s | Gyro Z (yaw rate) |
| `cal.*` | 0–3 | — | BNO055 calibration status per subsystem |

The wire order `[L1, R1, R2, L2]` differs from the firmware's internal
`[L1, R1, L2, R2]` order — see the index-order convention block in the
server architecture document before touching anything that indexes
these arrays.

## Browser ↔ Server

The dashboard (`http://localhost:3000`) and calibration page
(`/calibration.html`) talk to the server over WebSocket on the same
port: joystick velocity commands, pose/sensor broadcasts (10 Hz),
experiment-runner events (`experiment_tick`, `experiment_paused`), and
passthrough of the calibration commands listed above. The server
relays robot `ack` responses to all connected browsers so UI state
stays in sync.
