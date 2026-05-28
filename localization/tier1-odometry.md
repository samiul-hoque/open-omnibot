# Tier 1: Wheel Odometry

Dead reckoning using wheel encoder measurements.

## Theory

For an omnidirectional robot with mecanum wheels, the robot velocity in the body frame can be computed from wheel velocities.

### Kinematic Model

For a 4-wheel mecanum drive, the body-frame velocities are computed
from wheel angular velocities as:

```
vx = (w1 + w2 + w3 + w4) * r / 4
vy = (w1 - w2 + w3 - w4) * r / 4
wz = (-w1 + w2 + w3 - w4) * r / (4 * (lx + ly))
```

Where:
- `w1, w2, w3, w4` = wheel angular velocities (rad/s), indexed in
  **wire order** `[front-left, front-right, rear-right, rear-left]`
  (the order the robot broadcasts on the WebSocket stream).
- `r` = wheel radius
- `lx, ly` = half the lateral and longitudinal distance between
  wheel contact points and the body center.

The sign conventions above are: `vx` positive = forward, `vy`
positive = body-left, `wz` positive = counter-clockwise (right-hand
rule about body z). These match `server/src/localization/mecanumKinematics.js`
and `firmware/esp32-omni/src/mecanum.cpp` — if you derive your own
mecanum kinematics, double-check the sign pattern against those
sources, as a single sign flip silently produces drift along the
wrong axis.

### Position Integration

```
x(t+dt) = x(t) + (vx*cos(theta) - vy*sin(theta)) * dt
y(t+dt) = y(t) + (vx*sin(theta) + vy*cos(theta)) * dt
theta(t+dt) = theta(t) + wz * dt
```

## Implementation

See `matlab/odometry.m` for reference implementation.

### Key Parameters

Values shown are for the Open Omnibot reference platform; substitute
your own when adapting the model.

| Parameter | Description | Open Omnibot value |
|-----------|-------------|--------------------|
| Wheel radius (`r`) | Radius of mecanum wheel | 40 mm |
| Encoder CPR | Quadrature-decoded counts per wheel revolution (post-gearbox) | 1092 |
| Gear ratio | Motor-to-wheel reduction | 42 : 1 |
| Half-track width (`lx`) | Half the lateral distance between wheel centers | 0.1175 m |
| Half-wheelbase (`ly`) | Half the longitudinal distance between wheel centers | 0.0953 m |
| Sensor stream rate | WebSocket telemetry update period | 20 Hz (50 ms) |

## Limitations

- **Drift**: Errors accumulate over time
- **Slip**: Wheel slip causes errors
- **No absolute reference**: Cannot correct accumulated errors

## Calibration

1. Measure actual wheel radius
2. Verify encoder counts per revolution
3. Drive known distances and adjust parameters
