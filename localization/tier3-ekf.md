# Tier 3: Extended Kalman Filter

> **Reference theory only.** This tier is **not currently integrated**
> in the firmware/server runtime. The UWB reader, EKF fusion, and
> anchor handling were removed from the runtime on 2026-04-14 pending
> validation of the encoder-only and complementary-filter tiers
> (see `server/ARCHITECTURE.md` *Deferred Work*). The contents of
> this document and the accompanying `matlab/ekf_localization.m` are
> published as reference math for a future release — they are not a
> drop-in working module.

Full sensor fusion with UWB corrections using EKF.

## Theory

The Extended Kalman Filter provides optimal state estimation for nonlinear systems.

### State Vector

```
x = [px, py, theta, vx, vy, omega]^T
```

- `px, py`: Position
- `theta`: Heading
- `vx, vy`: Velocity
- `omega`: Angular velocity

### Process Model (Prediction)

```
x(k+1) = f(x(k), u(k)) + w(k)

px(k+1) = px(k) + (vx*cos(theta) - vy*sin(theta)) * dt
py(k+1) = py(k) + (vx*sin(theta) + vy*cos(theta)) * dt
theta(k+1) = theta(k) + omega * dt
vx(k+1) = vx(k) + ax * dt
vy(k+1) = vy(k) + ay * dt
omega(k+1) = omega(k) + alpha * dt
```

### Measurement Models (Update)

#### UWB Range Measurement
```
z_uwb = sqrt((px - ax)^2 + (py - ay)^2) + v_uwb
```

Where `(ax, ay)` is the anchor position.

#### IMU Measurement
```
z_imu = [ax, ay, omega]^T + v_imu
```

#### Odometry Measurement
```
z_odom = [vx, vy, omega]^T + v_odom
```

## EKF Algorithm

### Prediction Step
```
x_pred = f(x, u)
P_pred = F * P * F^T + Q
```

### Update Step (for each measurement)
```
y = z - h(x_pred)           # Innovation
S = H * P_pred * H^T + R    # Innovation covariance
K = P_pred * H^T * S^-1     # Kalman gain
x = x_pred + K * y          # State update
P = (I - K * H) * P_pred    # Covariance update
```

## Implementation

See `matlab/ekf_localization.m` for reference implementation.

### Tuning Parameters

| Matrix | Description | Tuning |
|--------|-------------|--------|
| Q | Process noise | Increase for faster response |
| R_uwb | UWB measurement noise | ~0.1-0.3m typical |
| R_imu | IMU noise | From datasheet/calibration |
| R_odom | Odometry noise | From encoder resolution |

## UWB Anchor Setup

1. Place 4+ anchors at known positions
2. Ensure good geometry (not collinear)
3. Measure anchor positions accurately
4. Configure in filter initialization

## Advantages

- Bounded error with UWB corrections
- Optimal fusion of all sensors
- Handles sensor failures gracefully
- Provides uncertainty estimates

## Computational Considerations

- Matrix operations at each timestep
- Consider update rates for each sensor
- Can run at 50-100Hz on ESP32
