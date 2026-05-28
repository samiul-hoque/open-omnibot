# Tier 2: Sensor Fusion

Complementary filter combining odometry and IMU.

## Theory

A complementary filter combines high-frequency and low-frequency sensor data:

- **Gyroscope**: Good for short-term (high-freq), drifts long-term
- **Accelerometer**: Good for long-term (low-freq), noisy short-term

### Complementary Filter for Heading

```
theta = alpha * (theta + gyro_z * dt) + (1 - alpha) * accel_heading
```

Where `alpha` is typically 0.95-0.98 (trust gyro more for short-term).

### For Position

Fuse odometry velocity with IMU acceleration:

```
velocity = alpha * odom_velocity + (1 - alpha) * integrated_accel
position += velocity * dt
```

## Implementation

See `matlab/complementary_filter.m` for reference implementation.

### Tuning Parameters

| Parameter | Description | Range |
|-----------|-------------|-------|
| alpha | Filter coefficient | 0.90 - 0.99 |
| gyro_bias | Gyroscope offset | Calibrate at startup |
| accel_bias | Accelerometer offset | Calibrate at startup |

## IMU Calibration

1. Place robot stationary on level surface
2. Collect 1000+ samples
3. Compute mean as bias offset
4. Apply bias correction in real-time

## Advantages over Tier 1

- Reduced heading drift
- Smoother velocity estimates
- Robust to wheel slip (partially)

## Limitations

- Still accumulates position drift
- Requires IMU calibration
- No absolute position correction
