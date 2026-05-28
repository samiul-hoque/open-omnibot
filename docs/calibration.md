# Calibration Guide

Proper calibration is essential for accurate localization.

## IMU Calibration

### Gyroscope Bias

1. Place robot stationary on level surface
2. Collect gyroscope data for 30+ seconds
3. Compute mean of each axis as bias

```python
gyro_bias = np.mean(gyro_data, axis=0)
```

4. Update firmware/configuration with bias values

### Accelerometer Calibration

1. Measure gravity vector when stationary
2. Compute bias and scale factors
3. Optional: 6-position calibration for full accuracy

### Magnetometer (if used)

1. Rotate robot 360 degrees slowly
2. Fit ellipsoid to measurements
3. Compute hard/soft iron corrections

## Wheel Encoder Calibration

### Wheel Radius

1. Mark a point on wheel and floor
2. Roll wheel forward exactly 1 revolution
3. Measure distance traveled
4. Calculate: `radius = distance / (2 * pi)`

### Encoder CPR Verification

1. Mark wheel position
2. Rotate exactly 1 revolution
3. Count encoder ticks
4. Verify matches expected CPR

### Wheel Spacing

Measure distance between wheel centers:
- `lx`: Half of front-to-back distance
- `ly`: Half of left-to-right distance

## UWB Calibration

### Anchor Position Survey

Accurate anchor positions are critical. Methods:

1. **Tape Measure**: Simple but less accurate
   - Measure from a common reference point
   - Record (x, y, z) for each anchor

2. **Laser Distance Meter**: Better accuracy
   - Measure inter-anchor distances
   - Use trilateration to compute positions

3. **Total Station**: Best accuracy
   - Professional survey equipment
   - Sub-centimeter accuracy

### Antenna Delay Calibration

1. Place tag at known distance from anchor
2. Compare measured vs actual distance
3. Adjust antenna delay parameter

```
Known distance: 2.000m
Measured: 2.053m
Error: +0.053m → Adjust antenna delay
```

### NLOS Detection

- Mark areas with obstacles between tag and anchors
- Configure NLOS rejection or weighting

## System-Level Calibration

### Drive System

1. Command robot to drive straight
2. Measure actual vs commanded distance
3. Adjust wheel radius or gear ratio

### Rotation

1. Command 360-degree rotation
2. Measure actual rotation angle
3. Adjust wheel spacing parameters

### Timing Synchronization

Ensure all sensors have aligned timestamps:
1. Log simultaneous sensor data
2. Check for time offsets
3. Apply corrections in software

## Calibration Verification

After calibration, verify with test trajectories:

1. Square path: Should return to start
2. Figure-8: Tests both translation and rotation
3. Compare to ground truth if available

## Saving Calibration

Store calibration parameters in:
- `firmware/esp32-omni/src/config.h`
- `server/src/config.js`
- Dataset metadata files

## Recalibration Schedule

Recalibrate when:
- Hardware changes (motor replacement, etc.)
- Environmental changes (new room, obstacles)
- Performance degradation observed
- At least every few months
