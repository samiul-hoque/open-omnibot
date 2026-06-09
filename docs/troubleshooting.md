# Troubleshooting

Common issues and solutions.

## Firmware Issues

### ESP32 Not Detected

**Symptoms**: PlatformIO can't find serial port

**Solutions**:
1. Check USB cable (use data cable, not charge-only)
2. Install CP210x or CH340 drivers
3. Check Device Manager for COM port
4. Try different USB port

### Upload Fails

**Symptoms**: Upload times out or fails

**Solutions**:
1. Hold BOOT button during upload
2. Reduce upload speed in platformio.ini
3. Check for other serial monitors using the port

### Motors Don't Move

**Symptoms**: Commands received but no motor movement

**Solutions**:
1. Check motor driver power supply
2. Verify motor connections
3. Check enable pins on motor driver
4. Test with simple motor test sketch

## Communication Issues

### Serial Port Busy

**Symptoms**: "Port already in use" error

**Solutions**:
1. Close PlatformIO serial monitor
2. Close other terminal programs
3. Restart server application

### WebSocket Not Connecting

**Symptoms**: Client can't connect to server

**Solutions**:
1. Check server is running
2. Verify correct port number
3. Check firewall settings
4. Try localhost vs IP address

## UWB Issues

> **[NOT CURRENTLY ACTIVE]** UWB was removed from the runtime
> 2026-04-14; re-integration is planned future work. This section is
> retained for the bench-characterization phase of that work.

### No Range Measurements

**Symptoms**: UWB data not received

**Solutions**:
1. Check tag/anchor power
2. Verify tag is configured as tag, anchors as anchors
3. Check UART connection to ESP32
4. Verify baud rate matches

### High Range Error

**Symptoms**: UWB ranges consistently wrong

**Solutions**:
1. Recalibrate antenna delay
2. Check for NLOS conditions
3. Verify anchor positions are accurate
4. Check for interference sources

### Intermittent Ranges

**Symptoms**: Range measurements drop out

**Solutions**:
1. Check battery power levels
2. Reduce range (move anchors closer)
3. Check for obstructions
4. Verify antenna orientation

## IMU Issues

### IMU Not Responding

**Symptoms**: I2C communication fails

**Solutions**:
1. Check wiring (SDA, SCL, power)
2. Verify I2C address (0x29 for the BNO055 as wired on this robot)
3. Add pull-up resistors if needed
4. Check voltage levels (3.3V vs 5V)
5. BNO055 stuck-read lockup: firmware watchdog latches after ~1 s of
   bit-identical gyro reads — power cycle required (software reset is
   not sufficient)

### Drifting Heading

**Symptoms**: Heading drifts over time

**Solutions**:
1. Calibrate gyroscope bias
2. Increase complementary filter alpha
3. Add magnetometer for absolute heading
4. Ensure IMU is away from motors (magnetic interference)

### Noisy Accelerometer

**Symptoms**: Acceleration data very noisy

**Solutions**:
1. Add low-pass filtering
2. Check mechanical mounting (vibration isolation)
3. Reduce accelerometer range setting
4. Average multiple samples

## Localization Issues

### Large Position Drift

**Symptoms**: Estimated position drifts from actual

**Solutions**:
1. Verify wheel calibration (radius, spacing)
2. Check encoder connections
3. Recalibrate IMU
4. Increase UWB update rate

### Jumpy Position Estimates

**Symptoms**: Position jumps suddenly

**Solutions**:
1. Increase measurement noise (R matrix) in EKF
2. Add outlier rejection for UWB
3. Check sensor timestamps alignment
4. Reduce process noise (Q matrix)

### Poor Convergence

**Symptoms**: EKF takes long to converge or diverges

**Solutions**:
1. Tune initial covariance (P matrix)
2. Check process model matches actual motion
3. Verify measurement models are correct
4. Check for sensor faults

## Performance Issues

### Slow Update Rate

**Symptoms**: Control loop slower than expected

**Solutions**:
1. Reduce serial print statements
2. Optimize sensor read functions
3. Use interrupts for encoders
4. Check for blocking delays

### Server Memory Issues

**Symptoms**: Node.js crashes with memory errors

**Solutions**:
1. Check for memory leaks in logging
2. Limit data buffer sizes
3. Increase Node.js memory limit
4. Implement circular buffers

## Getting Help

If issues persist:
1. Check GitHub issues for similar problems
2. Include detailed logs and configuration
3. Describe hardware setup
4. Share relevant code snippets
