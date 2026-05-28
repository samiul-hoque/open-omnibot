#include "sensors.h"
#include "config.h"
#include "driver/pcnt.h"
#include <Wire.h>

// ============================================
// Private Variables
// ============================================

static Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS);
static bool imuAvailable = false;

// IMU orientation offsets for zeroing
static float yawOffset = 0;
static float pitchOffset = 0;
static float rollOffset = 0;

// Encoder pin configurations - indices 2 and 3 swapped to match physical wiring
// Physical: [L1, R1, L2, R2] at indices [0, 1, 2, 3]
static const uint8_t encAPins[4] = {MOTOR_L1_ENC_A, MOTOR_R1_ENC_A, MOTOR_L2_ENC_A, MOTOR_R2_ENC_A};
static const uint8_t encBPins[4] = {MOTOR_L1_ENC_B, MOTOR_R1_ENC_B, MOTOR_L2_ENC_B, MOTOR_R2_ENC_B};
static const pcnt_unit_t pcntUnits[4] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};
static const int8_t encDirs[4] = {ENC_L1_DIR, ENC_R1_DIR, ENC_L2_DIR, ENC_R2_DIR};
static const char* motorNames[4] = {"L1", "R1", "L2", "R2"};

// Per-motor encoder gain factors (separate forward/reverse to handle directional asymmetry)
static float motorGainFwd[4] = {1.0f, 1.0f, 1.0f, 1.0f};
static float motorGainRev[4] = {1.0f, 1.0f, 1.0f, 1.0f};

// For velocity calculation (broadcast path)
static int32_t lastCounts[4] = {0, 0, 0, 0};
static uint32_t lastVelocityTime = 0;

// Odometry-dedicated encoder delta tracking (independent from broadcast and PID)
static int32_t odomLastCounts[4] = {0, 0, 0, 0};
static uint32_t odomLastTime = 0;

// Cached sensor snapshot (updated once per motor-update cycle)
static CachedSensors sensorCache = {};

// IMU stuck-read watchdog state. See isIMUStuck() in sensors.h for the
// failure mode this guards against. NaN-init means the first real read
// always differs (NaN != any float in IEEE 754), so we never false-
// positive on the very first cycle.
#define IMU_STUCK_THRESHOLD 50  // ~1 s at the 50 Hz sensor-cache update
static float    imuLastGyroZ   = NAN;
static uint16_t imuStuckCount  = 0;
static bool     imuStuckLatched = false;

// Overflow tracking for 16-bit PCNT
static int32_t overflowCounts[4] = {0, 0, 0, 0};
static int16_t lastRawCounts[4] = {0, 0, 0, 0};

// ============================================
// Encoder Functions
// ============================================

static void setupSingleEncoder(int index) {
    pcnt_config_t pcnt_config = {};
    
    pcnt_config.pulse_gpio_num = (gpio_num_t)encAPins[index];
    pcnt_config.ctrl_gpio_num  = (gpio_num_t)encBPins[index];
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.unit           = pcntUnits[index];
    pcnt_config.pos_mode       = PCNT_COUNT_DEC;
    pcnt_config.neg_mode       = PCNT_COUNT_INC;
    pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.counter_h_lim  = PCNT_H_LIM;
    pcnt_config.counter_l_lim  = PCNT_L_LIM;
    
    esp_err_t err = pcnt_unit_config(&pcnt_config);
    if (err != ESP_OK) {
        Serial.printf("ERROR: PCNT config failed for motor %s: %d\n", motorNames[index], err);
        return;
    }
    
    // Glitch filter
    pcnt_set_filter_value(pcntUnits[index], 100);
    pcnt_filter_enable(pcntUnits[index]);
    
    // Clear and start
    pcnt_counter_pause(pcntUnits[index]);
    pcnt_counter_clear(pcntUnits[index]);
    pcnt_counter_resume(pcntUnits[index]);
    
    Serial.printf("Encoder %s initialized (GPIO %d/%d)\n", 
                  motorNames[index], encAPins[index], encBPins[index]);
}

bool initEncoders() {
    Serial.println("Initializing encoders...");
    
    for (int i = 0; i < 4; i++) {
        setupSingleEncoder(i);
        lastCounts[i] = 0;
        overflowCounts[i] = 0;
        lastRawCounts[i] = 0;
    }
    
    lastVelocityTime = millis();
    
    Serial.println("Encoders initialized");
    return true;
}

int32_t getEncoderCount(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= 4) return 0;
    
    int16_t rawCount = 0;
    pcnt_get_counter_value(pcntUnits[motorIndex], &rawCount);
    
    // Handle overflow detection using calculated threshold from PCNT limits
    // Threshold is half the counter range to reliably detect wrap-around
    int16_t diff = rawCount - lastRawCounts[motorIndex];
    if (diff > PCNT_OVERFLOW_THRESHOLD) {
        // Underflow occurred (counter wrapped from low to high)
        overflowCounts[motorIndex] -= 65536;
    } else if (diff < -PCNT_OVERFLOW_THRESHOLD) {
        // Overflow occurred (counter wrapped from high to low)
        overflowCounts[motorIndex] += 65536;
    }
    lastRawCounts[motorIndex] = rawCount;
    
    // Apply direction correction
    return (overflowCounts[motorIndex] + rawCount) * encDirs[motorIndex];
}

void resetEncoder(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= 4) return;
    
    pcnt_counter_clear(pcntUnits[motorIndex]);
    overflowCounts[motorIndex] = 0;
    lastRawCounts[motorIndex] = 0;
    lastCounts[motorIndex] = 0;
    
    Serial.printf("Encoder %s reset\n", motorNames[motorIndex]);
}

void resetAllEncoders() {
    for (int i = 0; i < 4; i++) {
        resetEncoder(i);
    }
    Serial.println("All encoders reset");
}

EncoderData readEncoders() {
    EncoderData data;
    data.timestamp = millis();
    
    uint32_t dt = data.timestamp - lastVelocityTime;
    if (dt == 0) dt = 1;  // Prevent division by zero
    
    float dt_sec = dt / 1000.0f;
    
    // Motor direction constants — must match config.h / motors.cpp.
    // Order matches the canonical internal index order [L1, R1, L2, R2]
    // used everywhere else.
    static const int8_t motorDirs[4] = { MOTOR_L1_DIR, MOTOR_R1_DIR, MOTOR_L2_DIR, MOTOR_R2_DIR };

    for (int i = 0; i < 4; i++) {
        data.counts[i] = getEncoderCount(i);

        // Calculate TRUE wheel velocity in rad/s. Motor gains are NOT
        // applied to the measurement — gain compensation belongs on the
        // feedforward PWM side (see pid_controller.cpp). Applying gain
        // here would make the PID loop close on gained units against a
        // true-units target, forcing true_speed = target/gain at steady
        // state and producing per-wheel asymmetry under closed-loop
        // control.
        int32_t deltaCounts = (data.counts[i] - lastCounts[i]) * (-motorDirs[i]);
        float wheelRevs = (float)deltaCounts / COUNTS_PER_WHEEL_REV;
        float radians = wheelRevs * 2.0f * 3.14159265f;
        data.velocities[i] = radians / dt_sec;

        lastCounts[i] = data.counts[i];
    }
    
    lastVelocityTime = data.timestamp;
    
    return data;
}

// ============================================
// Odometry Encoder Reading (own delta state)
// ============================================

EncoderData readOdomEncoders() {
    EncoderData data;
    data.timestamp = millis();

    uint32_t dt = data.timestamp - odomLastTime;
    if (dt == 0) dt = 1;

    float dt_sec = dt / 1000.0f;

    // Must match the motorDirs array in readEncoders() / pid_controller.cpp
    // / motor_calibration.cpp. If any MOTOR_*_DIR in config.h is flipped,
    // all four sites consume that flip through this same multiplication.
    static const int8_t motorDirs[4] = { MOTOR_L1_DIR, MOTOR_R1_DIR, MOTOR_L2_DIR, MOTOR_R2_DIR };

    for (int i = 0; i < 4; i++) {
        data.counts[i] = getEncoderCount(i);

        // True wheel velocity — gain compensation lives on the feedforward
        // side in pid_controller.cpp, not on the measurement. See
        // readEncoders() above for rationale. The motor-direction sign is
        // applied here for parity with readEncoders/PID/motor-cal so pose
        // integration stays correct if anyone flips a MOTOR_*_DIR.
        int32_t deltaCounts = (data.counts[i] - odomLastCounts[i]) * (-motorDirs[i]);
        float wheelRevs = (float)deltaCounts / COUNTS_PER_WHEEL_REV;
        float radians = wheelRevs * 2.0f * 3.14159265f;
        data.velocities[i] = radians / dt_sec;

        odomLastCounts[i] = data.counts[i];
    }

    odomLastTime = data.timestamp;

    return data;
}

void resetOdomEncoders() {
    for (int i = 0; i < 4; i++) {
        odomLastCounts[i] = getEncoderCount(i);
    }
    odomLastTime = millis();
}

// ============================================
// Sensor Cache (single read per cycle)
// ============================================

void updateSensorCache() {
    sensorCache.enc = readEncoders();
    sensorCache.imuValid = isIMUAvailable();
    if (sensorCache.imuValid) {
        sensorCache.imu = readIMU();

        // Stuck-read watchdog: a live BNO055 always shows sub-LSB noise
        // on gyro_z even at rest, so bit-identical consecutive reads are
        // diagnostic of the I2C-frozen/chip-locked-up failure mode.
        // Latch the flag once tripped; only a reboot clears it.
        if (!imuStuckLatched) {
            if (sensorCache.imu.gyro_z == imuLastGyroZ) {
                if (imuStuckCount < 0xFFFF) imuStuckCount++;
                if (imuStuckCount >= IMU_STUCK_THRESHOLD) {
                    imuStuckLatched = true;
                }
            } else {
                imuStuckCount = 0;
            }
            imuLastGyroZ = sensorCache.imu.gyro_z;
        }
    } else {
        sensorCache.imu = IMUData{};
    }
}

bool isIMUStuck() {
    return imuStuckLatched;
}

const CachedSensors& getSensorCache() {
    return sensorCache;
}

// ============================================
// IMU Functions
// ============================================

bool initIMU() {
    Serial.println("Initializing IMU...");

    if (!bno.begin()) {
        Serial.println("WARNING: BNO055 not found! Continuing without IMU.");
        imuAvailable = false;
        return false;
    }

    delay(100);
    bno.setExtCrystalUse(true);

    // Configure for upside-down mounting (chip facing floor)
    // X-axis points forward, Y-axis points left, but Z-axis points down
    // This is a 180° rotation around X, so we invert Y and Z signs
    bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);  // Default: X=X, Y=Y, Z=Z
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);     // Invert Y and Z

    // Switch to IMUPLUS (6-DOF: accel + gyro only, no magnetometer) after
    // the axis config is done. Motors + MCP23017 sitting ~10 cm from the
    // chip make the on-board magnetometer useless, and in NDOF mode that
    // mag noise contaminates fused yaw and linear-accel. IMUPLUS gives
    // clean short-horizon yaw at the cost of long-term absolute-heading
    // drift, which matches how we use the IMU (heading-hold over seconds,
    // dead-reckoning windows, experiment A/Bs).
    //
    // The BNO055 datasheet specifies ~650 ms stabilisation after entering
    // a fusion mode; without it, the first several reads can come back as
    // zeros (observed intermittently on our board, 2026-04-15). Verify
    // afterwards with getMode() — if the read-back mode isn't IMUPLUS,
    // fall back to the pre-switch behaviour rather than silently serving
    // zeros.
    bno.setMode(OPERATION_MODE_IMUPLUS);
    delay(700);

    adafruit_bno055_opmode_t readback = bno.getMode();
    if (readback != OPERATION_MODE_IMUPLUS) {
        Serial.printf("WARNING: BNO055 mode readback=0x%02X (expected IMUPLUS=0x08). Retrying...\n", readback);
        bno.setMode(OPERATION_MODE_IMUPLUS);
        delay(700);
    }

    imuAvailable = true;

    Serial.println("BNO055 IMU initialized (IMUPLUS, upside-down mount)");
    return true;
}

bool isIMUAvailable() {
    return imuAvailable;
}

void zeroIMU() {
    if (!imuAvailable) return;

    // Read current orientation and store as offset
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    yawOffset = orientationData.orientation.x;
    pitchOffset = orientationData.orientation.y;
    rollOffset = orientationData.orientation.z;

    Serial.printf("IMU zeroed (offsets: yaw=%.2f, pitch=%.2f, roll=%.2f)\n",
                  yawOffset, pitchOffset, rollOffset);
}

IMUData readIMU() {
    IMUData data = {0};
    data.timestamp = millis();
    
    if (!imuAvailable) {
        return data;
    }
    
    // Get orientation (Euler angles) and apply zero offsets
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    // Apply offsets and negate to convert from BNO055 compass convention
    // (CW positive) to robot convention (CCW positive), then normalize.
    data.yaw = -(orientationData.orientation.x - yawOffset);
    if (data.yaw > 180.0f) data.yaw -= 360.0f;
    if (data.yaw < -180.0f) data.yaw += 360.0f;

    data.pitch = orientationData.orientation.y - pitchOffset;
    data.roll = orientationData.orientation.z - rollOffset;
    
    // Get angular velocity
    sensors_event_t gyroData;
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    data.gyro_x = gyroData.gyro.x;
    data.gyro_y = gyroData.gyro.y;
    // Negate to match CCW-positive robot convention (BNO055 reports CW-positive)
    data.gyro_z = -gyroData.gyro.z;
    
    // Get linear acceleration (gravity removed)
    sensors_event_t accelData;
    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    data.accel_x = accelData.acceleration.x;
    data.accel_y = accelData.acceleration.y;
    data.accel_z = accelData.acceleration.z;
    
    // Get calibration status
    bno.getCalibration(&data.cal_system, &data.cal_gyro, &data.cal_accel, &data.cal_mag);

    return data;
}

// ============================================
// IMU Calibration Persistence Helpers
// ============================================

bool getIMUCalibrationOffsets(adafruit_bno055_offsets_t& offsets) {
    if (!imuAvailable) return false;
    bno.getSensorOffsets(offsets);
    return true;
}

bool setIMUCalibrationOffsets(const adafruit_bno055_offsets_t& offsets) {
    if (!imuAvailable) return false;
    bno.setSensorOffsets(offsets);
    return true;
}

// ============================================
// Motor Gain Calibration
// ============================================

void setMotorGains(const float fwd[4], const float rev[4]) {
    for (int i = 0; i < 4; i++) {
        motorGainFwd[i] = fwd[i];
        motorGainRev[i] = rev[i];
    }
    Serial.printf("Motor gains fwd: [%.4f, %.4f, %.4f, %.4f]\n",
                  motorGainFwd[0], motorGainFwd[1], motorGainFwd[2], motorGainFwd[3]);
    Serial.printf("Motor gains rev: [%.4f, %.4f, %.4f, %.4f]\n",
                  motorGainRev[0], motorGainRev[1], motorGainRev[2], motorGainRev[3]);
}

void getMotorGains(float fwd[4], float rev[4]) {
    for (int i = 0; i < 4; i++) {
        fwd[i] = motorGainFwd[i];
        rev[i] = motorGainRev[i];
    }
}
