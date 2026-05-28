#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Adafruit_BNO055.h>

// ============================================
// Sensor Data Structures
// ============================================

struct EncoderData {
    int32_t counts[4];           // Raw encoder counts
    float velocities[4];         // Wheel velocities in rad/s
    uint32_t timestamp;          // Timestamp in ms
};

struct IMUData {
    float yaw;                   // Orientation around Z axis (degrees)
    float pitch;                 // Orientation around Y axis (degrees)
    float roll;                  // Orientation around X axis (degrees)
    float gyro_x;                // Angular velocity X (rad/s)
    float gyro_y;                // Angular velocity Y (rad/s)
    float gyro_z;                // Angular velocity Z (rad/s)
    float accel_x;               // Linear acceleration X (m/s^2)
    float accel_y;               // Linear acceleration Y (m/s^2)
    float accel_z;               // Linear acceleration Z (m/s^2)
    uint8_t cal_system;          // Calibration status 0-3
    uint8_t cal_gyro;
    uint8_t cal_accel;
    uint8_t cal_mag;
    uint32_t timestamp;
};

// ============================================
// Function Declarations
// ============================================

// Initialization
bool initEncoders();
bool initIMU();

// Encoder functions
int32_t getEncoderCount(int motorIndex);
void resetEncoder(int motorIndex);
void resetAllEncoders();
EncoderData readEncoders();

// Odometry-dedicated encoder reading (own delta tracking, independent of
// broadcast and PID paths). Call at 50Hz from the motor update loop.
EncoderData readOdomEncoders();
void resetOdomEncoders();

// Cached sensor snapshot — updated once per motor-update cycle in main loop,
// shared across broadcast, odometry, and heading-hold to eliminate redundant
// I2C and PCNT reads.
struct CachedSensors {
    EncoderData enc;
    IMUData imu;
    bool imuValid;   // true if IMU was available when snapshot was taken
};

// Call once per 50Hz motor-update cycle to refresh the cache.
void updateSensorCache();

// Get the cached snapshot (read-only).
const CachedSensors& getSensorCache();

// IMU functions
IMUData readIMU();
bool isIMUAvailable();
void zeroIMU();

// IMU stuck-read watchdog.
// Detects the BNO055 lock-up failure mode where the chip returns
// bit-identical cached values indefinitely (seen 2026-04-18 — only a
// hardware power cycle recovered it). When gyro_z is bit-identical for
// IMU_STUCK_THRESHOLD consecutive updates, the flag latches true and
// stays true until the ESP32 reboots, so the UI can prompt for a power
// cycle. A genuinely stationary BNO055 still produces sub-LSB noise on
// gyro_z, so bit-identical reads are a reliable stuck signature.
bool isIMUStuck();

// IMU calibration persistence
bool getIMUCalibrationOffsets(adafruit_bno055_offsets_t& offsets);
bool setIMUCalibrationOffsets(const adafruit_bno055_offsets_t& offsets);

// Motor gain calibration (per-direction: separate forward/reverse gains)
void setMotorGains(const float fwd[4], const float rev[4]);
void getMotorGains(float fwd[4], float rev[4]);

#endif // SENSORS_H
