#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <Arduino.h>

// ============================================
// Function Declarations
// ============================================

// Initialize WiFi and WebSocket server
void initWebSocket();

// Process incoming WebSocket messages (call in loop)
void handleWebSocket();

// Send sensor data to connected clients
void broadcastSensorData();

// Check if any client is connected
bool isClientConnected();

// Get last received velocity command
void getLastVelocityCommand(float* vx, float* vy, float* omega);

// Check if velocity command is still valid (not timed out)
bool isVelocityCommandValid();

// Get time since last command received
uint32_t getTimeSinceLastCommand();

// Send a formatted log message to all connected WebSocket clients
// Message format: {"type":"log","msg":"..."}
void wsLog(const char* format, ...);

// Broadcast a pre-formatted JSON string to all connected WebSocket clients
void wsBroadcastRaw(const char* json);

// Check if calibration mode is active (auto-clears on 500ms timeout)
bool isCalibrationMode();

// Auto-load IMU calibration from NVS (call after initIMU)
void autoLoadIMUCalibration();

// Auto-load motor calibration gains from NVS (call after initEncoders)
void autoLoadMotorCalibration();

// Check if NTP time is synchronized
bool isNtpSynced();

// Heading-hold: when enabled and the user commands pure translation
// (omega=0 + non-zero vx/vy), the motor loop adds a small P correction
// derived from IMU gyro_z (filtered + dead-banded) to cancel yaw drift.
// Tunables mutable via {"type":"set_heading_hold", ...}.
bool  isHeadingHoldEnabled();
float getHeadingHoldGain();
float getHeadingHoldDeadzone();
float getHeadingHoldAlpha();

// Zeros the file-scoped LPF state in main.cpp's heading-hold helper.
// Called from trajectory.cpp at trajStart/trajResume so the first few
// ticks of a translate segment don't consume stale filter bias left
// over from teleop or a prior segment. Implementation lives in main.cpp
// because the filter state is private to the heading-hold helper.
void  resetHeadingHoldFilter();

#endif // WEBSOCKET_SERVER_H
