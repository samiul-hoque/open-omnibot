#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>

// ============================================
// Function Declarations
// ============================================

// Initialization
bool initMotors(Adafruit_MCP23X17* mcpPtr);

// Individual motor control
// speed: -255 to 255
void setMotorSpeed(int motorIndex, int speed);
void stopMotor(int motorIndex);

// All motors
void stopAllMotors();

// Set individual wheel speeds (for direct control)
// speeds in range -255 to 255
void setWheelSpeeds(int speedL1, int speedR1, int speedR2, int speedL2);

#endif // MOTORS_H
