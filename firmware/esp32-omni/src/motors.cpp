#include "motors.h"
#include "config.h"

// ============================================
// Private Variables
// ============================================

static Adafruit_MCP23X17* mcp = nullptr;

// Motor configurations - indices 2 and 3 swapped to match physical wiring
// Physical: [L1, R1, L2, R2] at indices [0, 1, 2, 3]
static const uint8_t pwmPins[4] = {MOTOR_L1_PWM, MOTOR_R1_PWM, MOTOR_L2_PWM, MOTOR_R2_PWM};
static const uint8_t in1Pins[4] = {MOTOR_L1_IN1, MOTOR_R1_IN1, MOTOR_L2_IN1, MOTOR_R2_IN1};
static const uint8_t in2Pins[4] = {MOTOR_L1_IN2, MOTOR_R1_IN2, MOTOR_L2_IN2, MOTOR_R2_IN2};
// Direction corrections - indices 2 and 3 swapped to match physical wiring
static const int8_t motorDirs[4] = {MOTOR_L1_DIR, MOTOR_R1_DIR, MOTOR_L2_DIR, MOTOR_R2_DIR};
static const char* motorNames[4] = {"L1", "R1", "L2", "R2"};

// Slew rate limiter state
static int16_t lastPWM[4] = {0, 0, 0, 0};
static bool forceStop = false;

// Cache of last-written MCP23017 direction state per motor.
//   +1  : forward (IN1 HIGH, IN2 LOW)
//   -1  : reverse (IN1 LOW,  IN2 HIGH)
//    0  : coast   (both LOW)
// Initialised to 0 to match initMotors() which writes both pins LOW.
// We only push MCP I2C writes when the direction actually changes,
// saving two I2C transactions per motor per tick during steady-state motion.
static int8_t lastDir[4] = {0, 0, 0, 0};

// ============================================
// Initialization
// ============================================

bool initMotors(Adafruit_MCP23X17* mcpPtr) {
    if (mcpPtr == nullptr) {
        Serial.println("ERROR: MCP23017 pointer is null!");
        return false;
    }
    
    mcp = mcpPtr;
    
    Serial.println("Initializing motors...");
    
    // Initialize standby pins (active HIGH to enable motors)
    mcp->pinMode(STBY_FRONT, OUTPUT);
    mcp->pinMode(STBY_REAR, OUTPUT);
    mcp->digitalWrite(STBY_FRONT, HIGH);
    mcp->digitalWrite(STBY_REAR, HIGH);
    
    // Initialize each motor
    for (int i = 0; i < 4; i++) {
        // PWM pin setup (LEDC channel = motor index)
        ledcSetup(i, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(pwmPins[i], i);
        
        // Direction pins on MCP23017
        mcp->pinMode(in1Pins[i], OUTPUT);
        mcp->pinMode(in2Pins[i], OUTPUT);
        mcp->digitalWrite(in1Pins[i], LOW);
        mcp->digitalWrite(in2Pins[i], LOW);
        
        Serial.printf("Motor %s: PWM=GPIO%d, IN1=%d, IN2=%d\n",
                      motorNames[i], pwmPins[i], in1Pins[i], in2Pins[i]);
    }
    
    Serial.println("Motors initialized");
    return true;
}

// ============================================
// Motor Control Functions
// ============================================

void setMotorSpeed(int motorIndex, int speed) {
    if (motorIndex < 0 || motorIndex >= 4 || mcp == nullptr) return;

    // Apply direction correction for motor orientation
    speed = speed * motorDirs[motorIndex];

    // Constrain speed
    speed = constrain(speed, -255, 255);

    // Slew rate limiter — ramp toward target by at most MOTOR_MAX_PWM_STEP per call
    // Emergency stop bypasses the ramp
    if (!forceStop) {
        int16_t delta = speed - lastPWM[motorIndex];
        if (delta > MOTOR_MAX_PWM_STEP) {
            speed = lastPWM[motorIndex] + MOTOR_MAX_PWM_STEP;
        } else if (delta < -MOTOR_MAX_PWM_STEP) {
            speed = lastPWM[motorIndex] - MOTOR_MAX_PWM_STEP;
        }
    }
    lastPWM[motorIndex] = speed;

    // Set direction via MCP23017 — but only when it changes. At 50 Hz PID
    // with a motor running in the same direction, the previous code issued
    // two I2C writes per motor per tick (~400 I2C ops/s across 4 motors)
    // even though nothing changed.
    const int8_t desiredDir = (speed > 0) ? 1 : (speed < 0) ? -1 : 0;
    if (desiredDir != lastDir[motorIndex]) {
        if (desiredDir > 0) {
            mcp->digitalWrite(in1Pins[motorIndex], HIGH);
            mcp->digitalWrite(in2Pins[motorIndex], LOW);
        } else if (desiredDir < 0) {
            mcp->digitalWrite(in1Pins[motorIndex], LOW);
            mcp->digitalWrite(in2Pins[motorIndex], HIGH);
        } else {
            // Coast (both LOW). Both HIGH would be active brake — we don't
            // use that here to stay consistent with initMotors().
            mcp->digitalWrite(in1Pins[motorIndex], LOW);
            mcp->digitalWrite(in2Pins[motorIndex], LOW);
        }
        lastDir[motorIndex] = desiredDir;
    }

    // PWM always gets written — the LEDC register is on-chip, no I2C cost.
    ledcWrite(motorIndex, abs(speed));
}

void stopMotor(int motorIndex) {
    setMotorSpeed(motorIndex, 0);
}

void stopAllMotors() {
    forceStop = true;
    for (int i = 0; i < 4; i++) {
        stopMotor(i);
    }
    forceStop = false;
}

void setWheelSpeeds(int speedL1, int speedR1, int speedR2, int speedL2) {
    setMotorSpeed(0, speedL1);  // Motor 0 = L1
    setMotorSpeed(1, speedR1);  // Motor 1 = R1
    setMotorSpeed(2, speedL2);  // Motor 2 = L2 (swapped)
    setMotorSpeed(3, speedR2);  // Motor 3 = R2 (swapped)
}
