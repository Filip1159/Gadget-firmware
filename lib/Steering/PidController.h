#pragma once

#include <Arduino.h>
#include <DriveDirection.h>
#include <EncoderTracker.h>
#include <PidMath.h>
#include <driver/pcnt.h>
#include <stdint.h>

class PidController {
   private:
    PidGains gains = {0.15f, 0.05f, 0.0f};

    pcnt_unit_t pcntUnit;
    uint8_t motorPin1;
    uint8_t motorPin2;

    PidState state;
    WheelDirection direction = WHEEL_FORWARD;
    int16_t targetSpeed = 0;
    unsigned long lastLoopMicros = 0;
    EncoderTracker encoder;
    int8_t encoderSign;
    int16_t lastSignedDelta = 0;

   public:
    // encoderSign (+1 or -1) makes the signed tick delta positive when the wheel drives the robot
    // forward; the motors are mirrored, so it is per wheel.
    PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2, int8_t encoderSign = 1);
    void loop();
    void reset();
    void setPid(float P, float I, float D) {
        reset();
        gains = {P, I, D};
    }
    // Signed encoder ticks seen by the last loop() (positive = robot-forward). Counted also while
    // stopped, so coasting is included. Read under the same lock that guards loop().
    int16_t getSignedDelta() const {
        return lastSignedDelta;
    }
    float getKp() const {
        return gains.kp;
    }
    float getKi() const {
        return gains.ki;
    }
    float getKd() const {
        return gains.kd;
    }
    void setTargetSpeed(WheelDirection direction, uint16_t targetSpeed) {
        this->direction = direction;
        this->targetSpeed = targetSpeed;
    }
};
