#pragma once

#include <Arduino.h>
#include <DriveDirection.h>
#include <driver/pcnt.h>
#include <stdint.h>

class PidController {
   private:
    float Kp = 0.15;
    float Ki = 0.05;
    float Kd = 0.0;

    pcnt_unit_t pcntUnit;
    uint8_t motorPin1;
    uint8_t motorPin2;

    float integral = 0;
    float previousError = 0;
    WheelDirection direction = WHEEL_FORWARD;
    int16_t targetSpeed = 0;
    unsigned long lastLoopMicros = 0;

   public:
    PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2);
    void loop();
    void reset();
    void setPid(float P, float I, float D) {
        reset();
        this->Kp = P;
        this->Ki = I;
        this->Kd = D;
    }
    float getKp() const {
        return Kp;
    }
    float getKi() const {
        return Ki;
    }
    float getKd() const {
        return Kd;
    }
    void setTargetSpeed(WheelDirection direction, uint16_t targetSpeed) {
        this->direction = direction;
        this->targetSpeed = targetSpeed;
    }
};
