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
    WheelDirection direction;
    int16_t targetSpeed = 1000;

   public:
    PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2);
    void loop();
    void reset();
    void setTargetSpeed(WheelDirection direction, uint16_t targetSpeed) {
        this->direction = direction;
        this->targetSpeed = targetSpeed;
    }
};
