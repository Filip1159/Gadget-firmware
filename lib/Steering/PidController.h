#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>
#include <stdint.h>

class PidController {
   private:
    float Kp = 0.22;
    float Ki = 0.08;
    float Kd = 0.0;

    pcnt_unit_t pcntUnit;
    uint8_t motorPin1;
    uint8_t motorPin2;

    float integral = 0;
    float previousError = 0;
    int16_t targetSpeed = 1000;
    uint16_t lastEncoderValue = 0;

   public:
    PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2);
    void loop();
    void setTargetSpeed(int16_t targetSpeed) {
        this->targetSpeed = targetSpeed;
    }
};
