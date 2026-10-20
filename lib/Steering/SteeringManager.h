#pragma once

#include <Arduino.h>
#include <DriveDirection.h>
#include <PidController.h>
#include <driver/pcnt.h>

#define LEFT_1_PIN 5
#define LEFT_2_PIN 4
#define RIGHT_1_PIN 3
#define RIGHT_2_PIN 2

// ESP-IDF numbers, not Arduino
#define LEFT_ENCODER_A 17  // Arduino 8
#define LEFT_ENCODER_B 18  // Arduino 7
#define RIGHT_ENCODER_A 1  // Arduino A0
#define RIGHT_ENCODER_B 2  // Arduino A1

class SteeringManager {
   private:
    PidController* left;
    PidController* right;
    uint16_t speed = 1000;
    DriveDirection direction = STOP;

   public:
    SteeringManager();
    void init();
    void loop();
    void resetPids();
    void setSpeed(uint16_t speed) {
        this->speed = speed;
    }
    uint16_t getSpeed() const {
        return speed;
    }
    // Returns false (and leaves the current direction untouched) for unknown names.
    bool setDirection(const String& name);
    bool isMoving() const {
        return direction != STOP;
    }
    // Immediately stops both motors (does not wait for the next steering tick).
    void stop();
    void setPid(float P, float I, float D);
    // Both wheels always share the same gains.
    const PidController* getPid() const {
        return left;
    }
};
