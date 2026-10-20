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

// Failsafe: the frontend repeats the active direction every 500ms while a button/key is held.
// If nothing arrives within this window, the control task stops the robot by itself.
#define COMMAND_TIMEOUT_MS 1000

// Control period of the PID task.
#define STEERING_PERIOD_MS 100

// Owns the two wheel PIDs. The PID runs in its own FreeRTOS task (startTask) on a fixed period,
// independent of WiFi/WebSocket work in loop(), and so does the command timeout failsafe.
// All public methods are thread-safe (one mutex); the motors and the PCNT counters are only
// touched while holding it.
class SteeringManager {
   private:
    PidController* left;
    PidController* right;
    uint16_t speed = 1000;
    DriveDirection direction = STOP;
    unsigned long lastCommandMillis = 0;
    volatile unsigned long lastTickMillis = 0;
    SemaphoreHandle_t mutex;

    static void controlTask(void* parameter);
    void tickLocked();  // mutex must be held

   public:
    SteeringManager();
    SteeringManager(const SteeringManager&) = delete;  // owns raw pointers and a mutex
    SteeringManager& operator=(const SteeringManager&) = delete;
    void init();

    // Starts the control task (Core 1, higher priority than loop()). Call after init().
    void startTask();

    // One control tick. Normally called by the task only.
    void loop();

    void resetPids();
    void setSpeed(uint16_t speed);
    uint16_t getSpeed();
    // Returns false (and leaves the current direction untouched) for unknown names.
    // A successful call also refreshes the command timeout.
    bool setDirection(const String& name);
    bool isMoving();
    // Immediately stops both motors (does not wait for the next control tick).
    void stop();
    void setPid(float P, float I, float D);
    // Both wheels always share the same gains.
    void getPid(float& P, float& I, float& D);

    // Milliseconds since the control task last completed a tick (large = the task is stuck).
    unsigned long msSinceLastTick() const {
        return millis() - lastTickMillis;
    }
};
