#pragma once

#include <Arduino.h>
#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <SteeringManager.h>

// Failsafe: the frontend repeats the active direction every 500ms while a button/key is held.
// If nothing arrives within this window, the robot stops itself.
#define COMMAND_TIMEOUT_MS 1000
#define MEASUREMENT_INTERVAL_MS 200
#define MAX_SPEED 2000

// Parses frontend commands, dispatches them to the managers, and owns the robot-level
// behaviour that hangs off them: command timeout, continuous measurement, sweep broadcast.
// Wire format: see CLAUDE.md. Rejected commands are answered with "ERR <reason>".
class CommandHandler {
   private:
    SteeringManager& steering;
    DistanceManager& distance;
    ConnectionManager& connection;

    bool continuousMeasurement = false;
    unsigned long lastDirectionCommandMillis = 0;
    unsigned long lastMeasurementRequestMillis = 0;

    void reject(const String& reason);
    void broadcastSweep();

   public:
    CommandHandler(SteeringManager& steering, DistanceManager& distance, ConnectionManager& connection)
        : steering(steering), distance(distance), connection(connection) {}

    void handle(const String& message);

    // A client went away: stop the robot and stop sweeping for nobody.
    void onClientDisconnected();

    // "STATE SPEED=<n> P=<f> I=<f> D=<f> MEASURING=<0|1>", so the UI can initialise from the robot.
    void sendState(uint8_t clientNum);

    void loop();
};
