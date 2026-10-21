#pragma once

#include <Arduino.h>
#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <SteeringManager.h>

#define MEASUREMENT_INTERVAL_MS 200
#define MAX_SPEED 2000

// Pose broadcast period while clients are connected.
#define POSE_INTERVAL_MS 200

// Sensor characterisation (MAPPING.MD item 0.4): `CAL <distance_mm> <count>` ranges <count> sweeps in a row.
#define CAL_MAX_DISTANCE_MM 2000
#define CAL_MAX_COUNT 500

// Parses frontend commands, dispatches them to the managers, and owns the robot-level
// behaviour that hangs off them: continuous measurement and the sweep broadcast. (The command
// timeout failsafe lives in the steering control task.)
// Wire format: see CLAUDE.md. Rejected commands are answered with "ERR <reason>".
class CommandHandler {
   private:
    SteeringManager& steering;
    DistanceManager& distance;
    ConnectionManager& connection;

    bool continuousMeasurement = false;
    unsigned long lastMeasurementRequestMillis = 0;
    unsigned long lastPoseMillis = 0;

    // Characterisation run (CAL): true between CAL and its CAL_DONE.
    bool calibrating = false;
    uint16_t calDistanceMm = 0;
    uint16_t calTotal = 0;
    uint16_t calDone = 0;

    void reject(const String& reason);
    void broadcastSweep();
    String poseFrame();  // "POSE X=<mm> Y=<mm> TH=<mrad> T=<ms> Q=0"
    // " FRONT=412 FRONT_LEFT_45=OOR ..." for the latest sweep; `corrected` applies the per-sensor
    // calibration curve (MEASURE frames), raw is what CAL frames record.
    String sweepFields(bool corrected);
    void startCalibration(const String& args);
    void calibrationLoop();
    void finishCalibration();

   public:
    CommandHandler(SteeringManager& steering, DistanceManager& distance, ConnectionManager& connection)
        : steering(steering), distance(distance), connection(connection) {}

    void handle(const String& message);

    // A client went away: stop the robot and stop sweeping for nobody.
    void onClientDisconnected();

    // Also sends the current POSE to that client.
    // "STATE SPEED=<n> P=<f> I=<f> D=<f> MEASURING=<0|1>", so the UI can initialise from the robot.
    void sendState(uint8_t clientNum);

    void loop();
};
