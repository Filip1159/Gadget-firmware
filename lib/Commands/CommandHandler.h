#pragma once

#include <Arduino.h>
#include <CalibrationRun.h>
#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <SteeringManager.h>

#define MEASUREMENT_INTERVAL_MS 200
#define MAX_SPEED 2000

// Pose broadcast period while clients are connected.
#define POSE_INTERVAL_MS 200

// Dispatches frontend commands to the managers and owns the robot-level behaviour that hangs off them:
// continuous measurement and the sweep broadcast. The wire format itself (frame text, strict argument
// parsing) lives in lib/Protocol and the CAL run bookkeeping in lib/CalibrationRun, both host-tested.
// (The command timeout failsafe lives in the steering control task.)
// Wire format: see CLAUDE.md. Rejected commands are answered with "ERR <reason>".
class CommandHandler {
   private:
    SteeringManager& steering;
    DistanceManager& distance;
    ConnectionManager& connection;

    bool continuousMeasurement = false;
    unsigned long lastMeasurementRequestMillis = 0;
    unsigned long lastPoseMillis = 0;

    // Characterisation run (CAL): active between CAL and its CAL_DONE.
    CalibrationRun calibration;

    void reject(const String& reason);
    void broadcastSweep();
    String poseFrame();
    // " FRONT=412 FRONT_LEFT_45=OOR ..." for a sweep; `corrected` applies the per-sensor calibration
    // curve (MEASURE frames), raw is what CAL frames record.
    std::string sweepFields(uint16_t result[SENSOR_COUNT], bool corrected);
    void startCalibration(const String& args);
    void calibrationLoop();
    void finishCalibration();

   public:
    CommandHandler(SteeringManager& steering, DistanceManager& distance, ConnectionManager& connection)
        : steering(steering), distance(distance), connection(connection) {}

    void handle(const String& message);

    // A client went away: stop the robot and stop sweeping for nobody.
    void onClientDisconnected();

    // Sends "STATE ..." and the current POSE to that client, so the UI can initialise from the robot.
    void sendState(uint8_t clientNum);

    void loop();
};
