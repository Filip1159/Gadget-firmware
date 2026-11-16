#include <CommandHandler.h>
#include <Protocol.h>
#include <SensorCalibration.h>

static String toString(const std::string& text) {
    return String(text.c_str());
}

void CommandHandler::reject(const String& reason) {
    Serial.println("Rejected: " + reason);
    connection.send("ERR " + reason);
}

void CommandHandler::handle(const String& message) {
    if (message.startsWith("SPEED ")) {
        long speed;
        if (!parseUint(message.c_str() + 6, speed)) {
            reject("invalid SPEED");
            return;
        }
        steering.setSpeed(constrain(speed, 0L, (long)MAX_SPEED));
    } else if (message.startsWith("PID ")) {
        float P, I, D;
        if (!parsePidArgs(message.c_str() + 4, P, I, D)) {
            reject("invalid PID");
            return;
        }
        Serial.printf("PID set: P=%f I=%f D=%f\n", P, I, D);
        steering.setPid(P, I, D);
    } else if (message.startsWith("CAL ")) {
        startCalibration(message.substring(4));
    } else if (message == "CAL_STOP") {
        if (calibration.active()) {
            finishCalibration();
        }
    } else if (message == "IMU_CHECK") {
        startImuCheck();
    } else if (message == "IMU_CHECK_STOP") {
        if (imuCheck.active()) {
            abortImuCheck();
        }
    } else if (message == "POSE_RESET") {
        steering.resetPose();
        Serial.println("Pose reset");
        connection.send(poseFrame());
    } else if (message == "MEASURE_START") {
        if (calibration.active()) {
            reject("calibration running");
            return;
        }
        continuousMeasurement = true;
        Serial.println("Continuous measurement started");
    } else if (message == "MEASURE_STOP") {
        continuousMeasurement = false;
        Serial.println("Continuous measurement stopped");
    } else if (message == "MEASURE") {
        if (calibration.active()) {
            reject("calibration running");
            return;
        }
        // One-time measurement on Core 0 (non-blocking)
        distance.requestMeasurement();
    } else {
        if (imuCheck.active()) {
            abortImuCheck();  // the person driving wins; the robot is stopped before the command applies
        }
        if (!steering.setDirection(message)) {
            reject("unknown command");
            return;
        }
    }
}

void CommandHandler::onClientDisconnected() {
    steering.stop();
    continuousMeasurement = false;
    calibration.end();  // nobody is listening any more
    imuCheck.abort();   // the robot was already stopped above
}

void CommandHandler::sendState(uint8_t clientNum) {
    float P, I, D;
    steering.getPid(P, I, D);
    connection.sendTo(clientNum, toString(formatState(steering.getSpeed(), P, I, D, continuousMeasurement)));
    connection.sendTo(clientNum, poseFrame());
}

String CommandHandler::poseFrame() {
    return toString(formatPose(steering.getPose(), millis()));
}

std::string CommandHandler::sweepFields(uint16_t result[SENSOR_COUNT], bool corrected) {
    const char* names[SENSOR_COUNT];
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        names[i] = distance.sensorName(i);
        if (corrected) {
            result[i] = correctReading(i, result[i]);  // sensorsOrder is the SensorDirection order
        }
    }
    return formatSweep(names, result, SENSOR_COUNT);
}

// Two frames per sweep, in this order: "SWEEP_POSE ..." then "MEASURE FRONT=412 FRONT_LEFT_45=OOR ..."
// The SWEEP_POSE carries the pose at the start and at the end of the sweep (a sweep takes ~390 ms and the
// robot may move during it), so the UI can interpolate a pose per sensor reading.
void CommandHandler::broadcastSweep() {
    uint16_t result[SENSOR_COUNT];
    StampedPose start, end;
    distance.getLatestSweep(result, start, end);
    connection.send(toString(formatSweepPose(start, end)));
    connection.send("MEASURE" + toString(sweepFields(result, true)));
    distance.clearMeasurementReady();
}

// "CAL <distance_mm> <count>": ranges <count> sweeps back to back with the robot standing still and
// streams each as "CAL <distance_mm> <i>/<count> FRONT=412 ...", then "CAL_DONE <distance_mm> <done>".
// The UI records them; tools/analyze_sensors.py turns the recording into bias/noise/range numbers.
void CommandHandler::startCalibration(const String& args) {
    switch (calibration.start(args.c_str(), steering.isMoving())) {
        case CalibrationRun::StartResult::Invalid:
            reject("invalid CAL");
            return;
        case CalibrationRun::StartResult::AlreadyRunning:
            reject("calibration running");
            return;
        case CalibrationRun::StartResult::RobotMoving:
            reject("robot moving");
            return;
        case CalibrationRun::StartResult::Started:
            break;
    }
    continuousMeasurement = false;  // the sweeps below belong to the calibration run only
    distance.requestMeasurement();
    Serial.printf("Calibration started: %u mm, %u sweeps\n", calibration.distanceMm(), calibration.total());
}

void CommandHandler::finishCalibration() {
    calibration.end();
    connection.send(toString(formatCalDone(calibration.distanceMm(), calibration.done())));
    Serial.printf("Calibration done: %u sweeps\n", calibration.done());
}

void CommandHandler::calibrationLoop() {
    if (distance.isMeasurementReady()) {
        const bool last = calibration.recordSweep();
        uint16_t result[SENSOR_COUNT];
        distance.getLatestMeasurements(result);
        connection.send(toString(formatCal(
            calibration.distanceMm(), calibration.done(), calibration.total(), sweepFields(result, false)
        )));
        distance.clearMeasurementReady();
        if (last) {
            finishCalibration();
        } else {
            distance.requestMeasurement();
        }
    }
}

// "IMU_CHECK": the robot stands still for ~1 s, then spins LEFT in place for ~2 s while the IMU is read,
// and answers "IMU_ORIENT RESULT=<verdict> UP=<axis> YAW=<axis> TILT=<deg> RATE=<deg/s>". RESULT=ok means
// the board's up axis (from gravity) and its spin axis (from the spin) agree; YAW is then the sensor axis
// (with sign) whose rate is the robot's counter-clockwise yaw rate.
void CommandHandler::startImuCheck() {
    if (!imu.isOnline()) {
        reject("imu offline");
        return;
    }
    if (imuCheck.active()) {
        reject("imu check running");
        return;
    }
    if (calibration.active()) {
        reject("calibration running");
        return;
    }
    if (steering.isMoving()) {
        reject("robot moving");
        return;
    }
    continuousMeasurement = false;
    lastImuCheckMillis = 0;
    imuCheck.start(millis());
    Serial.println("IMU check started: keep the robot still, then it spins LEFT in place");
}

void CommandHandler::abortImuCheck() {
    imuCheck.abort();
    steering.stop();
    const String frame = toString(formatImuOrient(imuCheck.result()));
    Serial.println(frame);
    connection.send(frame);
}

void CommandHandler::imuCheckLoop() {
    const unsigned long now = millis();
    if (now - lastImuCheckMillis < IMU_CHECK_PERIOD_MS) {
        return;
    }
    lastImuCheckMillis = now;

    ImuSample sample;
    const bool haveSample = imu.read(sample);
    const bool finished = imuCheck.update(now, haveSample ? &sample : nullptr);
    if (finished) {
        steering.stop();
        const String frame = toString(formatImuOrient(imuCheck.result()));
        Serial.println(frame);
        connection.send(frame);
    } else if (imuCheck.wantSpin()) {
        steering.setDirection("LEFT");  // also refreshes the 1 s command timeout
    }
}

void CommandHandler::loop() {
    unsigned long poseNow = millis();
    if (poseNow - lastPoseMillis >= POSE_INTERVAL_MS) {
        lastPoseMillis = poseNow;
        connection.send(poseFrame());  // goes to ready clients only
    }

    if (imuCheck.active()) {
        imuCheckLoop();
    }

    if (calibration.active()) {
        calibrationLoop();
        return;
    }

    unsigned long now = millis();

    // Continuous measurement mode: request a new sweep every MEASUREMENT_INTERVAL_MS
    if (continuousMeasurement && now - lastMeasurementRequestMillis >= MEASUREMENT_INTERVAL_MS) {
        lastMeasurementRequestMillis = now;
        if (!distance.isMeasurementReady() && !distance.isMeasurementBusy()) {
            distance.requestMeasurement();
        }
    }

    if (distance.isMeasurementReady()) {
        broadcastSweep();
    }
}
