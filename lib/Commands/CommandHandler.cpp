#include <CommandHandler.h>
#include <SensorCalibration.h>

// Strict unsigned integer parse; returns false for empty/non-numeric/negative input.
static bool parseUint(const String& text, long& value) {
    String trimmed = text;
    trimmed.trim();
    if (trimmed.length() == 0) {
        return false;
    }
    for (unsigned int i = 0; i < trimmed.length(); i++) {
        if (!isDigit(trimmed[i])) {
            return false;
        }
    }
    value = trimmed.toInt();
    return true;
}

void CommandHandler::reject(const String& reason) {
    Serial.println("Rejected: " + reason);
    connection.send("ERR " + reason);
}

void CommandHandler::handle(const String& message) {
    if (message.startsWith("SPEED ")) {
        long speed;
        if (!parseUint(message.substring(6), speed)) {
            reject("invalid SPEED");
            return;
        }
        steering.setSpeed(constrain(speed, 0L, (long)MAX_SPEED));
    } else if (message.startsWith("PID ")) {
        float P, I, D;
        char extra;
        if (sscanf(message.c_str() + 4, "%f %f %f %c", &P, &I, &D, &extra) != 3 || !isfinite(P) || !isfinite(I) ||
            !isfinite(D) || P < 0 || I < 0 || D < 0) {
            reject("invalid PID");
            return;
        }
        Serial.printf("PID set: P=%f I=%f D=%f\n", P, I, D);
        steering.setPid(P, I, D);
    } else if (message.startsWith("CAL ")) {
        startCalibration(message.substring(4));
    } else if (message == "CAL_STOP") {
        if (calibrating) {
            finishCalibration();
        }
    } else if (message == "POSE_RESET") {
        steering.resetPose();
        Serial.println("Pose reset");
        connection.send(poseFrame());
    } else if (message == "MEASURE_START") {
        if (calibrating) {
            reject("calibration running");
            return;
        }
        continuousMeasurement = true;
        Serial.println("Continuous measurement started");
    } else if (message == "MEASURE_STOP") {
        continuousMeasurement = false;
        Serial.println("Continuous measurement stopped");
    } else if (message == "MEASURE") {
        if (calibrating) {
            reject("calibration running");
            return;
        }
        // One-time measurement on Core 0 (non-blocking)
        distance.requestMeasurement();
    } else {
        if (!steering.setDirection(message)) {
            reject("unknown command");
            return;
        }
    }
}

void CommandHandler::onClientDisconnected() {
    steering.stop();
    continuousMeasurement = false;
    calibrating = false;  // nobody is listening any more
}

void CommandHandler::sendState(uint8_t clientNum) {
    float P, I, D;
    steering.getPid(P, I, D);
    char buffer[96];
    snprintf(
        buffer, sizeof(buffer), "STATE SPEED=%u P=%.4f I=%.4f D=%.4f MEASURING=%d", steering.getSpeed(), P, I, D,
        continuousMeasurement ? 1 : 0
    );
    connection.sendTo(clientNum, buffer);
    connection.sendTo(clientNum, poseFrame());
}

// X/Y in mm, TH in milliradians (counter-clockwise), T = robot millis(), Q=1 will flag a suspected
// wheel slip/stall (MAPPING.MD 1.5; always 0 until then).
String CommandHandler::poseFrame() {
    const Pose pose = steering.getPose();
    char buffer[80];
    snprintf(
        buffer, sizeof(buffer), "POSE X=%ld Y=%ld TH=%ld T=%lu Q=0", lroundf(pose.x), lroundf(pose.y),
        lroundf(pose.theta * 1000.0f), millis()
    );
    return String(buffer);
}

String CommandHandler::sweepFields(bool corrected) {
    uint16_t result[SENSOR_COUNT];
    distance.getLatestMeasurements(result);
    String fields;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (corrected) {
            result[i] = correctReading(i, result[i]);  // sensorsOrder is the SensorDirection order
        }
        fields += " " + distance.directionToString(i) + "=";
        fields += (result[i] == OUT_OF_RANGE_MM) ? String("OOR") : String(result[i]);
    }
    return fields;
}

// One frame per sweep: "MEASURE FRONT=412 FRONT_LEFT_45=OOR ..."
void CommandHandler::broadcastSweep() {
    connection.send("MEASURE" + sweepFields(true));
    distance.clearMeasurementReady();
}

// "CAL <distance_mm> <count>": ranges <count> sweeps back to back with the robot standing still and
// streams each as "CAL <distance_mm> <i>/<count> FRONT=412 ...", then "CAL_DONE <distance_mm> <done>".
// The UI records them; tools/analyze_sensors.py turns the recording into bias/noise/range numbers.
void CommandHandler::startCalibration(const String& args) {
    long distanceMm, count;
    int space = args.indexOf(' ');
    if (space < 0 || !parseUint(args.substring(0, space), distanceMm) || !parseUint(args.substring(space + 1), count) ||
        distanceMm < 1 || distanceMm > CAL_MAX_DISTANCE_MM || count < 1 || count > CAL_MAX_COUNT) {
        reject("invalid CAL");
        return;
    }
    if (calibrating) {
        reject("calibration running");
        return;
    }
    if (steering.isMoving()) {
        reject("robot moving");
        return;
    }
    continuousMeasurement = false;  // the sweeps below belong to the calibration run only
    calibrating = true;
    calDistanceMm = distanceMm;
    calTotal = count;
    calDone = 0;
    distance.requestMeasurement();
    Serial.printf("Calibration started: %ld mm, %ld sweeps\n", distanceMm, count);
}

void CommandHandler::finishCalibration() {
    calibrating = false;
    connection.send("CAL_DONE " + String(calDistanceMm) + " " + String(calDone));
    Serial.printf("Calibration done: %u sweeps\n", calDone);
}

void CommandHandler::calibrationLoop() {
    if (distance.isMeasurementReady()) {
        calDone++;
        connection.send(
            "CAL " + String(calDistanceMm) + " " + String(calDone) + "/" + String(calTotal) + sweepFields(false)
        );
        distance.clearMeasurementReady();
        if (calDone >= calTotal) {
            finishCalibration();
        } else {
            distance.requestMeasurement();
        }
    }
}

void CommandHandler::loop() {
    unsigned long poseNow = millis();
    if (poseNow - lastPoseMillis >= POSE_INTERVAL_MS) {
        lastPoseMillis = poseNow;
        connection.send(poseFrame());  // goes to ready clients only
    }

    if (calibrating) {
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
