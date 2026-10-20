#include <CommandHandler.h>

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
    } else if (message == "MEASURE_START") {
        continuousMeasurement = true;
        Serial.println("Continuous measurement started");
    } else if (message == "MEASURE_STOP") {
        continuousMeasurement = false;
        Serial.println("Continuous measurement stopped");
    } else if (message == "MEASURE") {
        // One-time measurement on Core 0 (non-blocking)
        distance.requestMeasurement();
    } else {
        if (!steering.setDirection(message)) {
            reject("unknown command");
            return;
        }
        lastDirectionCommandMillis = millis();
    }
}

void CommandHandler::onClientDisconnected() {
    steering.stop();
    continuousMeasurement = false;
}

void CommandHandler::sendState(uint8_t clientNum) {
    const PidController* pid = steering.getPid();
    char buffer[96];
    snprintf(
        buffer, sizeof(buffer), "STATE SPEED=%u P=%.4f I=%.4f D=%.4f MEASURING=%d", steering.getSpeed(), pid->getKp(),
        pid->getKi(), pid->getKd(), continuousMeasurement ? 1 : 0
    );
    connection.sendTo(clientNum, buffer);
}

// One frame per sweep: "MEASURE FRONT=412 FRONT_LEFT_45=OOR ..."
void CommandHandler::broadcastSweep() {
    uint16_t result[SENSOR_COUNT];
    distance.getLatestMeasurements(result);
    String frame = "MEASURE";
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        frame += " " + distance.directionToString(i) + "=";
        frame += (result[i] == OUT_OF_RANGE_MM) ? String("OOR") : String(result[i]);
    }
    connection.send(frame);
    distance.clearMeasurementReady();
}

void CommandHandler::loop() {
    unsigned long now = millis();

    if (steering.isMoving() && now - lastDirectionCommandMillis >= COMMAND_TIMEOUT_MS) {
        Serial.println("Command timeout, stopping");
        steering.stop();
    }

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
