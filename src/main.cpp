#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <SteeringManager.h>

ConnectionManager connectionManager = ConnectionManager();
DistanceManager distanceManager = DistanceManager();
SteeringManager steeringManager = SteeringManager();

unsigned long previousSteeringMillis = 0;
unsigned long previousMillis = 0;
unsigned long previousMeasurementMillis = 0;
bool ledState = false;
bool continuousMeasurement = false;

// Failsafe: the frontend repeats the active direction every 500ms while a button is held.
// If nothing arrives within this window, the robot stops itself.
#define COMMAND_TIMEOUT_MS 1000
#define MAX_SPEED 2000
unsigned long lastDirectionCommandMillis = 0;

// Strict unsigned integer parse; returns false for empty/non-numeric/negative input.
bool parseUint(const String& text, long& value) {
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

void updateSteering(String message) {
    if (message.startsWith("SPEED ")) {
        long speed;
        if (!parseUint(message.substring(6), speed)) {
            Serial.println("Rejected invalid SPEED: " + message);
            return;
        }
        steeringManager.setSpeed(constrain(speed, 0L, (long)MAX_SPEED));
    } else if (message.startsWith("PID ")) {
        float P, I, D;
        char extra;
        if (sscanf(message.c_str() + 4, "%f %f %f %c", &P, &I, &D, &extra) != 3 || !isfinite(P) || !isfinite(I) ||
            !isfinite(D) || P < 0 || I < 0 || D < 0) {
            Serial.println("Rejected invalid PID: " + message);
            return;
        }
        Serial.printf("PID set: P=%f I=%f D=%f\n", P, I, D);
        steeringManager.setPid(P, I, D);
    } else if (message == "MEASURE_START") {
        // Start continuous measurement mode
        continuousMeasurement = true;
        Serial.println("Continuous measurement started");
    } else if (message == "MEASURE_STOP") {
        // Stop continuous measurement mode
        continuousMeasurement = false;
        Serial.println("Continuous measurement stopped");
    } else if (message == "MEASURE") {
        // One-time measurement on Core 0 (non-blocking)
        distanceManager.requestMeasurement();
    } else {
        if (!steeringManager.setDirection(message)) {
            Serial.println("Rejected unknown command: " + message);
            return;
        }
        lastDirectionCommandMillis = millis();
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Booting");

    pinMode(LED_GREEN, OUTPUT);

    connectionManager.initWiFi();
    connectionManager.setOnDisconnect([]() {
        steeringManager.stop();
        continuousMeasurement = false;  // don't keep sweeping/broadcasting to nobody
    });
    connectionManager.setOnOtaStart([]() { steeringManager.stop(); });
    connectionManager.initOta();
    connectionManager.initWebsocket(updateSteering);
    distanceManager.initAddresses();
    distanceManager.startMeasurementTask();  // Start measurement task on Core 0
    steeringManager.init();
}

void loop() {
    connectionManager.loop();
    unsigned long currentMillis = millis();

    // Continuous measurement mode - request new measurements every 200ms
    if (continuousMeasurement && currentMillis - previousMeasurementMillis >= 200) {
        previousMeasurementMillis = currentMillis;
        if (!distanceManager.isMeasurementReady() && !distanceManager.isMeasurementBusy()) {
            distanceManager.requestMeasurement();
        }
    }

    // Check if measurement is ready and send results
    if (distanceManager.isMeasurementReady()) {
        uint16_t result[10];
        distanceManager.getLatestMeasurements(result);
        // One frame per sweep: "MEASURE FRONT=412 FRONT_LEFT_45=OOR ..."
        String frame = "MEASURE";
        for (uint8_t i = 0; i < 10; i++) {
            frame += " " + distanceManager.directionToString(i) + "=";
            frame += (result[i] == OUT_OF_RANGE_MM) ? String("OOR") : String(result[i]);
        }
        connectionManager.send(frame);
        distanceManager.clearMeasurementReady();
    }

    if (steeringManager.isMoving() && currentMillis - lastDirectionCommandMillis >= COMMAND_TIMEOUT_MS) {
        Serial.println("Command timeout, stopping");
        steeringManager.stop();
    }

    if (currentMillis - previousSteeringMillis >= 100) {
        previousSteeringMillis = currentMillis;
        steeringManager.loop();
    }

    if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_GREEN, ledState);
    }
}
