#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <SteeringManager.h>

ConnectionManager connectionManager = ConnectionManager();
DistanceManager distanceManager = DistanceManager();
SteeringManager steeringManager = SteeringManager();

unsigned long previousSteeringMillis = 0;
unsigned long previousMillis = 0;
bool ledState = false;

void read_dual_sensors() {
    uint16_t result[10];
    distanceManager.measureAll(result);
    for (uint8_t i = 0; i < 10; i++) {
        connectionManager.send(String(distanceManager.directionToString(i)) + " " + result[i]);
    }
}

void updateSteering(String message) {
    if (message.startsWith("SPEED")) {
        steeringManager.setSpeed(message.substring(5).toInt());
    } else if (message.startsWith("PID")) {
        message = message.substring(4);
        String P = message.substring(0, message.indexOf(" "));
        message = message.substring(P.length() + 1);
        String I = message.substring(0, message.indexOf(" "));
        message = message.substring(I.length() + 1);
        String D = message.substring(0, message.indexOf(" "));
        Serial.println(P);
        Serial.println(I);
        Serial.println(D);
        steeringManager.setPid(P.toFloat(), I.toFloat(), D.toFloat());
    } else {
        steeringManager.resetPids();
        steeringManager.setDirection(message);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Booting");

    pinMode(LED_GREEN, OUTPUT);

    connectionManager.initWiFi();
    connectionManager.initOta();
    connectionManager.initWebsocket(updateSteering);
    distanceManager.initAddresses();
    steeringManager.init();
}

void loop() {
    connectionManager.loop();
    unsigned long currentMillis = millis();

    if (currentMillis - previousSteeringMillis >= 100) {
        previousSteeringMillis = currentMillis;
        steeringManager.loop();
    }

    if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_GREEN, ledState);

        // read_dual_sensors();
    }
}
