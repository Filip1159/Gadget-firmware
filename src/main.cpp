#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <ESPmDNS.h>
#include <SteeringManager.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiMulti.h>

ConnectionManager connectionManager = ConnectionManager();
DistanceManager distanceManager = DistanceManager();
SteeringManager steeringManager = SteeringManager();

WiFiMulti wiFiMulti;
WebSocketsServer webSocket = WebSocketsServer(88);

unsigned long previousSteeringMillis = 0;
unsigned long previousMillis = 0;
bool ledState = false;

void read_dual_sensors() {
    // Serial.println(WiFi.localIP());
    uint16_t result[10];
    distanceManager.measureAll(result);
    for (uint8_t i = 0; i < 10; i++) {
        webSocket.broadcastTXT(String(distanceManager.directionToString(i)) + " " + result[i]);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Booting");

    pinMode(LED_GREEN, OUTPUT);

    // digitalWrite(LEFT_1_PIN, LOW);
    // analogWrite(LEFT_2_PIN, 70);
    // digitalWrite(RIGHT_1_PIN, LOW);
    // analogWrite(RIGHT_2_PIN, 70);

    connectionManager.initWiFi();
    connectionManager.initOta();
    distanceManager.initAddresses();
    steeringManager.init();

    webSocket.begin();
}

void loop() {
    ArduinoOTA.handle();
    webSocket.loop();
    unsigned long currentMillis = millis();

    if (currentMillis - previousSteeringMillis >= 100) {
        previousSteeringMillis = currentMillis;
        steeringManager.loop();
    }

    if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_GREEN, ledState);

        // int16_t count = 0;
        // pcnt_get_counter_value(PCNT_UNIT_0, &count);
        // Serial.print("Encoder count: ");
        // Serial.println(count);

        // read_dual_sensors();
    }
}
