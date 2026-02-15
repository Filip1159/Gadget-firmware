#include <ArduinoOTA.h>
#include <DistanceManager.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiMulti.h>
#include <driver/pcnt.h>

DistanceManager distanceManager = DistanceManager();

WiFiMulti wiFiMulti;
WebSocketsServer webSocket = WebSocketsServer(88);

const char* ssid = "Filip S25";
const char* password = "abcdefgh";
uint32_t last_ota_time = 0;

unsigned long previousMillis = 0;
bool ledState = false;

#define LEFT_1_PIN 5
#define LEFT_2_PIN 4
#define RIGHT_1_PIN 3
#define RIGHT_2_PIN 2

#define LEFT_ENCODER_A 17
#define LEFT_ENCODER_B 18
#define RIGHT_ENCODER_A A0
#define RIGHT_ENCODER_B A1

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
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    pinMode(LED_GREEN, OUTPUT);
    pinMode(LEFT_1_PIN, OUTPUT);
    pinMode(LEFT_2_PIN, OUTPUT);
    pinMode(RIGHT_1_PIN, OUTPUT);
    pinMode(RIGHT_2_PIN, OUTPUT);

    // 1. Konfiguracja jednostki licznika
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = LEFT_ENCODER_A;
    pcnt_config.ctrl_gpio_num = LEFT_ENCODER_B;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT_0;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DEC;
    pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config.counter_h_lim = 32767;
    pcnt_config.counter_l_lim = -32768;

    pcnt_unit_config(&pcnt_config);

    // hardware debounce
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else {  // U_SPIFFS
                type = "filesystem";
            }

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total) {
            if (millis() - last_ota_time > 500) {
                Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
                last_ota_time = millis();
            }
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                Serial.println("Auth Failed");
            } else if (error == OTA_BEGIN_ERROR) {
                Serial.println("Begin Failed");
            } else if (error == OTA_CONNECT_ERROR) {
                Serial.println("Connect Failed");
            } else if (error == OTA_RECEIVE_ERROR) {
                Serial.println("Receive Failed");
            } else if (error == OTA_END_ERROR) {
                Serial.println("End Failed");
            }
        });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    digitalWrite(LEFT_1_PIN, LOW);
    analogWrite(LEFT_2_PIN, 70);
    digitalWrite(RIGHT_1_PIN, LOW);
    analogWrite(RIGHT_2_PIN, 70);

    distanceManager.initAddresses();

    webSocket.begin();
}

void loop() {
    ArduinoOTA.handle();
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_GREEN, ledState);

        int16_t count = 0;
        pcnt_get_counter_value(PCNT_UNIT_0, &count);
        Serial.print("Encoder count: ");
        Serial.println(count);

        read_dual_sensors();

        // webSocket.broadcastTXT("Hello from ESP32!");
    }

    webSocket.loop();
}
