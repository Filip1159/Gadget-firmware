#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <driver/pcnt.h>
#include "Adafruit_VL53L0X.h"
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>

#include <WebSocketsServer.h>

#define LOX_FRONT_ADDRESS 0x30
#define LOX_FRONT_LEFT_45_ADDRESS 0x31
#define LOX_FRONT_LEFT_ADDRESS 0x32
#define LOX_FRONT_RIGHT_45_ADDRESS 0x33
#define LOX_FRONT_RIGHT_ADDRESS 0x34
#define LOX_BACK_ADDRESS 0x35
#define LOX_BACK_LEFT_45_ADDRESS 0x36
#define LOX_BACK_LEFT_ADDRESS 0x37
#define LOX_BACK_RIGHT_45_ADDRESS 0x38
#define LOX_BACK_RIGHT_ADDRESS 0x39

#define SHT_FRONT A6
#define SHT_FRONT_LEFT_45 11
#define SHT_FRONT_LEFT 12
#define SHT_FRONT_RIGHT_45 A7
#define SHT_FRONT_RIGHT 13
#define SHT_BACK 6
#define SHT_BACK_LEFT_45 7
#define SHT_BACK_LEFT 10
#define SHT_BACK_RIGHT_45 A3
#define SHT_BACK_RIGHT A2

WiFiMulti wiFiMulti;
WebSocketsServer webSocket = WebSocketsServer(81);

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

const char *ssid = "Filip S25";
const char *password = "abcdefgh";
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

void setID() {
  // all reset
  digitalWrite(SHT_FRONT, LOW);
  digitalWrite(SHT_FRONT_LEFT_45, LOW);
  digitalWrite(SHT_FRONT_LEFT, LOW);
  digitalWrite(SHT_FRONT_RIGHT_45, LOW);
  digitalWrite(SHT_FRONT_RIGHT, LOW);
  digitalWrite(SHT_BACK, LOW);
  digitalWrite(SHT_BACK_LEFT_45, LOW);
  digitalWrite(SHT_BACK_LEFT, LOW);
  digitalWrite(SHT_BACK_RIGHT_45, LOW);
  digitalWrite(SHT_BACK_RIGHT, LOW);
  
  delay(10);
  // all unreset
  // digitalWrite(SHT_FRONT_LEFT_45, HIGH);
  // digitalWrite(SHT_FRONT_LEFT, HIGH);
  // digitalWrite(SHT_FRONT_RIGHT_45, HIGH);
  // digitalWrite(SHT_FRONT_RIGHT, HIGH);
  // digitalWrite(SHT_BACK, HIGH);
  // digitalWrite(SHT_BACK_LEFT_45, HIGH);
  // digitalWrite(SHT_BACK_LEFT, HIGH);
  // digitalWrite(SHT_BACK_RIGHT_45, HIGH);
  // digitalWrite(SHT_BACK_RIGHT, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_FRONT, HIGH);
  digitalWrite(SHT_BACK, LOW);

  // initing LOX1
  if(!lox1.begin(LOX_FRONT_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_BACK, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX_BACK_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
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
  pcnt_config.ctrl_gpio_num  = LEFT_ENCODER_B;
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
    .onEnd([]() {
      Serial.println("\nEnd");
    })
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
  analogWrite(LEFT_2_PIN, 100);
  digitalWrite(RIGHT_1_PIN, LOW);
  analogWrite(RIGHT_2_PIN, 200);

  pinMode(SHT_FRONT, OUTPUT);
  pinMode(SHT_FRONT_LEFT_45, OUTPUT);
  pinMode(SHT_FRONT_LEFT, OUTPUT);
  pinMode(SHT_FRONT_RIGHT_45, OUTPUT);
  pinMode(SHT_FRONT_RIGHT, OUTPUT);
  pinMode(SHT_BACK, OUTPUT);
  pinMode(SHT_BACK_LEFT_45, OUTPUT);
  pinMode(SHT_BACK_LEFT, OUTPUT);
  pinMode(SHT_BACK_RIGHT_45, OUTPUT);
  pinMode(SHT_BACK_RIGHT, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_FRONT, LOW);
  digitalWrite(SHT_BACK, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  Serial.println(F("Starting..."));
  setID();

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

    webSocket.broadcastTXT("Hello from ESP32!");
  }

  webSocket.loop();
}
