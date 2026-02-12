#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <driver/pcnt.h>

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

// pcnt_config_t pcnt_config;

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

  // pcnt_config.pulse_gpio_num = LEFT_ENCODER_A;
  // pcnt_config.ctrl_gpio_num = LEFT_ENCODER_B;
  // pcnt_config.channel = PCNT_CHANNEL_0;
  // pcnt_config.unit = PCNT_UNIT_0;

  // pcnt_config.pos_mode = PCNT_COUNT_INC;   // zbocze narastające
  // pcnt_config.neg_mode = PCNT_COUNT_DIS;   // ignoruj opadające

  // pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // B=0 → reverse
  // pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // B=1 → normal

  // gpio_set_pull_mode(GPIO_NUM_17, GPIO_PULLUP_ONLY);
	// gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLUP_ONLY);

  // pcnt_unit_config(&pcnt_config);
  // pcnt_set_filter_value(PCNT_UNIT_0, 100);
  // pcnt_filter_enable(PCNT_UNIT_0);

  // pcnt_counter_pause(PCNT_UNIT_0);
	// pcnt_counter_clear(PCNT_UNIT_0);
	// pcnt_counter_resume(PCNT_UNIT_0);

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
}

void loop() {
  ArduinoOTA.handle();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    ledState = !ledState;

    // set the LED with the ledState of the variable:
    digitalWrite(LED_GREEN, ledState);

    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    Serial.print("Encoder count: ");
    Serial.println(count);
  }
}
