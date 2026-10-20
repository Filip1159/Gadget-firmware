#include <ConnectionManager.h>

void ConnectionManager::initOta() {
    ArduinoOTA
        .onStart([this]() {
            if (otaStartCallback) {
                otaStartCallback();
            }
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
        .onError([](ota_error_t error) { Serial.printf("Error[%u]: ", error); });

    ArduinoOTA.begin();
}

void ConnectionManager::initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void ConnectionManager::initWebsocket(void (*callback)(String message)) {
    websocketCallback = callback;
    wiFiMulti = new WiFiMulti();
    webSocket = new WebSocketsServer(88);
    webSocket->begin();
    webSocket->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        switch (type) {
            case WStype_DISCONNECTED:
                Serial.println("WebSocket disconnected");
                if (disconnectCallback) {
                    disconnectCallback();
                }
                break;
            case WStype_CONNECTED:
                Serial.println("WebSocket connected");
                break;
            case WStype_TEXT:
                Serial.print("Received message from server: ");
                Serial.println((char*)payload);
                websocketCallback(String((char*)payload));
                break;
            case WStype_BIN:
                Serial.println("Received binary message");
                break;
        }
    });
}

void ConnectionManager::send(String message) {
    webSocket->broadcastTXT(message);
}

void ConnectionManager::loop() {
    ArduinoOTA.handle();
    webSocket->loop();
}
