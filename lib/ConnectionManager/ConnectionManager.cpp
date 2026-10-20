#include <ConnectionManager.h>

#include <esp32-hal.h>

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
        .onProgress([](unsigned int progress, unsigned int total) {
            feedLoopWDT();  // handle() blocks for the whole upload; don't let the loop watchdog fire
            static uint8_t lastPercent = 255;
            uint8_t percent = total ? (progress * 100) / total : 0;
            if (percent / 10 != lastPercent / 10) {
                lastPercent = percent;
                Serial.printf("OTA progress: %u%%\n", percent);
            }
        })
        .onEnd([]() { Serial.println("OTA finished"); })
        .onError([](ota_error_t error) {
            const char* reason = "unknown";
            switch (error) {
                case OTA_AUTH_ERROR:
                    reason = "auth failed";
                    break;
                case OTA_BEGIN_ERROR:
                    reason = "begin failed";
                    break;
                case OTA_CONNECT_ERROR:
                    reason = "connect failed";
                    break;
                case OTA_RECEIVE_ERROR:
                    reason = "receive failed";
                    break;
                case OTA_END_ERROR:
                    reason = "end failed";
                    break;
            }
            Serial.printf("OTA error[%u]: %s\n", error, reason);
        });

    ArduinoOTA.setHostname(HOSTNAME);  // reachable as <HOSTNAME>.local via mDNS
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.begin();
}

void ConnectionManager::initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    WiFi.setAutoReconnect(true);  // rejoin by itself after a drop
    WiFi.persistent(false);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);  // modem sleep adds latency to WebSocket commands
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void ConnectionManager::markReady(uint8_t num) {
    authenticated[num] = true;
    if (clientReadyCallback) {
        clientReadyCallback(num);
    }
}

void ConnectionManager::initWebsocket(std::function<void(const String&)> onMessage) {
    messageCallback = onMessage;
    webSocket = new WebSocketsServer(88);
    webSocket->begin();
    webSocket->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        switch (type) {
            case WStype_DISCONNECTED:
                Serial.println("WebSocket disconnected");
                authenticated[num] = false;
                if (disconnectCallback) {
                    disconnectCallback();
                }
                break;
            case WStype_CONNECTED:
                Serial.println("WebSocket connected");
#ifdef WS_TOKEN
                authenticated[num] = false;  // wait for AUTH
#else
                markReady(num);
#endif
                break;
            case WStype_TEXT: {
                if (length > MAX_MESSAGE_LENGTH) {
                    Serial.printf("Ignoring oversized message (%u bytes)\n", (unsigned)length);
                    break;
                }
                String message((char*)payload);
#ifdef WS_TOKEN
                if (!authenticated[num]) {
                    if (message == "AUTH " WS_TOKEN) {
                        Serial.println("WebSocket client authenticated");
                        markReady(num);
                    } else {
                        Serial.println("WebSocket auth failed, disconnecting client");
                        webSocket->disconnect(num);
                    }
                    break;
                }
#endif
                if (message.startsWith("AUTH ")) {
                    break;  // already ready (or auth not required): ignore
                }
                Serial.println("Received message: " + message);
                messageCallback(message);
                break;
            }
            case WStype_BIN:
                Serial.println("Received binary message");
                break;
            default:
                break;
        }
    });
}

void ConnectionManager::sendTo(uint8_t num, const String& message) {
    String copy = message;  // sendTXT takes a non-const String&
    webSocket->sendTXT(num, copy);
}

void ConnectionManager::send(const String& message) {
    for (uint8_t i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
        if (authenticated[i] && webSocket->clientIsConnected(i)) {
            sendTo(i, message);
        }
    }
}

void ConnectionManager::loop() {
    ArduinoOTA.handle();
    webSocket->loop();
}
