#pragma once

#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include <WiFiClientSecure.h>
#include <WiFiMulti.h>

#include "secrets.h"  // gitignored; copy secrets.h.example

class ConnectionManager {
   private:
    const char* ssid = WIFI_SSID;
    const char* password = WIFI_PASSWORD;
    void (*websocketCallback)(String);
    void (*disconnectCallback)() = nullptr;
    void (*otaStartCallback)() = nullptr;
    WiFiMulti* wiFiMulti;
    WebSocketsServer* webSocket;

   public:
    void initWiFi();
    void initOta();
    void initWebsocket(void (*callback)(String));
    void setOnDisconnect(void (*callback)()) {
        disconnectCallback = callback;
    }
    void setOnOtaStart(void (*callback)()) {
        otaStartCallback = callback;
    }
    void send(String message);
    void loop();
};
