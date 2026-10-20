#pragma once

#include <ArduinoOTA.h>
#include <WebSocketsServer.h>

#include <functional>

#include "secrets.h"  // gitignored; copy secrets.h.example

#define MAX_MESSAGE_LENGTH 64
#define HOSTNAME "gadget"

class ConnectionManager {
   private:
    const char* ssid = WIFI_SSID;
    const char* password = WIFI_PASSWORD;
    std::function<void(const String&)> messageCallback;
    std::function<void()> disconnectCallback;
    std::function<void()> otaStartCallback;
    std::function<void(uint8_t)> clientReadyCallback;
    WebSocketsServer* webSocket = nullptr;
    // Without WS_TOKEN every connected client is trusted; with it, a client must send "AUTH <token>" first.
    bool authenticated[WEBSOCKETS_SERVER_CLIENT_MAX] = {};

    void markReady(uint8_t num);

   public:
    void initWiFi();
    void initOta();
    void initWebsocket(std::function<void(const String&)> onMessage);
    void setOnDisconnect(std::function<void()> callback) {
        disconnectCallback = callback;
    }
    void setOnOtaStart(std::function<void()> callback) {
        otaStartCallback = callback;
    }
    // Called when a client is connected and (if WS_TOKEN is set) authenticated.
    void setOnClientReady(std::function<void(uint8_t)> callback) {
        clientReadyCallback = callback;
    }
    // Sends to all ready clients.
    void send(const String& message);
    void sendTo(uint8_t num, const String& message);
    void loop();
};
