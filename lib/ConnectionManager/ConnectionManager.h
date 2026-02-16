#pragma once

#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include <WiFiClientSecure.h>
#include <WiFiMulti.h>

class ConnectionManager {
   private:
    const char* ssid = "Filip S25";
    const char* password = "abcdefgh";
    void (*websocketCallback)(String);
    WiFiMulti* wiFiMulti;
    WebSocketsServer* webSocket;

   public:
    void initWiFi();
    void initOta();
    void initWebsocket(void (*callback)(String));
    void send(String message);
    void loop();
};
