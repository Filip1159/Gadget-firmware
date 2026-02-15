#pragma once

#include <ArduinoOTA.h>

class ConnectionManager {
   private:
    const char* ssid = "Filip S25";
    const char* password = "abcdefgh";

   public:
    void initWiFi();
    void initOta();
};
