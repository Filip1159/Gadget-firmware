#pragma once

#include <Adafruit_VL53L0X.h>

// Sentinel for "no obstacle in range" and for offline sensors; sent on the wire as OOR.
#define OUT_OF_RANGE_MM 0xFFFF

class DistanceSensor {
   private:
    Adafruit_VL53L0X* sensor;
    int address;
    uint8_t shtPin;
    bool online = false;

   public:
    DistanceSensor(int address, uint8_t shtPin);

    // Returns false (and leaves the sensor offline) if it did not respond.
    bool setAddress();

    uint16_t measure();

    bool isOnline() const {
        return online;
    }

    uint8_t getShtPin() const {
        return shtPin;
    }
};