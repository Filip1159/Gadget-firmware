#pragma once

#include <Adafruit_VL53L0X.h>

class DistanceSensor {
   private:
    Adafruit_VL53L0X* sensor;
    int address;
    uint8_t shtPin;

   public:
    DistanceSensor(int address, uint8_t shtPin);

    void setAddress();

    uint16_t measure();

    uint8_t getShtPin() const {
        return shtPin;
    }
};