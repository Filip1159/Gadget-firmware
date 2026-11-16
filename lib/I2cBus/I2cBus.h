#pragma once

#include <Arduino.h>

// The VL53L0X ranging (Core-0 sweep task) and the IMU reads (Core 1) share one `Wire` bus, and an I2C
// transaction must not be interleaved with another. Hold an `I2cBus::Lock` around every group of Wire
// calls after boot (setup() is single-threaded and does not need it).
namespace I2cBus {

class Lock {
   public:
    Lock();
    ~Lock();
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;
};

}  // namespace I2cBus
