#pragma once

#include <Arduino.h>
#include <ImuOrientation.h>

#define IMU_ADDRESS 0x68  // BMI160 with SAD tied to GND (3V3 would be 0x69)

// Bosch BMI160 6-axis IMU on the shared `Wire` bus (plain register access, no library). Accelerometer
// +-2 g, gyroscope +-500 deg/s, both at 100 Hz output rate. Raw sensor axes, no remapping: the board
// orientation is established by the IMU_CHECK command (see ImuOrientationCheck).
//
// begin() must run in setup() after the VL53L0X boot (which starts `Wire`). read() is thread-safe with
// respect to the sweep task (I2cBus lock).
class Imu {
   private:
    bool online = false;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);

   public:
    // Returns false (and stays offline) if the chip does not answer with the BMI160 id.
    bool begin();
    bool isOnline() const {
        return online;
    }
    // One accel + gyro sample in g and deg/s. False if the bus read failed.
    bool read(ImuSample& sample);
};
