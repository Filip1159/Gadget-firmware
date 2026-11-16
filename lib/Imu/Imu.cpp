#include "Imu.h"

#include <I2cBus.h>
#include <Wire.h>

namespace {

constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_PMU_STATUS = 0x03;
constexpr uint8_t REG_DATA_GYR = 0x0C;  // GYR X/Y/Z then ACC X/Y/Z, 16 bit little endian, 12 bytes
constexpr uint8_t REG_ACC_CONF = 0x40;
constexpr uint8_t REG_ACC_RANGE = 0x41;
constexpr uint8_t REG_GYR_CONF = 0x42;
constexpr uint8_t REG_GYR_RANGE = 0x43;
constexpr uint8_t REG_CMD = 0x7E;

constexpr uint8_t CHIP_ID = 0xD1;
constexpr uint8_t CMD_ACC_NORMAL = 0x11;
constexpr uint8_t CMD_GYR_NORMAL = 0x15;
constexpr uint8_t CMD_SOFT_RESET = 0xB6;
constexpr uint8_t PMU_ALL_NORMAL = 0x14;  // acc_pmu_status = 01, gyr_pmu_status = 01

constexpr uint8_t ACC_RANGE_2G = 0x03;
constexpr uint8_t GYR_RANGE_500DPS = 0x02;
constexpr uint8_t CONF_100HZ_NORMAL = 0x28;  // odr 100 Hz, normal filter mode

constexpr float ACC_LSB_PER_G = 16384.0f;   // +-2 g
constexpr float GYR_LSB_PER_DPS = 65.6f;    // +-500 deg/s

int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

}  // namespace

bool Imu::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool Imu::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    const size_t received = Wire.requestFrom(static_cast<uint8_t>(IMU_ADDRESS), static_cast<uint8_t>(length));
    if (received != length) {
        return false;
    }
    for (size_t i = 0; i < length; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

bool Imu::begin() {
    I2cBus::Lock lock;
    online = false;

    uint8_t id = 0;
    if (!readRegisters(REG_CHIP_ID, &id, 1) || id != CHIP_ID) {
        Serial.printf("BMI160 not found at %#x (chip id %#x, expected %#x)\n", IMU_ADDRESS, id, CHIP_ID);
        return false;
    }

    // Soft reset, then switch the accelerometer and gyroscope to normal mode (start-up takes up to
    // ~4 ms for the accelerometer and ~80 ms for the gyroscope).
    writeRegister(REG_CMD, CMD_SOFT_RESET);
    delay(100);
    writeRegister(REG_CMD, CMD_ACC_NORMAL);
    delay(10);
    writeRegister(REG_CMD, CMD_GYR_NORMAL);
    delay(100);
    writeRegister(REG_ACC_RANGE, ACC_RANGE_2G);
    writeRegister(REG_GYR_RANGE, GYR_RANGE_500DPS);
    writeRegister(REG_ACC_CONF, CONF_100HZ_NORMAL);
    writeRegister(REG_GYR_CONF, CONF_100HZ_NORMAL);

    uint8_t status = 0;
    if (!readRegisters(REG_PMU_STATUS, &status, 1) || status != PMU_ALL_NORMAL) {
        Serial.printf("BMI160 did not enter normal mode (PMU status %#x)\n", status);
        return false;
    }
    online = true;
    Serial.printf("BMI160 online at %#x\n", IMU_ADDRESS);
    return true;
}

bool Imu::read(ImuSample& sample) {
    if (!online) {
        return false;
    }
    uint8_t raw[12];
    {
        I2cBus::Lock lock;
        if (!readRegisters(REG_DATA_GYR, raw, sizeof(raw))) {
            return false;
        }
    }
    sample.gx = le16(raw + 0) / GYR_LSB_PER_DPS;
    sample.gy = le16(raw + 2) / GYR_LSB_PER_DPS;
    sample.gz = le16(raw + 4) / GYR_LSB_PER_DPS;
    sample.ax = le16(raw + 6) / ACC_LSB_PER_G;
    sample.ay = le16(raw + 8) / ACC_LSB_PER_G;
    sample.az = le16(raw + 10) / ACC_LSB_PER_G;
    return true;
}
