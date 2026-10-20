#pragma once

#include <Arduino.h>
#include <DistanceSensor.h>

#include <map>

#define FRONT_ADDRESS 0x30
#define FRONT_LEFT_45_ADDRESS 0x31
#define FRONT_LEFT_ADDRESS 0x32
#define FRONT_RIGHT_45_ADDRESS 0x33
#define FRONT_RIGHT_ADDRESS 0x34
#define BACK_ADDRESS 0x35
#define BACK_LEFT_45_ADDRESS 0x36
#define BACK_LEFT_ADDRESS 0x37
#define BACK_RIGHT_45_ADDRESS 0x38
#define BACK_RIGHT_ADDRESS 0x39

#define FRONT_SHT A6
#define FRONT_LEFT_45_SHT 11
#define FRONT_LEFT_SHT 12
#define FRONT_RIGHT_45_SHT A7
#define FRONT_RIGHT_SHT 13
#define BACK_SHT 6
#define BACK_LEFT_45_SHT 7
#define BACK_LEFT_SHT 10
#define BACK_RIGHT_45_SHT A3
#define BACK_RIGHT_SHT A2

enum SensorDirection {
    FRONT,
    FRONT_LEFT_45,
    FRONT_LEFT,
    FRONT_RIGHT_45,
    FRONT_RIGHT,
    BACK,
    BACK_LEFT_45,
    BACK_LEFT,
    BACK_RIGHT_45,
    BACK_RIGHT
};

class DistanceManager {
   private:
    std::map<SensorDirection, DistanceSensor*> sensors;
    SensorDirection* sensorsOrder;

    // Multi-core support
    TaskHandle_t measurementTaskHandle;
    SemaphoreHandle_t measurementMutex;
    volatile bool measurementRequested;
    volatile bool measurementReady;
    uint16_t latestMeasurements[10];

    static void measurementTask(void* parameter);

   public:
    DistanceManager();

    void initAddresses();

    void startMeasurementTask();

    uint16_t measure(SensorDirection direction);

    void measureAll(uint16_t result[10]);

    void requestMeasurement();

    bool isMeasurementReady();

    void getLatestMeasurements(uint16_t result[10]);

    void clearMeasurementReady();

    SensorDirection* getSensorsOrder() {
        return sensorsOrder;
    }

    String directionToString(SensorDirection direction);

    String directionToString(uint8_t sensorIndex);
};
