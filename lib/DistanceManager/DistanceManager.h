#pragma once

#include <Arduino.h>
#include <DistanceSensor.h>
#include <Odometry.h>
#include <functional>

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

#define SENSOR_COUNT 10

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
    DistanceSensor* sensors[SENSOR_COUNT];  // indexed by SensorDirection
    SensorDirection sensorsOrder[SENSOR_COUNT];

    // Multi-core support
    TaskHandle_t measurementTaskHandle;
    SemaphoreHandle_t measurementMutex;
    // All three guarded by measurementMutex
    bool measurementRequested;
    bool measurementInProgress;
    bool measurementReady;
    uint16_t latestMeasurements[SENSOR_COUNT];
    StampedPose latestSweepStart;  // robot pose when the latest sweep started / ended
    StampedPose latestSweepEnd;
    std::function<StampedPose()> poseProvider;  // set before startMeasurementTask()

    static void measurementTask(void* parameter);

   public:
    DistanceManager();

    void initAddresses();

    // Called from the Core-0 task at the start and end of every sweep. Must be thread-safe and quick.
    // Call before startMeasurementTask().
    void setPoseProvider(std::function<StampedPose()> provider) {
        poseProvider = provider;
    }

    void startMeasurementTask();

    void requestMeasurement();

    bool isMeasurementReady();

    // True while a requested sweep is queued or running.
    bool isMeasurementBusy();

    void getLatestMeasurements(uint16_t result[SENSOR_COUNT]);

    // The latest sweep and the robot pose at its start and end, read atomically (one lock).
    void getLatestSweep(uint16_t result[SENSOR_COUNT], StampedPose& start, StampedPose& end);

    void clearMeasurementReady();

    String directionToString(SensorDirection direction);

    String directionToString(uint8_t sensorIndex);
};
