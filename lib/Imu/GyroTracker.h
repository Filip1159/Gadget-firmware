#pragma once

#include <Arduino.h>
#include <GyroYaw.h>
#include <Imu.h>

#include <functional>

// Gyro sampling period; matches the BMI160's 100 Hz output rate.
#define GYRO_PERIOD_MS 10

// Orientation of the board on the robot, established with IMU_CHECK (2026-11-16): flat, Z up, so the
// robot's counter-clockwise yaw rate is +gz. If the board is ever remounted, re-run IMU_CHECK.
inline float yawRateDps(const ImuSample& sample) {
    return sample.gz;
}

// Reads the gyro every GYRO_PERIOD_MS in its own task (Core 1, priority 2 like the control task) and
// integrates the yaw with GyroYaw: boot bias calibration (robot standing still ~2.5 s after the boot),
// then bias tracking and yaw integration. All getters are thread-safe.
class GyroTracker {
   private:
    Imu& imu;
    std::function<bool()> isStationary;
    GyroYaw gyro;
    SemaphoreHandle_t mutex;

    static void task(void* parameter);

   public:
    // `isStationary` says whether the robot is not being driven (called from the task).
    GyroTracker(Imu& imu, std::function<bool()> isStationary);
    GyroTracker(const GyroTracker&) = delete;  // owns a mutex
    GyroTracker& operator=(const GyroTracker&) = delete;

    // Starts the sampling task. Does nothing if the IMU is offline.
    void startTask();

    bool calibrated();
    // Yaw since the first calibration or the last resetYaw(): radians in (-pi, pi], counter-clockwise.
    float yawRad();
    float biasDps();
    void resetYaw();
};
