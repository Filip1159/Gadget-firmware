#include "GyroTracker.h"

#define GYRO_TASK_STACK_BYTES 4096

GyroTracker::GyroTracker(Imu& imu, std::function<bool()> isStationary)
    : imu(imu), isStationary(std::move(isStationary)), mutex(xSemaphoreCreateMutex()) {}

void GyroTracker::startTask() {
    if (!imu.isOnline()) {
        return;
    }
    xTaskCreatePinnedToCore(task, "GyroTask", GYRO_TASK_STACK_BYTES, this, 2, NULL, 1);
}

void GyroTracker::task(void* parameter) {
    GyroTracker* tracker = static_cast<GyroTracker*>(parameter);
    TickType_t lastWake = xTaskGetTickCount();
    bool wasCalibrated = false;
#ifdef DEBUG_GYRO
    uint8_t printTick = 0;
#endif
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GYRO_PERIOD_MS));

        ImuSample sample;
        if (!tracker->imu.read(sample)) {
            continue;  // a failed read is a gap; GyroYaw does not integrate across long gaps
        }
        const bool stationary = tracker->isStationary();
        const uint32_t nowUs = micros();

        xSemaphoreTake(tracker->mutex, portMAX_DELAY);
        tracker->gyro.update(yawRateDps(sample), nowUs, stationary);
        tracker->lastSampleMillis = millis();
        const bool calibrated = tracker->gyro.calibrated();
        const float yaw = tracker->gyro.yawRad();
        const float bias = tracker->gyro.biasDps();
        xSemaphoreGive(tracker->mutex);

        if (calibrated && !wasCalibrated) {
            Serial.printf("Gyro calibrated: bias %.3f deg/s\n", bias);
        }
        wasCalibrated = calibrated;
#ifdef DEBUG_GYRO
        if (++printTick % 50 == 0) {
            Serial.printf("GYRO yaw=%.2f deg bias=%.3f deg/s rate=%.2f deg/s%s\n", yaw * 57.2957795f, bias,
                          yawRateDps(sample), calibrated ? "" : " (calibrating)");
        }
#else
        (void)yaw;
#endif
    }
}

bool GyroTracker::calibrated() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool value = gyro.calibrated();
    xSemaphoreGive(mutex);
    return value;
}

float GyroTracker::yawRad() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const float value = gyro.yawRad();
    xSemaphoreGive(mutex);
    return value;
}

bool GyroTracker::tryYaw(float& yawRad) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool usable = gyro.calibrated() && (millis() - lastSampleMillis) <= GYRO_MAX_AGE_MS;
    if (usable) {
        yawRad = gyro.yawRad();
    }
    xSemaphoreGive(mutex);
    return usable;
}

float GyroTracker::biasDps() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const float value = gyro.biasDps();
    xSemaphoreGive(mutex);
    return value;
}

void GyroTracker::resetYaw() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    gyro.resetYaw();
    xSemaphoreGive(mutex);
}
