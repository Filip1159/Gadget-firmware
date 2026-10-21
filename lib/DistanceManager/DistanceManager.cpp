#include <DistanceManager.h>
#include <DistanceSensor.h>

constexpr uint32_t MEASUREMENT_TASK_STACK_BYTES = 4096;

// Wire names, indexed by SensorDirection (must match the frontend's protocol.ts).
static const char* const kSensorNames[SENSOR_COUNT] = {
    "FRONT", "FRONT_LEFT_45", "FRONT_LEFT", "FRONT_RIGHT_45", "FRONT_RIGHT",
    "BACK",  "BACK_LEFT_45",  "BACK_LEFT",  "BACK_RIGHT_45",  "BACK_RIGHT",
};

DistanceManager::DistanceManager()
    : sensorsOrder{ FRONT, FRONT_LEFT_45, FRONT_LEFT, FRONT_RIGHT_45, FRONT_RIGHT,
                    BACK,  BACK_LEFT_45,  BACK_LEFT,  BACK_RIGHT_45,  BACK_RIGHT } {
    // clang-format off
    // Indexed by SensorDirection
    sensors[FRONT] = new DistanceSensor(FRONT_ADDRESS, FRONT_SHT);
    sensors[FRONT_LEFT_45] = new DistanceSensor(FRONT_LEFT_45_ADDRESS, FRONT_LEFT_45_SHT);
    sensors[FRONT_LEFT] = new DistanceSensor(FRONT_LEFT_ADDRESS, FRONT_LEFT_SHT);
    sensors[FRONT_RIGHT_45] = new DistanceSensor(FRONT_RIGHT_45_ADDRESS, FRONT_RIGHT_45_SHT);
    sensors[FRONT_RIGHT] = new DistanceSensor(FRONT_RIGHT_ADDRESS, FRONT_RIGHT_SHT);
    sensors[BACK] = new DistanceSensor(BACK_ADDRESS, BACK_SHT);
    sensors[BACK_LEFT_45] = new DistanceSensor(BACK_LEFT_45_ADDRESS, BACK_LEFT_45_SHT);
    sensors[BACK_LEFT] = new DistanceSensor(BACK_LEFT_ADDRESS, BACK_LEFT_SHT);
    sensors[BACK_RIGHT_45] = new DistanceSensor(BACK_RIGHT_45_ADDRESS, BACK_RIGHT_45_SHT);
    sensors[BACK_RIGHT] = new DistanceSensor(BACK_RIGHT_ADDRESS, BACK_RIGHT_SHT);
    // clang-format on

    // Initialize multi-core support
    measurementMutex = xSemaphoreCreateMutex();
    measurementRequested = false;
    measurementInProgress = false;
    measurementReady = false;
    measurementTaskHandle = NULL;

    // Initialize measurements array
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        latestMeasurements[i] = 0;
    }
}

void DistanceManager::initAddresses() {
    for (DistanceSensor* sensor : sensors) {
        pinMode(sensor->getShtPin(), OUTPUT);
    }

    for (DistanceSensor* sensor : sensors) {
        digitalWrite(sensor->getShtPin(), LOW);  // LOW = reset
    }

    delay(10);

    for (DistanceSensor* sensor : sensors) {
        sensor->setAddress();  // a failed sensor is marked offline, boot continues
    }
}

String DistanceManager::directionToString(SensorDirection direction) {
    return direction < SENSOR_COUNT ? String(kSensorNames[direction]) : String();
}

String DistanceManager::directionToString(uint8_t sensorIndex) {
    SensorDirection direction = sensorsOrder[sensorIndex];
    return directionToString(direction);
}

const char* DistanceManager::sensorName(uint8_t sensorIndex) const {
    if (sensorIndex >= SENSOR_COUNT || sensorsOrder[sensorIndex] >= SENSOR_COUNT) {
        return "";
    }
    return kSensorNames[sensorsOrder[sensorIndex]];
}

void DistanceManager::measurementTask(void* parameter) {
    DistanceManager* manager = static_cast<DistanceManager*>(parameter);
    bool stackLogged = false;

    while (true) {
        // Sleep until requestMeasurement() notifies us (no polling).
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Keep sweeping while requests keep arriving, so every request is answered by a sweep
        // that started after it. Requests made during a sweep are coalesced into one new sweep.
        while (true) {
            xSemaphoreTake(manager->measurementMutex, portMAX_DELAY);
            bool pending = manager->measurementRequested;
            manager->measurementRequested = false;
            manager->measurementInProgress = pending;
            xSemaphoreGive(manager->measurementMutex);
            if (!pending) {
                break;
            }

            // Measure into a private buffer so consumers never see a half-written sweep.
            uint16_t local[SENSOR_COUNT];
            const StampedPose startPose = manager->poseProvider ? manager->poseProvider() : StampedPose();
            for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
                local[i] = manager->sensors[manager->sensorsOrder[i]]->measure();
            }

            const StampedPose endPose = manager->poseProvider ? manager->poseProvider() : StampedPose();

            xSemaphoreTake(manager->measurementMutex, portMAX_DELAY);
            for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
                manager->latestMeasurements[i] = local[i];
            }
            manager->latestSweepStart = startPose;
            manager->latestSweepEnd = endPose;
            manager->measurementReady = true;
            manager->measurementInProgress = false;
            xSemaphoreGive(manager->measurementMutex);

            if (!stackLogged) {
                stackLogged = true;
                Serial.printf("MeasurementTask stack high-water mark: %u bytes free\n",
                              (unsigned)uxTaskGetStackHighWaterMark(NULL));
            }
        }
    }
}

void DistanceManager::startMeasurementTask() {
    // Create task on Core 0 (second core)
    xTaskCreatePinnedToCore(
        measurementTask,        // Task function
        "MeasurementTask",      // Task name
        MEASUREMENT_TASK_STACK_BYTES,  // Stack size
        this,                   // Parameter passed to task
        1,                      // Priority
        &measurementTaskHandle, // Task handle
        0                       // Core ID (0 = second core)
    );
}

void DistanceManager::requestMeasurement() {
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    measurementRequested = true;
    measurementReady = false;
    xSemaphoreGive(measurementMutex);
    xTaskNotifyGive(measurementTaskHandle);
}

bool DistanceManager::isMeasurementBusy() {
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    bool busy = measurementRequested || measurementInProgress;
    xSemaphoreGive(measurementMutex);
    return busy;
}

bool DistanceManager::isMeasurementReady() {
    bool ready;
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    ready = measurementReady;
    xSemaphoreGive(measurementMutex);
    return ready;
}

void DistanceManager::getLatestMeasurements(uint16_t result[SENSOR_COUNT]) {
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        result[i] = latestMeasurements[i];
    }
    xSemaphoreGive(measurementMutex);
}

void DistanceManager::getLatestSweep(uint16_t result[SENSOR_COUNT], StampedPose& start, StampedPose& end) {
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        result[i] = latestMeasurements[i];
    }
    start = latestSweepStart;
    end = latestSweepEnd;
    xSemaphoreGive(measurementMutex);
}

void DistanceManager::clearMeasurementReady() {
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    measurementReady = false;
    xSemaphoreGive(measurementMutex);
}
