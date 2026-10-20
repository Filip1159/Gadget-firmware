#include <DistanceManager.h>
#include <DistanceSensor.h>

#include <map>

DistanceManager::DistanceManager() {
    sensorsOrder = new SensorDirection[10]{ FRONT, FRONT_LEFT_45, FRONT_LEFT, FRONT_RIGHT_45, FRONT_RIGHT,
                                            BACK,  BACK_LEFT_45,  BACK_LEFT,  BACK_RIGHT_45,  BACK_RIGHT };
    // clang-format off
    sensors = {
        { FRONT, new DistanceSensor(FRONT_ADDRESS, FRONT_SHT) },
        { FRONT_LEFT_45, new DistanceSensor(FRONT_LEFT_45_ADDRESS, FRONT_LEFT_45_SHT) },
        { FRONT_LEFT, new DistanceSensor(FRONT_LEFT_ADDRESS, FRONT_LEFT_SHT) },
        { FRONT_RIGHT_45, new DistanceSensor(FRONT_RIGHT_45_ADDRESS, FRONT_RIGHT_45_SHT) },
        { FRONT_RIGHT, new DistanceSensor(FRONT_RIGHT_ADDRESS, FRONT_RIGHT_SHT) },
        { BACK, new DistanceSensor(BACK_ADDRESS, BACK_SHT) },
        { BACK_LEFT_45, new DistanceSensor(BACK_LEFT_45_ADDRESS, BACK_LEFT_45_SHT) },
        { BACK_LEFT, new DistanceSensor(BACK_LEFT_ADDRESS, BACK_LEFT_SHT) },
        { BACK_RIGHT_45, new DistanceSensor(BACK_RIGHT_45_ADDRESS, BACK_RIGHT_45_SHT) },
        { BACK_RIGHT, new DistanceSensor(BACK_RIGHT_ADDRESS, BACK_RIGHT_SHT) }
    };
    // clang-format on

    // Initialize multi-core support
    measurementMutex = xSemaphoreCreateMutex();
    measurementRequested = false;
    measurementReady = false;
    measurementTaskHandle = NULL;

    // Initialize measurements array
    for (uint8_t i = 0; i < 10; i++) {
        latestMeasurements[i] = 0;
    }
}

void initPinsAsOutputs() {
    pinMode(FRONT_SHT, OUTPUT);
    pinMode(FRONT_LEFT_45_SHT, OUTPUT);
    pinMode(FRONT_LEFT_SHT, OUTPUT);
    pinMode(FRONT_RIGHT_45_SHT, OUTPUT);
    pinMode(FRONT_RIGHT_SHT, OUTPUT);
    pinMode(BACK_SHT, OUTPUT);
    pinMode(BACK_LEFT_45_SHT, OUTPUT);
    pinMode(BACK_LEFT_SHT, OUTPUT);
    pinMode(BACK_RIGHT_45_SHT, OUTPUT);
    pinMode(BACK_RIGHT_SHT, OUTPUT);
}

void DistanceManager::initAddresses() {
    for (auto sensor : sensors) {
        pinMode(sensor.second->getShtPin(), OUTPUT);
    }

    for (auto sensor : sensors) {
        digitalWrite(sensor.second->getShtPin(), LOW);  // LOW = reset
    }

    delay(10);

    for (auto sensor : sensors) {
        sensor.second->setAddress();
    }
}

uint16_t DistanceManager::measure(SensorDirection direction) {
    return sensors.at(direction)->measure();
}

void DistanceManager::measureAll(uint16_t result[10]) {
    for (uint8_t i = 0; i < 10; i++) {
        result[i] = sensors.at(sensorsOrder[i])->measure();
    }
}

String DistanceManager::directionToString(SensorDirection direction) {
    switch (direction) {
        case FRONT:
            return "FRONT";
        case FRONT_LEFT_45:
            return "FRONT_LEFT_45";
        case FRONT_LEFT:
            return "FRONT_LEFT";
        case FRONT_RIGHT_45:
            return "FRONT_RIGHT_45";
        case FRONT_RIGHT:
            return "FRONT_RIGHT";
        case BACK:
            return "BACK";
        case BACK_LEFT_45:
            return "BACK_LEFT_45";
        case BACK_LEFT:
            return "BACK_LEFT";
        case BACK_RIGHT_45:
            return "BACK_RIGHT_45";
        case BACK_RIGHT:
            return "BACK_RIGHT";
    }
}

String DistanceManager::directionToString(uint8_t sensorIndex) {
    SensorDirection direction = sensorsOrder[sensorIndex];
    return directionToString(direction);
}

void DistanceManager::measurementTask(void* parameter) {
    DistanceManager* manager = static_cast<DistanceManager*>(parameter);

    while (true) {
        // Wait for measurement request
        if (manager->measurementRequested) {
            // Perform measurements
            for (uint8_t i = 0; i < 10; i++) {
                manager->latestMeasurements[i] = manager->sensors.at(manager->sensorsOrder[i])->measure();
            }

            // Mark as ready and clear request
            xSemaphoreTake(manager->measurementMutex, portMAX_DELAY);
            manager->measurementReady = true;
            manager->measurementRequested = false;
            xSemaphoreGive(manager->measurementMutex);
        }

        // Small delay to prevent busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void DistanceManager::startMeasurementTask() {
    // Create task on Core 0 (second core)
    xTaskCreatePinnedToCore(
        measurementTask,        // Task function
        "MeasurementTask",      // Task name
        4096,                   // Stack size
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
}

bool DistanceManager::isMeasurementReady() {
    bool ready;
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    ready = measurementReady;
    xSemaphoreGive(measurementMutex);
    return ready;
}

void DistanceManager::getLatestMeasurements(uint16_t result[10]) {
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    for (uint8_t i = 0; i < 10; i++) {
        result[i] = latestMeasurements[i];
    }
    xSemaphoreGive(measurementMutex);
}

void DistanceManager::clearMeasurementReady() {
    xSemaphoreTake(measurementMutex, portMAX_DELAY);
    measurementReady = false;
    xSemaphoreGive(measurementMutex);
}
