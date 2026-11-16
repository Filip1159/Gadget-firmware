#include "I2cBus.h"

static SemaphoreHandle_t busMutex() {
    static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();  // thread-safe local static
    return mutex;
}

I2cBus::Lock::Lock() {
    xSemaphoreTake(busMutex(), portMAX_DELAY);
}

I2cBus::Lock::~Lock() {
    xSemaphoreGive(busMutex());
}
