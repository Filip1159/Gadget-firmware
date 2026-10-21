#include <SensorCalibration.h>
#include <SensorCalibrationData.h>

uint16_t correctReading(uint8_t sensorIndex, uint16_t rawMm) {
    if (rawMm == SENSOR_CAL_OUT_OF_RANGE || sensorIndex >= 10) {
        return rawMm;
    }
    const uint16_t* raw = SENSOR_RAW_AT_REF_MM[sensorIndex];

    // First segment whose upper end is at or above the reading; the last segment for anything higher.
    uint8_t segment = 0;
    while (segment < SENSOR_REF_COUNT - 2 && rawMm > raw[segment + 1]) {
        segment++;
    }

    const float rawSpan = static_cast<float>(raw[segment + 1]) - raw[segment];
    const float trueSpan = static_cast<float>(SENSOR_REF_DISTANCES_MM[segment + 1]) - SENSOR_REF_DISTANCES_MM[segment];
    float corrected = SENSOR_REF_DISTANCES_MM[segment];
    if (rawSpan > 0.0f) {
        corrected += (static_cast<float>(rawMm) - raw[segment]) * trueSpan / rawSpan;
    }
    if (corrected < 0.0f) {
        return 0;
    }
    if (corrected > static_cast<float>(SENSOR_CAL_OUT_OF_RANGE - 1)) {
        return SENSOR_CAL_OUT_OF_RANGE - 1;  // never collide with the out-of-range marker
    }
    return static_cast<uint16_t>(corrected + 0.5f);
}
