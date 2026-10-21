#pragma once

#include <stdint.h>

// Hardware-free raw -> true distance correction for the VL53L0X sensors (host-testable, see test/).
// Piecewise-linear through the per-sensor calibration points in SensorCalibrationData.h.

// Marker the firmware uses for "no obstacle / out of range" (same value as OUT_OF_RANGE_MM).
constexpr uint16_t SENSOR_CAL_OUT_OF_RANGE = 0xFFFF;

// `sensorIndex` is the SensorDirection enum value. Out-of-range readings and unknown sensors pass through.
// Below the first / above the last calibration point the nearest segment is extended (clamped at 0).
uint16_t correctReading(uint8_t sensorIndex, uint16_t rawMm);
