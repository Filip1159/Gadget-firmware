#pragma once

#include <stdint.h>

// Measured by `CAL` runs against a flat wall, 100 samples per distance (MAPPING.MD item 0.4); regenerate
// with `python tools/analyze_sensors.py <csv...>` (it prints these arrays). The VL53L0X bias is NOT
// constant: it grows with distance (e.g. FRONT +18 mm at 100 mm, +63 mm at 1000 mm) and differs per
// sensor (the two BACK_RIGHT sensors read short at long range), hence a curve per sensor.
// Order: SensorDirection enum order.
constexpr uint8_t SENSOR_REF_COUNT = 4;
constexpr uint16_t SENSOR_REF_DISTANCES_MM[SENSOR_REF_COUNT] = {100, 300, 600, 1000};
// Mean raw reading of each sensor at the reference distances (must be strictly increasing).
constexpr uint16_t SENSOR_RAW_AT_REF_MM[10][SENSOR_REF_COUNT] = {
    {118, 342, 654, 1063},  // FRONT
    {139, 353, 658, 1078},  // FRONT_LEFT_45
    {111, 330, 639, 1049},  // FRONT_LEFT
    {118, 338, 646, 1062},  // FRONT_RIGHT_45
    {114, 328, 634, 1035},  // FRONT_RIGHT
    {131, 340, 646, 1059},  // BACK
    {121, 342, 650, 1053},  // BACK_LEFT_45
    {129, 356, 672, 1087},  // BACK_LEFT
    {122, 322, 611, 938},   // BACK_RIGHT_45
    {117, 321, 598, 914},   // BACK_RIGHT
};
// Largest tested distance with OOR <= 10% and std <= max(5 mm, 3% of distance). Not applied anywhere yet
// (the occupancy grid, MAPPING.MD 3.2, will use it); readings beyond it were never tested.
constexpr uint16_t SENSOR_TRUSTED_MAX_MM[10] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
