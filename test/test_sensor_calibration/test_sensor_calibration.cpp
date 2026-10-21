#include <SensorCalibration.h>
#include <SensorCalibrationData.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_reference_points_map_exactly_to_their_distances() {
    for (uint8_t sensor = 0; sensor < 10; sensor++) {
        for (uint8_t i = 0; i < SENSOR_REF_COUNT; i++) {
            TEST_ASSERT_EQUAL_UINT16(SENSOR_REF_DISTANCES_MM[i], correctReading(sensor, SENSOR_RAW_AT_REF_MM[sensor][i]));
        }
    }
}

void test_calibration_curves_are_strictly_increasing() {
    for (uint8_t sensor = 0; sensor < 10; sensor++) {
        for (uint8_t i = 1; i < SENSOR_REF_COUNT; i++) {
            TEST_ASSERT_TRUE_MESSAGE(SENSOR_RAW_AT_REF_MM[sensor][i] > SENSOR_RAW_AT_REF_MM[sensor][i - 1], "curve not increasing");
        }
    }
}

void test_correction_is_monotonic() {
    for (uint8_t sensor = 0; sensor < 10; sensor++) {
        uint16_t previous = 0;
        for (uint16_t raw = 0; raw <= 1500; raw++) {
            uint16_t corrected = correctReading(sensor, raw);
            TEST_ASSERT_TRUE(corrected >= previous);
            previous = corrected;
        }
    }
}

void test_between_points_interpolates() {
    // FRONT: raw 342 -> 300 mm and raw 654 -> 600 mm, so raw 498 is halfway.
    TEST_ASSERT_UINT16_WITHIN(1, 450, correctReading(0, 498));
}

void test_short_reading_is_extended_and_clamped_at_zero() {
    TEST_ASSERT_TRUE(correctReading(0, 100) < 100);
    TEST_ASSERT_EQUAL_UINT16(0, correctReading(0, 0));
}

void test_far_reading_extends_the_last_segment() {
    TEST_ASSERT_TRUE(correctReading(0, 1200) > 1000);
}

void test_out_of_range_and_unknown_sensor_pass_through() {
    TEST_ASSERT_EQUAL_UINT16(SENSOR_CAL_OUT_OF_RANGE, correctReading(0, SENSOR_CAL_OUT_OF_RANGE));
    TEST_ASSERT_EQUAL_UINT16(500, correctReading(10, 500));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_reference_points_map_exactly_to_their_distances);
    RUN_TEST(test_calibration_curves_are_strictly_increasing);
    RUN_TEST(test_correction_is_monotonic);
    RUN_TEST(test_between_points_interpolates);
    RUN_TEST(test_short_reading_is_extended_and_clamped_at_zero);
    RUN_TEST(test_far_reading_extends_the_last_segment);
    RUN_TEST(test_out_of_range_and_unknown_sensor_pass_through);
    return UNITY_END();
}
