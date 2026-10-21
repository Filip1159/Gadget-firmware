#include <EncoderTracker.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_first_reading_is_the_delta_from_zero() {
    EncoderTracker tracker;
    TEST_ASSERT_EQUAL_INT16(120, tracker.update(120).delta);
}

void test_deltas_are_differences_of_consecutive_readings() {
    EncoderTracker tracker;
    tracker.update(100);
    TEST_ASSERT_EQUAL_INT16(50, tracker.update(150).delta);
    TEST_ASSERT_EQUAL_INT16(-30, tracker.update(120).delta);
    TEST_ASSERT_EQUAL_INT16(0, tracker.update(120).delta);
}

void test_requests_a_clear_near_the_limit_and_continues_from_zero() {
    EncoderTracker tracker;
    EncoderTracker::Reading reading = tracker.update(16001);
    TEST_ASSERT_EQUAL_INT16(16001, reading.delta);
    TEST_ASSERT_TRUE(reading.clearCounter);
    // The hardware counter was cleared and kept counting: 40 more ticks read as 40.
    reading = tracker.update(40);
    TEST_ASSERT_EQUAL_INT16(40, reading.delta);
    TEST_ASSERT_FALSE(reading.clearCounter);
}

void test_negative_direction_clears_too() {
    EncoderTracker tracker;
    EncoderTracker::Reading reading = tracker.update(-16001);
    TEST_ASSERT_EQUAL_INT16(-16001, reading.delta);
    TEST_ASSERT_TRUE(reading.clearCounter);
    TEST_ASSERT_EQUAL_INT16(-25, tracker.update(-25).delta);
}

void test_no_counts_lost_over_a_long_drive() {
    EncoderTracker tracker;
    int32_t hardware = 0;  // what the PCNT would hold
    int32_t total = 0;
    for (int tick = 0; tick < 5000; tick++) {
        hardware += 7;  // constant speed
        EncoderTracker::Reading reading = tracker.update(static_cast<int16_t>(hardware));
        total += reading.delta;
        if (reading.clearCounter) {
            hardware = 0;
        }
    }
    TEST_ASSERT_EQUAL_INT32(5000 * 7, total);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_first_reading_is_the_delta_from_zero);
    RUN_TEST(test_deltas_are_differences_of_consecutive_readings);
    RUN_TEST(test_requests_a_clear_near_the_limit_and_continues_from_zero);
    RUN_TEST(test_negative_direction_clears_too);
    RUN_TEST(test_no_counts_lost_over_a_long_drive);
    return UNITY_END();
}
