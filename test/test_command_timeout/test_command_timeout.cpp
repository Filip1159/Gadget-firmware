#include <CommandTimeout.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_expires_exactly_at_the_timeout() {
    CommandTimeout timeout;
    timeout.refresh(5000);
    TEST_ASSERT_FALSE(timeout.expired(5000, true));
    TEST_ASSERT_FALSE(timeout.expired(5000 + COMMAND_TIMEOUT_MS - 1, true));
    TEST_ASSERT_TRUE(timeout.expired(5000 + COMMAND_TIMEOUT_MS, true));
}

void test_refresh_restarts_the_window() {
    CommandTimeout timeout;
    timeout.refresh(0);
    timeout.refresh(900);
    TEST_ASSERT_FALSE(timeout.expired(1500, true));
    TEST_ASSERT_TRUE(timeout.expired(900 + COMMAND_TIMEOUT_MS, true));
}

void test_a_stopped_robot_never_times_out() {
    CommandTimeout timeout;
    timeout.refresh(0);
    TEST_ASSERT_FALSE(timeout.expired(1000000, false));
}

void test_correct_across_millis_wraparound() {
    CommandTimeout timeout;
    timeout.refresh(0xFFFFFF00u);
    TEST_ASSERT_FALSE(timeout.expired(0xFFFFFF00u + 500u, true));  // wrapped, only 500 ms elapsed
    TEST_ASSERT_TRUE(timeout.expired(0xFFFFFF00u + COMMAND_TIMEOUT_MS, true));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_expires_exactly_at_the_timeout);
    RUN_TEST(test_refresh_restarts_the_window);
    RUN_TEST(test_a_stopped_robot_never_times_out);
    RUN_TEST(test_correct_across_millis_wraparound);
    return UNITY_END();
}
