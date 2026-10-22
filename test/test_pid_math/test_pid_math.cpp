#include <PidMath.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

static const PidGains kPOnly = {0.1f, 0.0f, 0.0f};

void test_proportional_output_is_gain_times_error() {
    PidState state;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, pidStep(kPOnly, state, 300, 100, 0.1f));
}

void test_output_is_clamped_to_the_pwm_range() {
    PidState state;
    TEST_ASSERT_EQUAL_FLOAT(PWM_MAX, pidStep(kPOnly, state, 5000, 0, 0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pidStep(kPOnly, state, 0, 500, 0.1f));
}

void test_integral_accumulates_error_over_time() {
    PidState state;
    const PidGains gains = {0.0f, 1.0f, 0.0f};
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, pidStep(gains, state, 100, 0, 0.1f));  // 100 * 0.1
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, pidStep(gains, state, 100, 0, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, state.integral);
}

void test_derivative_reacts_to_the_change_of_error() {
    PidState state;
    const PidGains gains = {0.0f, 0.0f, 0.01f};
    pidStep(gains, state, 100, 100, 0.1f);                                        // error 0
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, pidStep(gains, state, 100, 50, 0.1f));  // (50-0)/0.1 * 0.01
}

void test_integral_stops_growing_while_saturated_high() {
    PidState state;
    const PidGains gains = {1.0f, 1.0f, 0.0f};
    pidStep(gains, state, 1000, 0, 0.1f);  // unclamped far above PWM_MAX, error > 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.integral);
}

void test_integral_stops_growing_while_saturated_low() {
    PidState state;
    const PidGains gains = {1.0f, 1.0f, 0.0f};
    pidStep(gains, state, 0, 1000, 0.1f);  // unclamped far below 0, error < 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.integral);
}

void test_integral_resumes_when_the_error_pulls_out_of_saturation() {
    PidState state;
    state.integral = 4000.0f;  // wound up
    const PidGains gains = {0.1f, 1.0f, 0.0f};
    // Saturated high, but the error is negative: it helps unwind, so the integral keeps integrating.
    pidStep(gains, state, 90, 100, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3999.0f, state.integral);
}

void test_reset_clears_the_state() {
    PidState state;
    pidStep({0.1f, 1.0f, 0.1f}, state, 100, 0, 0.1f);
    state.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.previousError);
}

void test_sanitize_dt_keeps_plausible_values() {
    TEST_ASSERT_EQUAL_FLOAT(0.02f, sanitizeDt(0.02f));
    TEST_ASSERT_EQUAL_FLOAT(0.1f, sanitizeDt(0.1f));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, sanitizeDt(0.5f));
}

void test_sanitize_dt_falls_back_for_implausible_values() {
    TEST_ASSERT_EQUAL_FLOAT(0.1f, sanitizeDt(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.1f, sanitizeDt(0.019f));
    TEST_ASSERT_EQUAL_FLOAT(0.1f, sanitizeDt(0.51f));
    TEST_ASSERT_EQUAL_FLOAT(0.1f, sanitizeDt(30.0f));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_proportional_output_is_gain_times_error);
    RUN_TEST(test_output_is_clamped_to_the_pwm_range);
    RUN_TEST(test_integral_accumulates_error_over_time);
    RUN_TEST(test_derivative_reacts_to_the_change_of_error);
    RUN_TEST(test_integral_stops_growing_while_saturated_high);
    RUN_TEST(test_integral_stops_growing_while_saturated_low);
    RUN_TEST(test_integral_resumes_when_the_error_pulls_out_of_saturation);
    RUN_TEST(test_reset_clears_the_state);
    RUN_TEST(test_sanitize_dt_keeps_plausible_values);
    RUN_TEST(test_sanitize_dt_falls_back_for_implausible_values);
    return UNITY_END();
}
