#include <GyroYaw.h>
#include <math.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

namespace {

constexpr uint32_t STEP_US = 10000;  // 100 Hz
constexpr float RAD_TO_DEG = 180.0f / 3.14159265f;

// Feeds samples at 100 Hz from a simulated gyro; `rate(t)` is the true rate (deg/s) at time t (s).
struct Rig {
    GyroYaw gyro;
    uint32_t nowUs = 1000;
    float bias = 0.4f;  // the gyro's offset, added to every sample

    void run(float seconds, float rate, bool stationary) {
        const int steps = static_cast<int>(seconds * 100.0f + 0.5f);
        for (int i = 0; i < steps; i++) {
            gyro.update(rate + bias, nowUs, stationary);
            nowUs += STEP_US;
        }
    }
    // Stands still long enough for the boot calibration (settle 0.5 s + 2 s window).
    void calibrate() {
        run(3.0f, 0.0f, true);
    }
    float yawDeg() const {
        return gyro.yawRad() * RAD_TO_DEG;
    }
};

}  // namespace

void test_nothing_is_integrated_before_the_first_calibration() {
    Rig rig;
    rig.run(1.0f, 50.0f, false);
    TEST_ASSERT_FALSE(rig.gyro.calibrated());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.gyro.yawRad());
}

void test_boot_calibration_takes_the_still_rate_as_bias() {
    Rig rig;
    rig.run(2.4f, 0.0f, true);  // settle (0.5 s) + 1.9 s: window not complete yet
    TEST_ASSERT_FALSE(rig.gyro.calibrated());
    rig.run(0.3f, 0.0f, true);
    TEST_ASSERT_TRUE(rig.gyro.calibrated());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, rig.gyro.biasDps());
}

void test_a_window_with_motion_in_it_is_rejected() {
    Rig rig;
    // "Stationary" but the gyro swings +-5 deg/s: noisy, not a bias.
    for (int i = 0; i < 400; i++) {
        rig.gyro.update((i % 2 ? 5.0f : -5.0f), rig.nowUs, true);
        rig.nowUs += STEP_US;
    }
    TEST_ASSERT_FALSE(rig.gyro.calibrated());
    rig.calibrate();  // then it holds still
    TEST_ASSERT_TRUE(rig.gyro.calibrated());
}

void test_a_large_constant_offset_is_not_taken_as_bias() {
    Rig rig;
    rig.bias = 40.0f;
    rig.calibrate();
    TEST_ASSERT_FALSE(rig.gyro.calibrated());
}

void test_integrates_a_constant_turn_and_removes_the_bias() {
    Rig rig;
    rig.calibrate();
    rig.run(1.0f, 90.0f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, rig.yawDeg());
    rig.run(1.0f, -90.0f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, rig.yawDeg());
}

void test_a_360_spin_returns_to_zero() {
    Rig rig;
    rig.calibrate();
    rig.run(4.0f, 90.0f, false);  // 360 deg
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, rig.yawDeg());
}

void test_heading_wraps_into_minus_pi_pi() {
    Rig rig;
    rig.calibrate();
    rig.run(2.2f, 90.0f, false);  // 198 deg -> -162 deg
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -162.0f, rig.yawDeg());
    TEST_ASSERT_TRUE(rig.gyro.yawRad() > -3.14159266f && rig.gyro.yawRad() <= 3.14159266f);
}

void test_trapezoid_integrates_a_rate_ramp_exactly() {
    Rig rig;
    rig.calibrate();
    // 0 -> 100 deg/s linearly over 1 s: 50 deg.
    for (int i = 1; i <= 100; i++) {
        rig.gyro.update(i * 1.0f + rig.bias, rig.nowUs, false);
        rig.nowUs += STEP_US;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.6f, 50.0f, rig.yawDeg());
}

void test_a_still_robot_does_not_drift_even_with_a_wrong_bias() {
    Rig rig;
    rig.calibrate();
    rig.bias = 0.9f;  // the gyro warmed up: offset moved by 0.5 deg/s
    rig.run(60.0f, 0.0f, true);
    // Only the 0.5 s settle period integrates the wrong bias (0.25 deg); then the yaw is held. Without
    // the hold it would be 0.5 deg/s * 60 s = 30 deg.
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 0.0f, rig.yawDeg());
}

void test_bias_tracks_drift_while_still() {
    Rig rig;
    rig.calibrate();
    rig.bias = 0.9f;
    rig.run(30.0f, 0.0f, true);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.9f, rig.gyro.biasDps());
    // After tracking, a turn is integrated with the new bias.
    rig.gyro.resetYaw();
    rig.run(1.0f, 90.0f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, rig.yawDeg());
}

void test_pushing_the_still_robot_around_is_still_integrated() {
    Rig rig;
    rig.calibrate();
    rig.run(1.0f, 30.0f, true);  // beyond HOLD_DPS while flagged stationary
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, rig.yawDeg());
}

void test_a_long_gap_between_samples_is_not_integrated() {
    Rig rig;
    rig.calibrate();
    rig.run(0.1f, 90.0f, false);
    const float before = rig.yawDeg();
    rig.nowUs += 500000;  // 0.5 s stall
    rig.run(0.01f, 90.0f, false);
    TEST_ASSERT_EQUAL_FLOAT(before, rig.yawDeg());  // the sample after the gap is not integrated
    rig.run(0.01f, 90.0f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, before + 0.9f, rig.yawDeg());  // normal integration resumes
    TEST_ASSERT_EQUAL_UINT32(1, rig.gyro.skippedGaps());
}

void test_survives_the_microsecond_clock_wrapping() {
    Rig rig;
    rig.nowUs = 0xFFFFFFFFu - 1000000u;  // wraps within the boot calibration
    rig.calibrate();
    TEST_ASSERT_TRUE(rig.gyro.calibrated());
    rig.run(1.0f, 90.0f, false);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, rig.yawDeg());
}

void test_reset_yaw_keeps_the_bias() {
    Rig rig;
    rig.calibrate();
    rig.run(1.0f, 90.0f, false);
    rig.gyro.resetYaw();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, rig.gyro.yawRad());
    TEST_ASSERT_TRUE(rig.gyro.calibrated());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, rig.gyro.biasDps());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_nothing_is_integrated_before_the_first_calibration);
    RUN_TEST(test_boot_calibration_takes_the_still_rate_as_bias);
    RUN_TEST(test_a_window_with_motion_in_it_is_rejected);
    RUN_TEST(test_a_large_constant_offset_is_not_taken_as_bias);
    RUN_TEST(test_integrates_a_constant_turn_and_removes_the_bias);
    RUN_TEST(test_a_360_spin_returns_to_zero);
    RUN_TEST(test_heading_wraps_into_minus_pi_pi);
    RUN_TEST(test_trapezoid_integrates_a_rate_ramp_exactly);
    RUN_TEST(test_a_still_robot_does_not_drift_even_with_a_wrong_bias);
    RUN_TEST(test_bias_tracks_drift_while_still);
    RUN_TEST(test_pushing_the_still_robot_around_is_still_integrated);
    RUN_TEST(test_a_long_gap_between_samples_is_not_integrated);
    RUN_TEST(test_survives_the_microsecond_clock_wrapping);
    RUN_TEST(test_reset_yaw_keeps_the_bias);
    return UNITY_END();
}
