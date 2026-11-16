#include <HeadingFusion.h>
#include <Odometry.h>
#include <RobotGeometry.h>
#include <math.h>
#include <unity.h>

using RobotGeometry::MM_PER_TICK;
using RobotGeometry::TRACK_WIDTH_MM;

static constexpr float PI_F = 3.14159265f;

void setUp() {}
void tearDown() {}

void test_uses_the_wheels_while_the_gyro_is_invalid() {
    HeadingFusion fusion;
    TEST_ASSERT_EQUAL_FLOAT(0.05f, fusion.update(false, 1.0f, 0.05f));
    TEST_ASSERT_FALSE(fusion.usingGyro());
}

void test_first_valid_tick_only_sets_the_baseline() {
    HeadingFusion fusion;
    TEST_ASSERT_EQUAL_FLOAT(0.05f, fusion.update(true, 2.0f, 0.05f));
    TEST_ASSERT_FALSE(fusion.usingGyro());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, fusion.update(true, 2.3f, 0.05f));
    TEST_ASSERT_TRUE(fusion.usingGyro());
}

void test_gyro_change_wins_over_the_wheels() {
    HeadingFusion fusion;
    fusion.update(true, 0.0f, 0.0f);
    // The wheels slipped and saw 0.2 rad, the gyro saw 0.1.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, fusion.update(true, 0.1f, 0.2f));
}

void test_yaw_wrapping_through_pi_is_a_small_change() {
    HeadingFusion fusion;
    fusion.update(true, 3.10f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.1f, fusion.update(true, -3.0832f, 0.0f));  // 3.10 -> -3.0832 = +0.1 rad
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.1f, fusion.update(true, 3.10f, 0.0f));
}

void test_scale_multiplies_the_gyro_change() {
    HeadingFusion fusion(1.02f);
    fusion.update(true, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.102f, fusion.update(true, 0.1f, 0.0f));
}

void test_gap_in_gyro_validity_rebaselines_instead_of_jumping() {
    HeadingFusion fusion;
    fusion.update(true, 0.0f, 0.0f);
    fusion.update(true, 0.1f, 0.0f);
    fusion.update(false, 0.0f, 0.04f);  // gyro stale: wheels
    // The gyro is back and reads 1.0 rad now: that must not become a 0.9 rad jump.
    TEST_ASSERT_EQUAL_FLOAT(0.03f, fusion.update(true, 1.0f, 0.03f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, fusion.update(true, 1.05f, 0.0f));
}

void test_a_slipping_360_spin_returns_to_zero_with_the_gyro_but_not_with_the_wheels() {
    // 24 control ticks of 15 deg each. The wheels only deliver 90% of the turn (slip).
    Odometry fused(MM_PER_TICK, TRACK_WIDTH_MM);
    Odometry wheelsOnly(MM_PER_TICK, TRACK_WIDTH_MM);
    HeadingFusion fusion;
    const float turn = 15.0f * PI_F / 180.0f;
    const int32_t slipTicks = static_cast<int32_t>(lroundf(0.9f * turn * TRACK_WIDTH_MM / 2.0f / MM_PER_TICK));
    float gyroYaw = 0.0f;
    fused.update(0, 0, fusion.update(true, gyroYaw, 0.0f));  // baseline tick, standing still
    for (int i = 0; i < 24; i++) {
        gyroYaw += turn;
        if (gyroYaw > PI_F) {
            gyroYaw -= 2.0f * PI_F;
        }
        const float wheel = fused.wheelHeadingChange(-slipTicks, slipTicks);
        fused.update(-slipTicks, slipTicks, fusion.update(true, gyroYaw, wheel));
        wheelsOnly.update(-slipTicks, slipTicks);
    }
    // The gyro saw the full 360 deg, so the heading is back at 0 ...
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fused.pose().theta);
    // ... the wheels only delivered 24 * 13.5 = 324 deg, i.e. 36 deg short of a lap.
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -36.0f * PI_F / 180.0f, wheelsOnly.pose().theta);
}

void test_odometry_lays_the_distance_along_the_overridden_heading() {
    Odometry odo(MM_PER_TICK, TRACK_WIDTH_MM);
    // 1000 mm of wheel travel on both wheels, but the gyro says the robot turned 90 deg during the tick:
    // translation along the mean heading (45 deg).
    const int32_t ticks = static_cast<int32_t>(lroundf(1000.0f / MM_PER_TICK));
    odo.update(ticks, ticks, PI_F / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f * cosf(PI_F / 4.0f), odo.pose().x);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f * sinf(PI_F / 4.0f), odo.pose().y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PI_F / 2.0f, odo.pose().theta);
}

void test_wheel_only_update_is_unchanged_by_the_override_overload() {
    Odometry a(MM_PER_TICK, TRACK_WIDTH_MM);
    Odometry b(MM_PER_TICK, TRACK_WIDTH_MM);
    a.update(900, 1100);
    b.update(900, 1100, b.wheelHeadingChange(900, 1100));
    TEST_ASSERT_EQUAL_FLOAT(a.pose().x, b.pose().x);
    TEST_ASSERT_EQUAL_FLOAT(a.pose().y, b.pose().y);
    TEST_ASSERT_EQUAL_FLOAT(a.pose().theta, b.pose().theta);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_uses_the_wheels_while_the_gyro_is_invalid);
    RUN_TEST(test_first_valid_tick_only_sets_the_baseline);
    RUN_TEST(test_gyro_change_wins_over_the_wheels);
    RUN_TEST(test_yaw_wrapping_through_pi_is_a_small_change);
    RUN_TEST(test_scale_multiplies_the_gyro_change);
    RUN_TEST(test_gap_in_gyro_validity_rebaselines_instead_of_jumping);
    RUN_TEST(test_a_slipping_360_spin_returns_to_zero_with_the_gyro_but_not_with_the_wheels);
    RUN_TEST(test_odometry_lays_the_distance_along_the_overridden_heading);
    RUN_TEST(test_wheel_only_update_is_unchanged_by_the_override_overload);
    return UNITY_END();
}
