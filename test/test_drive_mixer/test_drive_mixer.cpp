#include <DriveMixer.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

static void assertTargets(
    DriveDirection direction, uint16_t speed,
    WheelDirection leftDir, uint16_t leftSpeed, WheelDirection rightDir, uint16_t rightSpeed
) {
    const WheelTargets t = mixDrive(direction, speed);
    TEST_ASSERT_EQUAL_INT(leftDir, t.leftDir);
    TEST_ASSERT_EQUAL_UINT16(leftSpeed, t.leftSpeed);
    TEST_ASSERT_EQUAL_INT(rightDir, t.rightDir);
    TEST_ASSERT_EQUAL_UINT16(rightSpeed, t.rightSpeed);
}

void test_straight_drives_both_wheels_the_same() {
    assertTargets(FORWARD, 1000, WHEEL_FORWARD, 1000, WHEEL_FORWARD, 1000);
    assertTargets(BACKWARD, 1000, WHEEL_BACKWARD, 1000, WHEEL_BACKWARD, 1000);
}

void test_forward_arcs_slow_the_inner_wheel() {
    assertTargets(FORWARD_LEFT, 1000, WHEEL_FORWARD, 700, WHEEL_FORWARD, 1000);
    assertTargets(FORWARD_RIGHT, 1000, WHEEL_FORWARD, 1000, WHEEL_FORWARD, 700);
}

void test_backward_arcs_slow_the_inner_wheel() {
    assertTargets(BACKWARD_LEFT, 1000, WHEEL_BACKWARD, 700, WHEEL_BACKWARD, 1000);
    assertTargets(BACKWARD_RIGHT, 1000, WHEEL_BACKWARD, 1000, WHEEL_BACKWARD, 700);
}

void test_spin_in_place_runs_the_wheels_in_opposite_directions() {
    assertTargets(LEFT, 800, WHEEL_BACKWARD, 800, WHEEL_FORWARD, 800);
    assertTargets(RIGHT, 800, WHEEL_FORWARD, 800, WHEEL_BACKWARD, 800);
}

void test_stop_is_zero_speed_whatever_the_target() {
    assertTargets(STOP, 1500, WHEEL_FORWARD, 0, WHEEL_FORWARD, 0);
}

void test_inner_wheel_speed_is_truncated() {
    // 999 * 0.7 = 699.3 -> 699; 3 * 0.7 = 2.1 -> 2
    assertTargets(FORWARD_LEFT, 999, WHEEL_FORWARD, 699, WHEEL_FORWARD, 999);
    assertTargets(FORWARD_RIGHT, 3, WHEEL_FORWARD, 3, WHEEL_FORWARD, 2);
}

void test_zero_and_max_speed() {
    assertTargets(FORWARD_LEFT, 0, WHEEL_FORWARD, 0, WHEEL_FORWARD, 0);
    assertTargets(FORWARD_LEFT, 2000, WHEEL_FORWARD, 1400, WHEEL_FORWARD, 2000);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_straight_drives_both_wheels_the_same);
    RUN_TEST(test_forward_arcs_slow_the_inner_wheel);
    RUN_TEST(test_backward_arcs_slow_the_inner_wheel);
    RUN_TEST(test_spin_in_place_runs_the_wheels_in_opposite_directions);
    RUN_TEST(test_stop_is_zero_speed_whatever_the_target);
    RUN_TEST(test_inner_wheel_speed_is_truncated);
    RUN_TEST(test_zero_and_max_speed);
    return UNITY_END();
}
