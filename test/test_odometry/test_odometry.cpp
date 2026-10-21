#include <Odometry.h>
#include <RobotGeometry.h>
#include <math.h>
#include <unity.h>

using RobotGeometry::MM_PER_TICK;
using RobotGeometry::TRACK_WIDTH_MM;

static constexpr float PI_F = 3.14159265f;

static int32_t ticksFor(float mm) {
    return static_cast<int32_t>(lroundf(mm / MM_PER_TICK));
}

static Odometry makeOdometry() {
    return Odometry(MM_PER_TICK, TRACK_WIDTH_MM);
}

void setUp() {}
void tearDown() {}

void test_starts_at_origin() {
    Odometry odo = makeOdometry();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, odo.pose().x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, odo.pose().y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, odo.pose().theta);
}

void test_straight_forward_one_metre() {
    Odometry odo = makeOdometry();
    int32_t ticks = ticksFor(1000.0f);
    for (int i = 0; i < 10; i++) {
        odo.update(ticks / 10, ticks / 10);
    }
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 1000.0f, odo.pose().x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, odo.pose().y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, odo.pose().theta);
}

void test_backward_is_negative_x() {
    Odometry odo = makeOdometry();
    int32_t ticks = ticksFor(500.0f);
    odo.update(-ticks, -ticks);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -500.0f, odo.pose().x);
}

void test_spin_left_in_place_is_counter_clockwise() {
    Odometry odo = makeOdometry();
    // Quarter turn: each wheel travels a quarter of the circle of diameter = track width.
    int32_t ticks = ticksFor(PI_F * TRACK_WIDTH_MM / 4.0f);
    odo.update(-ticks, ticks);  // LEFT: left wheel backward, right wheel forward
    TEST_ASSERT_FLOAT_WITHIN(0.01f, PI_F / 2.0f, odo.pose().theta);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, odo.pose().x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, odo.pose().y);
}

void test_spin_right_in_place_is_clockwise() {
    Odometry odo = makeOdometry();
    int32_t ticks = ticksFor(PI_F * TRACK_WIDTH_MM / 4.0f);
    odo.update(ticks, -ticks);  // RIGHT
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -PI_F / 2.0f, odo.pose().theta);
}

void test_full_spin_returns_to_zero_heading() {
    Odometry odo = makeOdometry();
    int32_t ticks = ticksFor(PI_F * TRACK_WIDTH_MM / 8.0f);  // an eighth of a turn
    for (int i = 0; i < 8; i++) {
        odo.update(-ticks, ticks);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, odo.pose().theta);
}

void test_heading_wraps_into_minus_pi_to_pi() {
    Odometry odo = makeOdometry();
    int32_t ticks = ticksFor(PI_F * TRACK_WIDTH_MM / 4.0f);  // 90 degrees per call
    for (int i = 0; i < 3; i++) {
        odo.update(-ticks, ticks);
    }
    // 270 degrees counter-clockwise is -90 degrees.
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -PI_F / 2.0f, odo.pose().theta);
}

void test_quarter_circle_arc() {
    Odometry odo = makeOdometry();
    // Left turn along a circle of radius 500 mm (robot centre), 90 degrees. Done in many small
    // steps with cumulative rounding so integer ticks do not accumulate a bias.
    const float radius = 500.0f;
    const int steps = 50;
    const float leftTotal = ticksFor((radius - TRACK_WIDTH_MM / 2.0f) * PI_F / 2.0f);
    const float rightTotal = ticksFor((radius + TRACK_WIDTH_MM / 2.0f) * PI_F / 2.0f);
    for (int k = 0; k < steps; k++) {
        int32_t left = lroundf(leftTotal * (k + 1) / steps) - lroundf(leftTotal * k / steps);
        int32_t right = lroundf(rightTotal * (k + 1) / steps) - lroundf(rightTotal * k / steps);
        odo.update(left, right);
    }
    TEST_ASSERT_FLOAT_WITHIN(3.0f, radius, odo.pose().x);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, radius, odo.pose().y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, PI_F / 2.0f, odo.pose().theta);
}

void test_reset_zeroes_pose() {
    Odometry odo = makeOdometry();
    odo.update(1000, 1500);
    odo.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, odo.pose().x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, odo.pose().y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, odo.pose().theta);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_at_origin);
    RUN_TEST(test_straight_forward_one_metre);
    RUN_TEST(test_backward_is_negative_x);
    RUN_TEST(test_spin_left_in_place_is_counter_clockwise);
    RUN_TEST(test_spin_right_in_place_is_clockwise);
    RUN_TEST(test_full_spin_returns_to_zero_heading);
    RUN_TEST(test_heading_wraps_into_minus_pi_to_pi);
    RUN_TEST(test_quarter_circle_arc);
    RUN_TEST(test_reset_zeroes_pose);
    return UNITY_END();
}
