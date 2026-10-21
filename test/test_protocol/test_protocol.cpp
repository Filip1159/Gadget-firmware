#include <Protocol.h>
#include <limits.h>
#include <math.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

// Frame strings: the exact examples of CLAUDE.md and the UI protocol.spec.ts.

void test_pose_frame_rounds_mm_and_converts_heading_to_milliradians() {
    Pose pose;
    pose.x = 1000.4f;
    pose.y = -12.6f;
    pose.theta = 1.5708f;
    TEST_ASSERT_EQUAL_STRING("POSE X=1000 Y=-13 TH=1571 T=123456 Q=0", formatPose(pose, 123456).c_str());
}

void test_sweep_pose_frame_has_start_and_end() {
    StampedPose start, end;
    start.pose.x = 0;
    start.tMs = 1000;
    end.pose.x = 20.4f;
    end.pose.y = 1;
    end.pose.theta = -0.01f;
    end.tMs = 1390;
    TEST_ASSERT_EQUAL_STRING(
        "SWEEP_POSE T0=1000 X0=0 Y0=0 TH0=0 T1=1390 X1=20 Y1=1 TH1=-10", formatSweepPose(start, end).c_str()
    );
}

void test_sweep_fields_write_oor_for_the_out_of_range_marker() {
    const char* names[] = {"FRONT", "FRONT_LEFT_45", "BACK"};
    const uint16_t readings[] = {412, PROTOCOL_OUT_OF_RANGE, 0};
    TEST_ASSERT_EQUAL_STRING(" FRONT=412 FRONT_LEFT_45=OOR BACK=0", formatSweep(names, readings, 3).c_str());
    TEST_ASSERT_EQUAL_STRING("", formatSweep(names, readings, 0).c_str());
}

void test_state_frame() {
    TEST_ASSERT_EQUAL_STRING(
        "STATE SPEED=800 P=0.1500 I=0.0500 D=0.0000 MEASURING=1", formatState(800, 0.15f, 0.05f, 0.0f, true).c_str()
    );
    TEST_ASSERT_EQUAL_STRING(
        "STATE SPEED=0 P=1.0000 I=2.0000 D=3.0000 MEASURING=0", formatState(0, 1, 2, 3, false).c_str()
    );
}

void test_cal_frames() {
    const char* names[] = {"FRONT", "BACK"};
    const uint16_t readings[] = {300, PROTOCOL_OUT_OF_RANGE};
    TEST_ASSERT_EQUAL_STRING(
        "CAL 300 5/100 FRONT=300 BACK=OOR", formatCal(300, 5, 100, formatSweep(names, readings, 2)).c_str()
    );
    TEST_ASSERT_EQUAL_STRING("CAL_DONE 300 100", formatCalDone(300, 100).c_str());
}

void test_parse_uint_accepts_digits_with_surrounding_whitespace() {
    long value = -1;
    TEST_ASSERT_TRUE(parseUint("800", value));
    TEST_ASSERT_EQUAL_INT(800, value);
    TEST_ASSERT_TRUE(parseUint("  42 ", value));
    TEST_ASSERT_EQUAL_INT(42, value);
    TEST_ASSERT_TRUE(parseUint("0", value));
    TEST_ASSERT_EQUAL_INT(0, value);
}

void test_parse_uint_rejects_everything_else() {
    long value = 7;
    const char* bad[] = {"", "   ", "-5", "+5", "12a", "a12", "1 2", "1.5", "0x10"};
    for (const char* text : bad) {
        TEST_ASSERT_FALSE_MESSAGE(parseUint(text, value), text);
    }
    TEST_ASSERT_EQUAL_INT(7, value);  // untouched on failure
}

void test_parse_uint_saturates_instead_of_wrapping() {
    long value = 0;
    TEST_ASSERT_TRUE(parseUint("99999999999999999999", value));
    TEST_ASSERT_EQUAL_INT(LONG_MAX, value);
}

void test_parse_pid_args() {
    float p, i, d;
    TEST_ASSERT_TRUE(parsePidArgs("0.15 0.05 0", p, i, d));
    TEST_ASSERT_EQUAL_FLOAT(0.15f, p);
    TEST_ASSERT_EQUAL_FLOAT(0.05f, i);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, d);
}

void test_parse_pid_args_rejects_bad_input_and_leaves_outputs_alone() {
    float p = 9, i = 9, d = 9;
    const char* bad[] = {"", "1 2", "1 2 3 4", "1 2 x", "-1 2 3", "1 -2 3", "1 2 -3", "nan 1 2", "1 inf 2", "1,2,3"};
    for (const char* args : bad) {
        TEST_ASSERT_FALSE_MESSAGE(parsePidArgs(args, p, i, d), args);
    }
    TEST_ASSERT_EQUAL_FLOAT(9.0f, p);
    TEST_ASSERT_EQUAL_FLOAT(9.0f, i);
    TEST_ASSERT_EQUAL_FLOAT(9.0f, d);
}

void test_parse_cal_args() {
    long distance = 0, count = 0;
    TEST_ASSERT_TRUE(parseCalArgs("300 100", 2000, 500, distance, count));
    TEST_ASSERT_EQUAL_INT(300, distance);
    TEST_ASSERT_EQUAL_INT(100, count);
    TEST_ASSERT_TRUE(parseCalArgs("1 1", 2000, 500, distance, count));
    TEST_ASSERT_TRUE(parseCalArgs("2000 500", 2000, 500, distance, count));
}

void test_parse_cal_args_rejects_bad_input() {
    long distance = 0, count = 0;
    const char* bad[] = {"", "300", "300 ", " 5", "0 5", "300 0", "2001 5", "300 501", "-1 5", "300 -1",
                         "a 5",  "300 b", "300 5 7", "300  5x"};
    for (const char* args : bad) {
        TEST_ASSERT_FALSE_MESSAGE(parseCalArgs(args, 2000, 500, distance, count), args);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pose_frame_rounds_mm_and_converts_heading_to_milliradians);
    RUN_TEST(test_sweep_pose_frame_has_start_and_end);
    RUN_TEST(test_sweep_fields_write_oor_for_the_out_of_range_marker);
    RUN_TEST(test_state_frame);
    RUN_TEST(test_cal_frames);
    RUN_TEST(test_parse_uint_accepts_digits_with_surrounding_whitespace);
    RUN_TEST(test_parse_uint_rejects_everything_else);
    RUN_TEST(test_parse_uint_saturates_instead_of_wrapping);
    RUN_TEST(test_parse_pid_args);
    RUN_TEST(test_parse_pid_args_rejects_bad_input_and_leaves_outputs_alone);
    RUN_TEST(test_parse_cal_args);
    RUN_TEST(test_parse_cal_args_rejects_bad_input);
    return UNITY_END();
}
