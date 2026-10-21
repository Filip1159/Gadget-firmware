#include <CalibrationRun.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

using Result = CalibrationRun::StartResult;

void test_starts_with_valid_arguments() {
    CalibrationRun run;
    TEST_ASSERT_FALSE(run.active());
    TEST_ASSERT_EQUAL(Result::Started, run.start("300 5", false));
    TEST_ASSERT_TRUE(run.active());
    TEST_ASSERT_EQUAL_UINT16(300, run.distanceMm());
    TEST_ASSERT_EQUAL_UINT16(5, run.total());
    TEST_ASSERT_EQUAL_UINT16(0, run.done());
}

void test_rejects_bad_arguments_without_starting() {
    CalibrationRun run;
    TEST_ASSERT_EQUAL(Result::Invalid, run.start("", false));
    TEST_ASSERT_EQUAL(Result::Invalid, run.start("0 5", false));
    TEST_ASSERT_EQUAL(Result::Invalid, run.start("300 501", false));
    TEST_ASSERT_EQUAL(Result::Invalid, run.start("2001 5", false));
    TEST_ASSERT_FALSE(run.active());
}

void test_rejects_a_start_while_the_robot_is_moving() {
    CalibrationRun run;
    TEST_ASSERT_EQUAL(Result::RobotMoving, run.start("300 5", true));
    TEST_ASSERT_FALSE(run.active());
}

void test_a_second_start_while_running_is_rejected_and_keeps_the_run() {
    CalibrationRun run;
    run.start("300 5", false);
    run.recordSweep();
    TEST_ASSERT_EQUAL(Result::AlreadyRunning, run.start("500 10", false));
    TEST_ASSERT_EQUAL_UINT16(300, run.distanceMm());
    TEST_ASSERT_EQUAL_UINT16(1, run.done());
}

void test_invalid_arguments_are_reported_before_a_running_run() {
    CalibrationRun run;
    run.start("300 5", false);
    TEST_ASSERT_EQUAL(Result::Invalid, run.start("x", false));
}

void test_the_last_sweep_is_flagged() {
    CalibrationRun run;
    run.start("300 3", false);
    TEST_ASSERT_FALSE(run.recordSweep());
    TEST_ASSERT_FALSE(run.recordSweep());
    TEST_ASSERT_TRUE(run.recordSweep());
    TEST_ASSERT_EQUAL_UINT16(3, run.done());
}

void test_end_stops_the_run_but_keeps_the_numbers_for_cal_done() {
    CalibrationRun run;
    run.start("300 10", false);
    run.recordSweep();
    run.recordSweep();
    run.end();
    TEST_ASSERT_FALSE(run.active());
    TEST_ASSERT_EQUAL_UINT16(300, run.distanceMm());
    TEST_ASSERT_EQUAL_UINT16(2, run.done());
}

void test_a_new_run_starts_from_zero_after_the_previous_one_ended() {
    CalibrationRun run;
    run.start("300 2", false);
    run.recordSweep();
    run.end();
    TEST_ASSERT_EQUAL(Result::Started, run.start("600 4", false));
    TEST_ASSERT_EQUAL_UINT16(0, run.done());
    TEST_ASSERT_EQUAL_UINT16(600, run.distanceMm());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_starts_with_valid_arguments);
    RUN_TEST(test_rejects_bad_arguments_without_starting);
    RUN_TEST(test_rejects_a_start_while_the_robot_is_moving);
    RUN_TEST(test_a_second_start_while_running_is_rejected_and_keeps_the_run);
    RUN_TEST(test_invalid_arguments_are_reported_before_a_running_run);
    RUN_TEST(test_the_last_sweep_is_flagged);
    RUN_TEST(test_end_stops_the_run_but_keeps_the_numbers_for_cal_done);
    RUN_TEST(test_a_new_run_starts_from_zero_after_the_previous_one_ended);
    return UNITY_END();
}
