#include <ImuOrientation.h>
#include <string.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

namespace {

using Verdict = ImuOrientationCheck::Verdict;

// Runs a whole check at 100 Hz: `still` samples during the still phase, `spin` during the spin phase.
ImuOrientationCheck::Result run(const ImuSample& still, const ImuSample& spin, bool nullSamples = false) {
    ImuOrientationCheck check;
    uint32_t now = 1000;
    check.start(now);
    bool done = false;
    int ticks = 0;
    while (!done && ticks++ < 1000) {
        const ImuSample& sample = check.wantSpin() ? spin : still;
        done = check.update(now, nullSamples ? nullptr : &sample);
        now += 10;
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_FALSE(check.active());
    return check.result();
}

ImuSample sample(float ax, float ay, float az, float gx, float gy, float gz) {
    ImuSample s;
    s.ax = ax;
    s.ay = ay;
    s.az = az;
    s.gx = gx;
    s.gy = gy;
    s.gz = gz;
    return s;
}

}  // namespace

void test_level_board_is_identity() {
    const auto r = run(sample(0.01f, -0.02f, 1.0f, 0.5f, -0.3f, 0.2f), sample(0.0f, 0.0f, 1.0f, 0.5f, -0.3f, 90.2f));
    TEST_ASSERT_EQUAL(Verdict::Ok, r.verdict);
    TEST_ASSERT_EQUAL_STRING("+Z", r.up.name());
    TEST_ASSERT_EQUAL_STRING("+Z", r.yaw.name());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, r.rateDps);  // the 0.2 dps stationary offset is removed
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, r.tiltDeg);
}

void test_upside_down_board_reads_minus_z_on_both() {
    const auto r = run(sample(0, 0, -1.0f, 0, 0, 0), sample(0, 0, -1.0f, 0, 0, -80.0f));
    TEST_ASSERT_EQUAL(Verdict::Ok, r.verdict);
    TEST_ASSERT_EQUAL_STRING("-Z", r.up.name());
    TEST_ASSERT_EQUAL_STRING("-Z", r.yaw.name());
}

void test_board_on_its_side_is_found_on_another_axis() {
    const auto r = run(sample(0.0f, 1.0f, 0.0f, 0, 0, 0), sample(0.0f, 1.0f, 0.0f, 0, 70.0f, 0));
    TEST_ASSERT_EQUAL(Verdict::Ok, r.verdict);
    TEST_ASSERT_EQUAL_STRING("+Y", r.up.name());
    TEST_ASSERT_EQUAL_STRING("+Y", r.yaw.name());
}

void test_spin_in_the_wrong_direction_is_inconsistent() {
    const auto r = run(sample(0, 0, 1.0f, 0, 0, 0), sample(0, 0, 1.0f, 0, 0, -90.0f));
    TEST_ASSERT_EQUAL(Verdict::Inconsistent, r.verdict);
    TEST_ASSERT_EQUAL_STRING("+Z", r.up.name());
    TEST_ASSERT_EQUAL_STRING("-Z", r.yaw.name());
}

void test_spin_about_another_axis_than_gravity_is_inconsistent() {
    const auto r = run(sample(0, 0, 1.0f, 0, 0, 0), sample(0, 0, 1.0f, 90.0f, 0, 0));
    TEST_ASSERT_EQUAL(Verdict::Inconsistent, r.verdict);
}

void test_tilted_board_is_flagged() {
    // ~35 deg off level: gravity splits between X and Z.
    const auto r = run(sample(0.57f, 0, 0.82f, 0, 0, 0), sample(0.57f, 0, 0.82f, 0, 0, 90.0f));
    TEST_ASSERT_EQUAL(Verdict::Tilted, r.verdict);
    TEST_ASSERT_FLOAT_WITHIN(1.5f, 35.0f, r.tiltDeg);
}

void test_slow_spin_cannot_decide() {
    const auto r = run(sample(0, 0, 1.0f, 0, 0, 0), sample(0, 0, 1.0f, 0, 0, 5.0f));
    TEST_ASSERT_EQUAL(Verdict::NoSpin, r.verdict);
}

void test_robot_moving_during_the_still_phase() {
    const auto r = run(sample(0, 0, 1.0f, 0, 0, 60.0f), sample(0, 0, 1.0f, 0, 0, 90.0f));
    TEST_ASSERT_EQUAL(Verdict::Moving, r.verdict);
}

void test_accelerometer_not_reading_one_g() {
    const auto r = run(sample(0, 0, 0.1f, 0, 0, 0), sample(0, 0, 0.1f, 0, 0, 90.0f));
    TEST_ASSERT_EQUAL(Verdict::NoGravity, r.verdict);
}

void test_failed_reads_end_with_no_data_and_still_finish() {
    const auto r = run(sample(0, 0, 1.0f, 0, 0, 0), sample(0, 0, 1.0f, 0, 0, 90.0f), true);
    TEST_ASSERT_EQUAL(Verdict::NoData, r.verdict);
}

void test_spin_is_requested_only_after_the_still_phase() {
    ImuOrientationCheck check;
    const ImuSample s = sample(0, 0, 1.0f, 0, 0, 0);
    TEST_ASSERT_FALSE(check.active());
    check.start(0);
    TEST_ASSERT_TRUE(check.active());
    TEST_ASSERT_FALSE(check.wantSpin());
    check.update(ImuOrientationCheck::STILL_MS - 1, &s);
    TEST_ASSERT_FALSE(check.wantSpin());
    check.update(ImuOrientationCheck::STILL_MS, &s);
    TEST_ASSERT_TRUE(check.wantSpin());
    check.abort();
    TEST_ASSERT_FALSE(check.active());
    TEST_ASSERT_FALSE(check.wantSpin());
    TEST_ASSERT_EQUAL(Verdict::Aborted, check.result().verdict);
    TEST_ASSERT_EQUAL_STRING("aborted", ImuOrientationCheck::verdictName(Verdict::Aborted));
    TEST_ASSERT_FALSE(check.update(5000, &s));  // idle: ignored
}

void test_abort_while_idle_keeps_the_last_result() {
    ImuOrientationCheck check;
    const ImuSample still = sample(0, 0, 1.0f, 0, 0, 0);
    const ImuSample spin = sample(0, 0, 1.0f, 0, 0, 90.0f);
    uint32_t now = 0;
    check.start(now);
    bool done = false;
    for (int i = 0; i < 1000 && !done; i++) {
        done = check.update(now, check.wantSpin() ? &spin : &still);
        now += 10;
    }
    TEST_ASSERT_TRUE(done);
    check.abort();
    TEST_ASSERT_EQUAL(Verdict::Ok, check.result().verdict);
}

void test_survives_millis_wraparound() {
    ImuOrientationCheck check;
    const ImuSample still = sample(0, 0, 1.0f, 0, 0, 0);
    const ImuSample spin = sample(0, 0, 1.0f, 0, 0, 90.0f);
    uint32_t now = 0xFFFFFFFFu - 500;
    check.start(now);
    bool done = false;
    for (int i = 0; i < 1000 && !done; i++) {
        done = check.update(now, check.wantSpin() ? &spin : &still);
        now += 10;
    }
    TEST_ASSERT_TRUE(done);
    TEST_ASSERT_EQUAL(Verdict::Ok, check.result().verdict);
}

void test_axis_names() {
    TEST_ASSERT_EQUAL_STRING("?", SignedAxis().name());
    TEST_ASSERT_EQUAL_STRING("-X", (SignedAxis{0, -1}).name());
    TEST_ASSERT_EQUAL_STRING("+Y", (SignedAxis{1, 1}).name());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_level_board_is_identity);
    RUN_TEST(test_upside_down_board_reads_minus_z_on_both);
    RUN_TEST(test_board_on_its_side_is_found_on_another_axis);
    RUN_TEST(test_spin_in_the_wrong_direction_is_inconsistent);
    RUN_TEST(test_spin_about_another_axis_than_gravity_is_inconsistent);
    RUN_TEST(test_tilted_board_is_flagged);
    RUN_TEST(test_slow_spin_cannot_decide);
    RUN_TEST(test_robot_moving_during_the_still_phase);
    RUN_TEST(test_accelerometer_not_reading_one_g);
    RUN_TEST(test_failed_reads_end_with_no_data_and_still_finish);
    RUN_TEST(test_spin_is_requested_only_after_the_still_phase);
    RUN_TEST(test_abort_while_idle_keeps_the_last_result);
    RUN_TEST(test_survives_millis_wraparound);
    RUN_TEST(test_axis_names);
    return UNITY_END();
}
