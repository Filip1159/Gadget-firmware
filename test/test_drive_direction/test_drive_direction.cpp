#include <DriveDirection.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_every_wire_name_parses_to_its_direction() {
    const struct {
        const char* name;
        DriveDirection direction;
    } expected[] = {
        {"FORWARD", FORWARD},   {"FORWARD_LEFT", FORWARD_LEFT},     {"FORWARD_RIGHT", FORWARD_RIGHT},
        {"LEFT", LEFT},         {"STOP", STOP},                     {"RIGHT", RIGHT},
        {"BACKWARD", BACKWARD}, {"BACKWARD_LEFT", BACKWARD_LEFT},   {"BACKWARD_RIGHT", BACKWARD_RIGHT},
    };
    for (const auto& e : expected) {
        DriveDirection out = STOP;
        TEST_ASSERT_TRUE_MESSAGE(parseDriveDirection(e.name, out), e.name);
        TEST_ASSERT_EQUAL_INT_MESSAGE(e.direction, out, e.name);
    }
}

void test_table_covers_every_direction_once() {
    bool seen[9] = {};
    for (const DriveDirectionName& entry : kDriveDirectionNames) {
        TEST_ASSERT_FALSE(seen[entry.direction]);
        seen[entry.direction] = true;
    }
    for (bool s : seen) {
        TEST_ASSERT_TRUE(s);
    }
}

void test_unknown_names_are_rejected_and_leave_out_untouched() {
    const char* bad[] = {"", "forward", "FORWARD ", " FORWARD", "FORWARD_", "SPIN", "FORWARDS", "STOP\n"};
    for (const char* name : bad) {
        DriveDirection out = BACKWARD;
        TEST_ASSERT_FALSE_MESSAGE(parseDriveDirection(name, out), name);
        TEST_ASSERT_EQUAL_INT(BACKWARD, out);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_every_wire_name_parses_to_its_direction);
    RUN_TEST(test_table_covers_every_direction_once);
    RUN_TEST(test_unknown_names_are_rejected_and_leave_out_untouched);
    return UNITY_END();
}
