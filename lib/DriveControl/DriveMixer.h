#pragma once

#include <DriveDirection.h>
#include <stdint.h>

// Inner-wheel speed scale when turning while driving.
constexpr float TURNING_FACTOR = 0.7f;

struct WheelTargets {
    WheelDirection leftDir;
    uint16_t leftSpeed;
    WheelDirection rightDir;
    uint16_t rightSpeed;
};

// Hardware-free: maps a drive direction and the target speed to what each wheel PID should aim for.
// Turns while driving scale the inner wheel by TURNING_FACTOR (truncated to a whole speed);
// LEFT/RIGHT spin in place; STOP is speed 0 on both wheels.
WheelTargets mixDrive(DriveDirection direction, uint16_t speed);
