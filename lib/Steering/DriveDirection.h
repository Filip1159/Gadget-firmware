#pragma once

#include <Arduino.h>

enum DriveDirection {
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    LEFT,
    STOP,
    RIGHT,
    BACKWARD,
    BACKWARD_LEFT,
    BACKWARD_RIGHT
};

enum WheelDirection { WHEEL_FORWARD, WHEEL_BACKWARD };

// Wire names for DriveDirection (must match the frontend's protocol.ts).
struct DriveDirectionName {
    const char* name;
    DriveDirection direction;
};

static const DriveDirectionName kDriveDirectionNames[] = {
    { "FORWARD", FORWARD },
    { "FORWARD_LEFT", FORWARD_LEFT },
    { "FORWARD_RIGHT", FORWARD_RIGHT },
    { "LEFT", LEFT },
    { "STOP", STOP },
    { "RIGHT", RIGHT },
    { "BACKWARD", BACKWARD },
    { "BACKWARD_LEFT", BACKWARD_LEFT },
    { "BACKWARD_RIGHT", BACKWARD_RIGHT },
};
