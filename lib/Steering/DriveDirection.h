#pragma once

#include <Arduino.h>

#include <map>

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
