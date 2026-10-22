#include <DriveMixer.h>

WheelTargets mixDrive(DriveDirection direction, uint16_t speed) {
    const uint16_t inner = static_cast<uint16_t>(speed * TURNING_FACTOR);
    switch (direction) {
        case FORWARD:
            return {WHEEL_FORWARD, speed, WHEEL_FORWARD, speed};
        case FORWARD_LEFT:
            return {WHEEL_FORWARD, inner, WHEEL_FORWARD, speed};
        case FORWARD_RIGHT:
            return {WHEEL_FORWARD, speed, WHEEL_FORWARD, inner};
        case LEFT:
            return {WHEEL_BACKWARD, speed, WHEEL_FORWARD, speed};
        case RIGHT:
            return {WHEEL_FORWARD, speed, WHEEL_BACKWARD, speed};
        case BACKWARD:
            return {WHEEL_BACKWARD, speed, WHEEL_BACKWARD, speed};
        case BACKWARD_LEFT:
            return {WHEEL_BACKWARD, inner, WHEEL_BACKWARD, speed};
        case BACKWARD_RIGHT:
            return {WHEEL_BACKWARD, speed, WHEEL_BACKWARD, inner};
        case STOP:
            break;
    }
    return {WHEEL_FORWARD, 0, WHEEL_FORWARD, 0};
}
