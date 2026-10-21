#include "CalibrationRun.h"

#include <Protocol.h>

CalibrationRun::StartResult CalibrationRun::start(const char* args, bool robotMoving) {
    long distanceMm, count;
    if (!parseCalArgs(args, MAX_DISTANCE_MM, MAX_COUNT, distanceMm, count)) {
        return StartResult::Invalid;
    }
    if (running) {
        return StartResult::AlreadyRunning;
    }
    if (robotMoving) {
        return StartResult::RobotMoving;
    }
    running = true;
    distance = static_cast<uint16_t>(distanceMm);
    requested = static_cast<uint16_t>(count);
    completed = 0;
    return StartResult::Started;
}

bool CalibrationRun::recordSweep() {
    completed++;
    return completed >= requested;
}
