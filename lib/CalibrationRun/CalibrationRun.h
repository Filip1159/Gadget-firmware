#pragma once

#include <stdint.h>

// State of a sensor characterisation run (`CAL <distance_mm> <count>`, MAPPING.MD 0.4): the true wall
// distance, how many sweeps were asked for and how many are done. Hardware-free, host-tested.
// The caller does the I/O (requesting sweeps, sending frames) and asks this class what is allowed.
class CalibrationRun {
   public:
    static constexpr long MAX_DISTANCE_MM = 2000;
    static constexpr long MAX_COUNT = 500;

    enum class StartResult { Started, Invalid, AlreadyRunning, RobotMoving };

    // Checks, in this order: arguments ("<distance_mm> <count>"), a run already active, robot moving.
    StartResult start(const char* args, bool robotMoving);

    bool active() const {
        return running;
    }
    uint16_t distanceMm() const {
        return distance;
    }
    uint16_t total() const {
        return requested;
    }
    uint16_t done() const {
        return completed;
    }

    // Counts one finished sweep (its 1-based index is then done()). True when that was the last one.
    bool recordSweep();

    // Ends the run: after the last sweep, on CAL_STOP, or when the client is gone. Keeps the numbers so the
    // final CAL_DONE can still report them.
    void end() {
        running = false;
    }

   private:
    bool running = false;
    uint16_t distance = 0;
    uint16_t requested = 0;
    uint16_t completed = 0;
};
