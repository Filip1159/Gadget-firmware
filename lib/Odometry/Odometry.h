#pragma once

#include <stdint.h>

// Differential-drive dead reckoning. Hardware-free (no Arduino/ESP-IDF includes) so it can be unit
// tested on the host with `pio test -e native`.
//
// Frame (ROS REP-103): millimetres, +x forward, +y left, theta counter-clockwise in radians,
// normalised to (-pi, pi]. Ticks are signed and positive when the wheel moves the robot forward.

struct Pose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

class Odometry {
   public:
    Odometry(float mmPerTick, float trackWidthMm);

    // Integrates the signed tick deltas of one control tick (arc / midpoint model: the heading used
    // for the translation is the mean of the heading before and after the tick).
    void update(int32_t leftTicks, int32_t rightTicks);

    // Back to (0, 0, 0).
    void reset();

    const Pose& pose() const {
        return current;
    }

   private:
    float mmPerTick;
    float trackWidthMm;
    Pose current;
};
