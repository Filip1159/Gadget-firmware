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

// A pose and the robot millis() of the control tick it was computed in.
struct StampedPose {
    Pose pose;
    uint32_t tMs = 0;
};

class Odometry {
   public:
    Odometry(float mmPerTick, float trackWidthMm);

    // Integrates the signed tick deltas of one control tick (arc / midpoint model: the heading used
    // for the translation is the mean of the heading before and after the tick).
    void update(int32_t leftTicks, int32_t rightTicks);

    // Heading change (radians, counter-clockwise) the wheels alone imply for these tick deltas.
    float wheelHeadingChange(int32_t leftTicks, int32_t rightTicks) const;

    // Same, but the heading change comes from elsewhere (the gyro, see HeadingFusion): the distance still
    // comes from the wheels and is laid along the mean heading of the tick.
    void update(int32_t leftTicks, int32_t rightTicks, float headingChange);

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
