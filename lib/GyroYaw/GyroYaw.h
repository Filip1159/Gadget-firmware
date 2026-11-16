#pragma once

#include <stdint.h>

// Yaw angle from a gyro's z rate (MAPPING.MD 6.2). Hardware-free so it can be unit tested on the host
// with `pio test -e native`.
//
// - Bias: a gyro reads a small constant rate even when still, and that drifts with temperature. While
//   the caller says the robot is stationary (and has been for SETTLE_US), the rate is averaged over a
//   window; if the window is quiet (std below MAX_STD_DPS) it becomes the bias estimate: the first
//   window sets it outright (that is the boot calibration, ~2.5 s), later ones nudge it
//   (BIAS_BLEND), which tracks the temperature drift. A window with motion in it is discarded.
// - Integration: trapezoid over the real time between samples, minus the bias. Nothing is integrated
//   until the first calibration. While stationary and settled the yaw is held, so a still robot does
//   not wander by the noise left in the bias; a real turn (rate beyond HOLD_DPS) is still integrated.
//
// Rates are counter-clockwise positive degrees per second, the yaw is radians in (-pi, pi], 0 at the
// first calibration or resetYaw().
class GyroYaw {
   public:
    static constexpr uint32_t SETTLE_US = 500000;       // stationary this long before a window starts
    static constexpr uint32_t BOOT_WINDOW_SAMPLES = 200;  // first window: ~2 s at 100 Hz
    static constexpr uint32_t TRACK_WINDOW_SAMPLES = 100;  // later windows: ~1 s
    static constexpr float MAX_STD_DPS = 0.5f;          // a window noisier than this had motion in it
    static constexpr float MAX_BIAS_DPS = 20.0f;        // |bias| beyond this is not a gyro offset
    static constexpr float BIAS_BLEND = 0.2f;           // weight of a new window in the running bias
    static constexpr float HOLD_DPS = 2.0f;             // stationary: hold yaw unless |rate - bias| exceeds this
    static constexpr float MAX_GAP_S = 0.2f;            // a longer gap between samples is not integrated

    // One gyro sample. `nowUs` is a free-running microsecond clock (wraps at 2^32), `stationary` says the
    // robot is not being driven.
    void update(float rateDps, uint32_t nowUs, bool stationary);

    bool calibrated() const {
        return isCalibrated;
    }
    float yawRad() const;  // wrapped to (-pi, pi]; 0 until calibrated
    float biasDps() const {
        return static_cast<float>(bias);
    }
    // How many gaps between samples were too long to integrate (each one is lost rotation).
    uint32_t skippedGaps() const {
        return gapCount;
    }
    // Zeroes the yaw (keeps the bias).
    void resetYaw() {
        yaw = 0.0;
    }

   private:
    bool isCalibrated = false;
    double bias = 0.0;  // deg/s
    double yaw = 0.0;   // rad, wrapped
    uint32_t gapCount = 0;

    bool havePrevious = false;
    float previousRate = 0.0f;
    uint32_t previousUs = 0;

    bool settling = false;  // stationary right now
    uint32_t stationarySinceUs = 0;
    uint32_t windowCount = 0;
    double windowSum = 0.0;
    double windowSumSquares = 0.0;

    void trackBias(float rateDps, uint32_t nowUs, bool stationary);
    void closeWindow();
};
