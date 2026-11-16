#pragma once

#include <stdint.h>

// Works out how the IMU board is mounted, from two measurements: gravity while the robot stands still
// (which sensor axis points up) and the rotation axis during a LEFT (counter-clockwise) spin in place.
// Hardware-free so it can be unit tested on the host with `pio test -e native`.
//
// Right-hand rule: for a board mounted level, "up" in the sensor frame is the axis whose accelerometer
// reads about +1 g, and a counter-clockwise spin shows up as a positive rate on that same axis. If the
// board is upside down both are on the Z axis with a negative sign. The two measurements must agree,
// which also validates the spin direction and the gyro's sign convention.

struct ImuSample {
    float ax = 0.0f, ay = 0.0f, az = 0.0f;  // g
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;  // degrees per second
};

// A sensor axis with a direction: axis 0/1/2 = X/Y/Z, sign +1/-1. axis < 0 = unknown.
struct SignedAxis {
    int8_t axis = -1;
    int8_t sign = 0;

    bool valid() const {
        return axis >= 0;
    }
    // "+X", "-Z", ... ("?" if unknown); static storage.
    const char* name() const;
};

class ImuOrientationCheck {
   public:
    enum class Verdict : uint8_t {
        Running,
        Ok,
        NoData,        // too few samples (IMU not answering)
        Moving,        // the robot was not still during the first phase
        NoGravity,     // accelerometer does not read ~1 g
        Tilted,        // up (or the spin axis) is more than ~20 deg off a sensor axis
        NoSpin,        // the spin was too slow to tell the axis
        Inconsistent,  // the spin axis/direction does not match gravity
        Aborted,       // stopped before it finished (IMU_CHECK_STOP or a drive command)
    };

    struct Result {
        Verdict verdict = Verdict::Running;
        SignedAxis up;         // from gravity (set whenever gravity was readable)
        SignedAxis yaw;        // from the spin (set whenever the spin was readable)
        float tiltDeg = 0.0f;  // angle between gravity and the up axis
        float rateDps = 0.0f;  // mean spin rate on the yaw axis (absolute)
    };

    // Phase lengths. The robot must stand still for STILL_MS, then spins LEFT for SPIN_MS; the first
    // SPIN_SETTLE_MS of the spin (the PID ramping up) are ignored.
    static constexpr uint32_t STILL_MS = 1000;
    static constexpr uint32_t SPIN_SETTLE_MS = 500;
    static constexpr uint32_t SPIN_MS = 2000;

    static constexpr float GRAVITY_MIN_G = 0.8f;
    static constexpr float GRAVITY_MAX_G = 1.2f;
    static constexpr float MAX_TILT_DEG = 20.0f;
    static constexpr float STILL_MAX_RATE_DPS = 20.0f;
    static constexpr float MIN_SPIN_RATE_DPS = 30.0f;
    static constexpr uint32_t MIN_SAMPLES = 20;

    void start(uint32_t nowMs);
    // Ends a running check early; the result's verdict is then Aborted. No-op if idle.
    void abort();
    bool active() const {
        return phase != Phase::Idle;
    }
    // True while the robot should be spinning LEFT.
    bool wantSpin() const {
        return phase == Phase::Spin;
    }

    // Advances the clock and feeds one sample (nullptr if the read failed). Returns true once on the tick
    // that finishes the check; the verdict is then in result().
    bool update(uint32_t nowMs, const ImuSample* sample);

    const Result& result() const {
        return outcome;
    }

    // Stable lower-case name for the wire format and logs.
    static const char* verdictName(Verdict verdict);

   private:
    enum class Phase : uint8_t { Idle, Still, Spin };
    struct Mean {
        double sum[6] = {0, 0, 0, 0, 0, 0};
        uint32_t count = 0;
        void add(const ImuSample& s);
        float get(int i) const {
            return count ? static_cast<float>(sum[i] / count) : 0.0f;
        }
    };

    Phase phase = Phase::Idle;
    uint32_t phaseStartMs = 0;
    Mean still;
    Mean spin;
    Result outcome;

    void evaluate();
};
