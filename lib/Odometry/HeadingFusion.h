#pragma once

// Chooses the heading change of one odometry tick (MAPPING.MD 6.3). Hardware-free, host-tested.
//
// The gyro's yaw drifts far less than the wheels' (slip, uneven wheels, the track width), so while it is
// valid the heading change is the change of the gyro yaw since the previous tick; position still comes
// from the encoders. Odometry has no absolute heading to correct the gyro with, so blending it in would
// only bring its drift back: the wheels are the fallback, not a second opinion. (Disagreement between the
// two is a wheel-slip signal, MAPPING.MD 1.5.)

// Multiplies the gyro yaw change. 1.0 until a measured 360 deg spin says otherwise (BMI160 scale error
// is typically around 1%): scale = 360 / (what the robot reported for a true lap).
constexpr float GYRO_YAW_SCALE = 1.0f;

class HeadingFusion {
   public:
    explicit HeadingFusion(float gyroScale = GYRO_YAW_SCALE) : scale(gyroScale) {}

    // `gyroYawRad` is the gyro's absolute yaw (wraps at +-pi) and only meaningful if `gyroValid`.
    // `wheelDeltaTheta` is the wheels' heading change this tick. Returns the heading change to use.
    // The first valid tick, and the first one after the gyro was invalid, only sets the baseline and uses
    // the wheels.
    float update(bool gyroValid, float gyroYawRad, float wheelDeltaTheta);

    // Whether the last update() used the gyro.
    bool usingGyro() const {
        return gyroUsed;
    }

   private:
    float scale;
    bool haveBaseline = false;
    float previousYaw = 0.0f;
    bool gyroUsed = false;
};
