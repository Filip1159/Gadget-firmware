#include "HeadingFusion.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265f;

// Smallest signed difference of two angles, in (-pi, pi].
float angleDifference(float to, float from) {
    float difference = fmodf(to - from, 2.0f * kPi);
    if (difference > kPi) {
        difference -= 2.0f * kPi;
    } else if (difference <= -kPi) {
        difference += 2.0f * kPi;
    }
    return difference;
}
}  // namespace

float HeadingFusion::update(bool gyroValid, float gyroYawRad, float wheelDeltaTheta) {
    if (!gyroValid) {
        haveBaseline = false;
        gyroUsed = false;
        return wheelDeltaTheta;
    }
    const bool hadBaseline = haveBaseline;
    const float delta = angleDifference(gyroYawRad, previousYaw) * scale;
    previousYaw = gyroYawRad;
    haveBaseline = true;
    gyroUsed = hadBaseline;
    return hadBaseline ? delta : wheelDeltaTheta;
}
