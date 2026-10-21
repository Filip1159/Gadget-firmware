#include <Odometry.h>

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979f;

// Wraps an angle to (-pi, pi].
float normalizeAngle(float angle) {
    angle = fmodf(angle, 2.0f * kPi);
    if (angle > kPi) {
        angle -= 2.0f * kPi;
    } else if (angle <= -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}
}  // namespace

Odometry::Odometry(float mmPerTick, float trackWidthMm) : mmPerTick(mmPerTick), trackWidthMm(trackWidthMm) {}

void Odometry::update(int32_t leftTicks, int32_t rightTicks) {
    const float leftMm = static_cast<float>(leftTicks) * mmPerTick;
    const float rightMm = static_cast<float>(rightTicks) * mmPerTick;
    const float distance = (leftMm + rightMm) / 2.0f;
    const float deltaTheta = (rightMm - leftMm) / trackWidthMm;

    const float midTheta = current.theta + deltaTheta / 2.0f;
    current.x += distance * cosf(midTheta);
    current.y += distance * sinf(midTheta);
    current.theta = normalizeAngle(current.theta + deltaTheta);
}

void Odometry::reset() {
    current = Pose();
}
