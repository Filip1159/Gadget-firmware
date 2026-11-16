#include "GyroYaw.h"

#include <math.h>

namespace {
constexpr double PI_D = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI_D / 180.0;
}  // namespace

void GyroYaw::update(float rateDps, uint32_t nowUs, bool stationary) {
    const bool holdWindow = stationary && isCalibrated && fabsf(rateDps - static_cast<float>(bias)) <= HOLD_DPS;
    const bool settled = stationary && settling && (nowUs - stationarySinceUs) >= SETTLE_US;

    if (havePrevious && isCalibrated) {
        const uint32_t elapsedUs = nowUs - previousUs;  // wraparound safe
        const float dt = elapsedUs * 1e-6f;
        const bool integrate = !(settled && holdWindow);
        if (integrate && dt > 0.0f && dt <= MAX_GAP_S) {
            const double meanRate = 0.5 * (previousRate + rateDps) - bias;
            yaw += meanRate * dt * DEG_TO_RAD;
            if (yaw > PI_D) {
                yaw -= 2.0 * PI_D;
            } else if (yaw <= -PI_D) {
                yaw += 2.0 * PI_D;
            }
        }
    }
    previousRate = rateDps;
    previousUs = nowUs;
    havePrevious = true;

    trackBias(rateDps, nowUs, stationary);
}

void GyroYaw::trackBias(float rateDps, uint32_t nowUs, bool stationary) {
    if (!stationary) {
        settling = false;
        windowCount = 0;
        windowSum = 0.0;
        windowSumSquares = 0.0;
        return;
    }
    if (!settling) {
        settling = true;
        stationarySinceUs = nowUs;
        windowCount = 0;
        windowSum = 0.0;
        windowSumSquares = 0.0;
        return;
    }
    if (nowUs - stationarySinceUs < SETTLE_US) {
        return;
    }
    windowSum += rateDps;
    windowSumSquares += static_cast<double>(rateDps) * rateDps;
    windowCount++;
    if (windowCount >= (isCalibrated ? TRACK_WINDOW_SAMPLES : BOOT_WINDOW_SAMPLES)) {
        closeWindow();
    }
}

void GyroYaw::closeWindow() {
    const double mean = windowSum / windowCount;
    const double variance = windowSumSquares / windowCount - mean * mean;
    const double std = variance > 0.0 ? sqrt(variance) : 0.0;
    windowCount = 0;
    windowSum = 0.0;
    windowSumSquares = 0.0;
    if (std > MAX_STD_DPS || fabs(mean) > MAX_BIAS_DPS) {
        return;  // there was motion in this window, or this is not an offset
    }
    if (!isCalibrated) {
        bias = mean;
        isCalibrated = true;
        yaw = 0.0;
    } else {
        bias += BIAS_BLEND * (mean - bias);
    }
}

float GyroYaw::yawRad() const {
    return static_cast<float>(yaw);
}
