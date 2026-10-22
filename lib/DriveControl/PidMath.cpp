#include <PidMath.h>

float sanitizeDt(float dtSeconds) {
    if (dtSeconds < 0.02f || dtSeconds > 0.5f) {
        return 0.1f;
    }
    return dtSeconds;
}

float pidStep(const PidGains& gains, PidState& state, float targetSpeed, float speed, float dtSeconds) {
    const float error = targetSpeed - speed;

    const float integralNext = state.integral + error * dtSeconds;
    const float derivative = (error - state.previousError) / dtSeconds;
    state.previousError = error;

    const float unclamped = gains.kp * error + gains.ki * integralNext + gains.kd * derivative;
    const float output = unclamped < 0.0f ? 0.0f : (unclamped > PWM_MAX ? PWM_MAX : unclamped);

    const bool saturatedHigh = unclamped > PWM_MAX && error > 0;
    const bool saturatedLow = unclamped < 0.0f && error < 0;
    if (!saturatedHigh && !saturatedLow) {
        state.integral = integralNext;
    }
    return output;
}
