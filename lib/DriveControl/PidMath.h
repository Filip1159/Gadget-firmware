#pragma once

// Maximum analogWrite duty (8-bit PWM).
constexpr float PWM_MAX = 255.0f;

struct PidGains {
    float kp;
    float ki;
    float kd;
};

struct PidState {
    float integral = 0;
    float previousError = 0;
    void reset() {
        integral = 0;
        previousError = 0;
    }
};

// Real elapsed time between control ticks, in seconds. The first tick after a pause or an implausible
// value (outside 0.02..0.5 s) falls back to the nominal 0.1 s period.
float sanitizeDt(float dtSeconds);

// One PID step, hardware-free. `speed` is the measured speed (ticks per second), `targetSpeed` the setpoint.
// Returns the PWM duty in 0..PWM_MAX and advances `state`. The integral uses conditional-integration
// anti-windup: it keeps accumulating unless the output is saturated and the error pushes it further in.
float pidStep(const PidGains& gains, PidState& state, float targetSpeed, float speed, float dtSeconds);
