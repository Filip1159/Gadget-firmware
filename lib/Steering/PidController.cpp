#include <PidController.h>

PidController::PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2)
    : pcntUnit(pcntUnit), motorPin1(motorPin1), motorPin2(motorPin2) {}

void PidController::reset() {
    integral = 0;
    previousError = 0;
}

void PidController::loop() {
    // Always consume the encoder counter, even when stopped, so pulses from coasting or
    // being pushed don't show up as a huge speed on the first tick after a new command.
    // The counter keeps running and we diff against the previous reading, so no pulse can be
    // lost between a read and a clear. It is only cleared when it nears its hardware limit
    // (+-32767), which loses at most the few pulses of that one tick.
    int16_t raw = 0;
    pcnt_get_counter_value(pcntUnit, &raw);
    int16_t count = abs(static_cast<int16_t>(raw - lastRawCount));
    lastRawCount = raw;
    if (abs(raw) > 16000) {
        pcnt_counter_clear(pcntUnit);
        lastRawCount = 0;
    }

    // Real elapsed time instead of assuming exactly 100ms between ticks.
    unsigned long now = micros();
    float dt = (now - lastLoopMicros) / 1000000.0f;
    lastLoopMicros = now;
    if (dt < 0.02f || dt > 0.5f) {
        dt = 0.1f;  // first tick after a pause, or implausible value
    }

    if (targetSpeed == 0) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
        reset();
        return;
    }

    float speed = count / dt;

    float error = targetSpeed - speed;

    float integralNext = integral + error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;

    float unclamped = Kp * error + Ki * integralNext + Kd * derivative;
    float output = constrain(unclamped, 0.0f, PWM_MAX);

    // Conditional integration anti-windup: keep integrating unless the output is saturated
    // and the error would push it further into saturation.
    bool saturatedHigh = unclamped > PWM_MAX && error > 0;
    bool saturatedLow = unclamped < 0.0f && error < 0;
    if (!saturatedHigh && !saturatedLow) {
        integral = integralNext;
    }

#ifdef DEBUG_PID
    Serial.println(
        String(output) + " | " + Kp * error + " | " + Ki * integral + " | " + Kd * derivative + " || " + error + " | " +
        speed + " | " + count
    );
#endif

    if (direction == WHEEL_FORWARD) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, output);
    } else {
        analogWrite(motorPin1, output);
        analogWrite(motorPin2, 0);
    }
}
