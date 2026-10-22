#include <PidController.h>

PidController::PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2, int8_t encoderSign)
    : pcntUnit(pcntUnit), motorPin1(motorPin1), motorPin2(motorPin2), encoderSign(encoderSign) {}

void PidController::reset() {
    state.reset();
}

void PidController::loop() {
    // Always consume the encoder counter, even when stopped, so pulses from coasting or
    // being pushed don't show up as a huge speed on the first tick after a new command.
    // The counter keeps running and we diff against the previous reading, so no pulse can be
    // lost between a read and a clear. It is only cleared when it nears its hardware limit
    // (+-32767), which loses at most the few pulses of that one tick.
    int16_t raw = 0;
    pcnt_get_counter_value(pcntUnit, &raw);
    const EncoderTracker::Reading reading = encoder.update(raw);
    const int16_t signedDelta = reading.delta;
    lastSignedDelta = static_cast<int16_t>(signedDelta * encoderSign);
    int16_t count = abs(signedDelta);
    if (reading.clearCounter) {
        pcnt_counter_clear(pcntUnit);
    }

#ifdef DEBUG_TICKS
    if (signedDelta != 0) {
        Serial.printf("TICKS unit=%d delta=%d\n", static_cast<int>(pcntUnit), lastSignedDelta);
    }
#endif

    // Real elapsed time instead of assuming exactly 100ms between ticks.
    unsigned long now = micros();
    const float dt = sanitizeDt((now - lastLoopMicros) / 1000000.0f);
    lastLoopMicros = now;

    if (targetSpeed == 0) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
        reset();
        return;
    }

    const float speed = count / dt;
    const float output = pidStep(gains, state, targetSpeed, speed, dt);

#ifdef DEBUG_PID
    Serial.println(String(output) + " | " + (targetSpeed - speed) + " | " + speed + " | " + count);
#endif

    if (direction == WHEEL_FORWARD) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, output);
    } else {
        analogWrite(motorPin1, output);
        analogWrite(motorPin2, 0);
    }
}
