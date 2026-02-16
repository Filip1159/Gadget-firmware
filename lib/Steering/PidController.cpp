#include <PidController.h>

PidController::PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2)
    : pcntUnit(pcntUnit), motorPin1(motorPin1), motorPin2(motorPin2) {}

void PidController::reset() {
    integral = 0;
    previousError = 0;
}

void PidController::loop() {
    if (targetSpeed == 0) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, 0);
        return;
    }

    int16_t count = 0;
    pcnt_get_counter_value(pcntUnit, &count);
    pcnt_counter_clear(pcntUnit);
    count = abs(count);

    float speed = count / 0.1;  // assume loop running every 100ms

    float error = targetSpeed - speed;

    integral += error * 0.1;
    float derivative = (error - previousError) / 0.1;
    previousError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    if (output == 255 && error > 0) {
        integral -= error * 0.1;
    } else if (output == 0 && error < 0) {
        integral -= error * 0.1;
    }

    if (output > 255)
        output = 255;
    if (output < 0)
        output = 0;

    Serial.println(
        String(output) + " | " + Kp * error + " | " + Ki * integral + " | " + Kd * derivative + " || " + error + " | " +
        speed + " | " + count
    );

    if (direction == WHEEL_FORWARD) {
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, output);
    } else {
        analogWrite(motorPin1, output);
        analogWrite(motorPin2, 0);
    }
}
