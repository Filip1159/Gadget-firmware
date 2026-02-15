#include <PidController.h>

PidController::PidController(pcnt_unit_t pcntUnit, uint8_t motorPin1, uint8_t motorPin2)
    : pcntUnit(pcntUnit), motorPin1(motorPin1), motorPin2(motorPin2) {}

void PidController::loop() {
    int16_t count = 0;
    pcnt_get_counter_value(pcntUnit, &count);

    uint16_t overflowSafeCount = (uint16_t)count;
    if (count < lastEncoderValue) {
        overflowSafeCount += 32768;
    }

    uint16_t diff = overflowSafeCount - lastEncoderValue;
    lastEncoderValue = (uint16_t)count;

    float speed = diff / 0.1;  // assume loop running every 100ms

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
        speed + " | " + count + " | " + overflowSafeCount
    );

    analogWrite(motorPin1, output);
    digitalWrite(motorPin2, LOW);
}
