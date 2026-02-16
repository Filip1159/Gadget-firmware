#include <SteeringManager.h>

#define TURNING_FACTOR 0.7

SteeringManager::SteeringManager() {
    left = new PidController(PCNT_UNIT_0, LEFT_2_PIN, LEFT_1_PIN);
    right = new PidController(PCNT_UNIT_1, RIGHT_2_PIN, RIGHT_1_PIN);
}

void SteeringManager::init() {
    pinMode(LEFT_1_PIN, OUTPUT);
    pinMode(LEFT_2_PIN, OUTPUT);
    pinMode(RIGHT_1_PIN, OUTPUT);
    pinMode(RIGHT_2_PIN, OUTPUT);

    pcnt_config_t pcnt_config_left = {};
    pcnt_config_left.pulse_gpio_num = LEFT_ENCODER_A;
    pcnt_config_left.ctrl_gpio_num = LEFT_ENCODER_B;
    pcnt_config_left.channel = PCNT_CHANNEL_0;
    pcnt_config_left.unit = PCNT_UNIT_0;
    pcnt_config_left.pos_mode = PCNT_COUNT_INC;
    pcnt_config_left.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_left.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_left.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_left.counter_h_lim = 32767;
    pcnt_config_left.counter_l_lim = -32768;

    pcnt_config_t pcnt_config_right = {};
    pcnt_config_right.pulse_gpio_num = RIGHT_ENCODER_A;
    pcnt_config_right.ctrl_gpio_num = RIGHT_ENCODER_B;
    pcnt_config_right.channel = PCNT_CHANNEL_0;
    pcnt_config_right.unit = PCNT_UNIT_1;
    pcnt_config_right.pos_mode = PCNT_COUNT_INC;
    pcnt_config_right.neg_mode = PCNT_COUNT_DEC;
    pcnt_config_right.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_right.hctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_right.counter_h_lim = 32767;
    pcnt_config_right.counter_l_lim = -32768;

    pcnt_unit_config(&pcnt_config_left);
    pcnt_unit_config(&pcnt_config_right);

    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_set_filter_value(PCNT_UNIT_1, 100);
    pcnt_filter_enable(PCNT_UNIT_1);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

void SteeringManager::resetPids() {
    left->reset();
    right->reset();
}

void SteeringManager::setPid(float P, float I, float D) {
    left->setPid(P, I, D);
    right->setPid(P, I, D);
}

void SteeringManager::loop() {
    switch (direction) {
        case FORWARD:
            left->setTargetSpeed(WHEEL_FORWARD, speed);
            right->setTargetSpeed(WHEEL_FORWARD, speed);
            break;
        case FORWARD_LEFT:
            left->setTargetSpeed(WHEEL_FORWARD, speed * TURNING_FACTOR);
            right->setTargetSpeed(WHEEL_FORWARD, speed);
            break;
        case FORWARD_RIGHT:
            left->setTargetSpeed(WHEEL_FORWARD, speed);
            right->setTargetSpeed(WHEEL_FORWARD, speed * TURNING_FACTOR);
            break;
        case LEFT:
            left->setTargetSpeed(WHEEL_BACKWARD, speed);
            right->setTargetSpeed(WHEEL_FORWARD, speed);
            break;
        case STOP:
            left->setTargetSpeed(WHEEL_FORWARD, 0);
            right->setTargetSpeed(WHEEL_FORWARD, 0);
            left->reset();
            right->reset();
            break;
        case RIGHT:
            left->setTargetSpeed(WHEEL_FORWARD, speed);
            right->setTargetSpeed(WHEEL_BACKWARD, speed);
            break;
        case BACKWARD:
            left->setTargetSpeed(WHEEL_BACKWARD, speed);
            right->setTargetSpeed(WHEEL_BACKWARD, speed);
            break;
        case BACKWARD_LEFT:
            left->setTargetSpeed(WHEEL_BACKWARD, speed * TURNING_FACTOR);
            right->setTargetSpeed(WHEEL_BACKWARD, speed);
            break;
        case BACKWARD_RIGHT:
            left->setTargetSpeed(WHEEL_BACKWARD, speed);
            right->setTargetSpeed(WHEEL_BACKWARD, speed * TURNING_FACTOR);
            break;
    }
    left->loop();
    right->loop();
}
