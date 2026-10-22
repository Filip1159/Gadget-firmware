#include <SteeringManager.h>

// Glitch filter on the encoder inputs, in APB clock cycles.
constexpr uint16_t PCNT_FILTER_CYCLES = 100;
constexpr uint32_t CONTROL_TASK_STACK_BYTES = 4096;

SteeringManager::SteeringManager()
    : left(PCNT_UNIT_0, LEFT_2_PIN, LEFT_1_PIN, LEFT_ENCODER_SIGN),
      right(PCNT_UNIT_1, RIGHT_2_PIN, RIGHT_1_PIN, RIGHT_ENCODER_SIGN) {
    mutex = xSemaphoreCreateMutex();
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

    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_CYCLES);
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_CYCLES);
    pcnt_filter_enable(PCNT_UNIT_1);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

namespace {
// Scoped lock for the steering mutex.
struct Lock {
    SemaphoreHandle_t m;
    explicit Lock(SemaphoreHandle_t m) : m(m) {
        xSemaphoreTake(m, portMAX_DELAY);
    }
    ~Lock() {
        xSemaphoreGive(m);
    }
};
}  // namespace

void SteeringManager::resetPids() {
    Lock lock(mutex);
    left.reset();
    right.reset();
}

void SteeringManager::setSpeed(uint16_t newSpeed) {
    Lock lock(mutex);
    speed = newSpeed;
}

uint16_t SteeringManager::getSpeed() {
    Lock lock(mutex);
    return speed;
}

void SteeringManager::setPid(float P, float I, float D) {
    Lock lock(mutex);
    left.setPid(P, I, D);
    right.setPid(P, I, D);
}

void SteeringManager::getPid(float& P, float& I, float& D) {
    Lock lock(mutex);
    P = left.getKp();
    I = left.getKi();
    D = left.getKd();
}

bool SteeringManager::isMoving() {
    Lock lock(mutex);
    return direction != STOP;
}

bool SteeringManager::setDirection(const String& name) {
    Lock lock(mutex);
    DriveDirection requested;
    if (!parseDriveDirection(name.c_str(), requested)) {
        return false;
    }
    if (requested != direction) {
        left.reset();  // only on an actual direction change; repeats must not wipe the integral
        right.reset();
    }
    direction = requested;
    commandTimeout.refresh(millis());
    return true;
}

void SteeringManager::stop() {
    Lock lock(mutex);
    direction = STOP;
    tickLocked();
}

void SteeringManager::loop() {
    Lock lock(mutex);
    tickLocked();
}

void SteeringManager::startTask() {
    lastTickMillis = millis();
    xTaskCreatePinnedToCore(
        controlTask,    // Task function
        "ControlTask",  // Task name
        CONTROL_TASK_STACK_BYTES,  // Stack size
        this,           // Parameter passed to task
        2,              // Priority: above loop() (1), so network I/O cannot delay a tick
        NULL,           // Task handle (not needed)
        1               // Core 1, away from the sensor task on Core 0
    );
}

void SteeringManager::controlTask(void* parameter) {
    SteeringManager* manager = static_cast<SteeringManager*>(parameter);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STEERING_PERIOD_MS));
        manager->loop();
    }
}

void SteeringManager::tickLocked() {
    // Command timeout failsafe: runs here so a stalled network loop cannot delay it.
    if (commandTimeout.expired(millis(), direction != STOP)) {
        Serial.println("Command timeout, stopping");
        direction = STOP;
    }
    lastTickMillis = millis();

    const WheelTargets targets = mixDrive(direction, speed);
    left.setTargetSpeed(targets.leftDir, targets.leftSpeed);
    right.setTargetSpeed(targets.rightDir, targets.rightSpeed);
    if (direction == STOP) {
        left.reset();
        right.reset();
    }
    left.loop();
    right.loop();
    // The PID loops consume the encoder counters every tick (also while stopped, so coasting counts).
    odometry.update(left.getSignedDelta(), right.getSignedDelta());
#ifdef DEBUG_POSE
    static uint8_t poseTick = 0;
    if (++poseTick % 5 == 0) {
        const Pose& pose = odometry.pose();
        Serial.println("POSE x=" + String(pose.x, 1) + " y=" + String(pose.y, 1) + " th=" + String(pose.theta, 3));
    }
#endif
}

Pose SteeringManager::getPose() {
    Lock lock(mutex);
    return odometry.pose();
}

StampedPose SteeringManager::getStampedPose() {
    Lock lock(mutex);
    StampedPose stamped;
    stamped.pose = odometry.pose();
    stamped.tMs = static_cast<uint32_t>(lastTickMillis);
    return stamped;
}

void SteeringManager::resetPose() {
    Lock lock(mutex);
    odometry.reset();
}
