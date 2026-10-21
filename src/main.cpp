#include <CommandHandler.h>
#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <SteeringManager.h>

#define HEARTBEAT_PERIOD_MS 1000
#define CONTROL_TASK_STALL_MS 2000

ConnectionManager connectionManager;
DistanceManager distanceManager;
SteeringManager steeringManager;
CommandHandler commandHandler(steeringManager, distanceManager, connectionManager);

unsigned long previousHeartbeatMillis = 0;
bool ledState = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Booting");

    pinMode(LED_GREEN, OUTPUT);

    connectionManager.initWiFi();
    connectionManager.setOnDisconnect([]() { commandHandler.onClientDisconnected(); });
    connectionManager.setOnOtaStart([]() { steeringManager.stop(); });
    connectionManager.setOnClientReady([](uint8_t num) { commandHandler.sendState(num); });
    connectionManager.initOta();
    connectionManager.initWebsocket([](const String& message) { commandHandler.handle(message); });
    distanceManager.initAddresses();
    distanceManager.setPoseProvider([]() { return steeringManager.getStampedPose(); });
    distanceManager.startMeasurementTask();  // Start measurement task on Core 0
    steeringManager.init();
    steeringManager.startTask();  // PID + command timeout run in their own task (Core 1)

    // Loop watchdog: if loop() hangs for more than the task WDT timeout (5s) the board resets,
    // which releases the motor pins instead of leaving the last PWM running.
    enableLoopWDT();
}

void loop() {
    connectionManager.loop();
    commandHandler.loop();

    unsigned long currentMillis = millis();

    // If the control task is stuck nothing can stop the motors by software: reset the board,
    // which releases the motor pins.
    if (steeringManager.msSinceLastTick() > CONTROL_TASK_STALL_MS) {
        Serial.println("Control task stalled, restarting");
        ESP.restart();
    }

    if (currentMillis - previousHeartbeatMillis >= HEARTBEAT_PERIOD_MS) {
        previousHeartbeatMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_GREEN, ledState);
    }
}
