#include <CommandHandler.h>
#include <ConnectionManager.h>
#include <DistanceManager.h>
#include <SteeringManager.h>

#define STEERING_PERIOD_MS 100
#define HEARTBEAT_PERIOD_MS 1000

ConnectionManager connectionManager;
DistanceManager distanceManager;
SteeringManager steeringManager;
CommandHandler commandHandler(steeringManager, distanceManager, connectionManager);

unsigned long previousSteeringMillis = 0;
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
    distanceManager.startMeasurementTask();  // Start measurement task on Core 0
    steeringManager.init();

    // Loop watchdog: if loop() hangs for more than the task WDT timeout (5s) the board resets,
    // which releases the motor pins instead of leaving the last PWM running.
    enableLoopWDT();
}

void loop() {
    connectionManager.loop();
    commandHandler.loop();

    unsigned long currentMillis = millis();

    if (currentMillis - previousSteeringMillis >= STEERING_PERIOD_MS) {
        previousSteeringMillis = currentMillis;
        steeringManager.loop();
    }

    if (currentMillis - previousHeartbeatMillis >= HEARTBEAT_PERIOD_MS) {
        previousHeartbeatMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_GREEN, ledState);
    }
}
