# Robot firmware (Arduino Nano ESP32)

PlatformIO firmware for a 2-wheeled robot: PID-controlled drive motors, 10 VL53L0X distance sensors, WiFi, OTA updates and a WebSocket control server for the `the-sneaky-spy` frontend. Architecture, wire protocol and gotchas are in `CLAUDE.md`; the backlog is in `STATUS.MD`.

## Do not change the sensor measurement flow

The 10 VL53L0X sensors are read **one after another, with blocking `rangingTest()` calls**, on a dedicated Core-0 task (`DistanceManager::measurementTask`). A sweep takes about 390ms. That is slow, and it is deliberate: every attempt to make it faster broke on this hardware.

Tried and rolled back:

- **Ranging all 10 sensors at once** (`startRange()` on all, poll `isRangeComplete()`, read all). Several sensors failed to start (`VL53L0X_ERROR_TIME_OUT`, each stalling for ~1.3s) and the others returned the bogus value 8191, so the UI showed only the first 5-6 readings, or none.
- **Batches of 5, or of 2 neighbouring sensors.** Same failures.
- **Batches of 2 opposite-facing sensors** (for example `FRONT` + `BACK`). Looked clean at first, but readings still broke at random after about 30 seconds.
- **I2C at 400kHz** did not help.

The cause was never found (crosstalk between sensors, power, or the driver's state handling are all possible). Do not reintroduce concurrent or split ranging, and do not rewrite the flow without the robot on the bench and a soak test of several minutes.
