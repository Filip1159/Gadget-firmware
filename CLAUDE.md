# Robot Firmware (Arduino Nano ESP32)

PlatformIO firmware for a 2-wheeled robot built on an **Arduino Nano ESP32** (ESP32-S3, dual-core). It drives two DC gear motors with encoders via PID, reads 10x VL53L0X time-of-flight distance sensors, and exposes both over a WebSocket server to the companion frontend app (`the-sneaky-spy`, a separate repo — see "Companion frontend" below).

## Build / upload

- Build system: PlatformIO (`platformio.ini`), environment `arduino_nano_esp32`.
- Framework: Arduino, platform `espressif32`.
- Library deps: `adafruit/Adafruit_VL53L0X@1.2.4`, `links2004/WebSockets@2.7.3`.
- Upload: currently via serial (`upload_port = COM14`, 115200 baud). OTA upload is wired up in firmware (`ArduinoOTA`) but `upload_protocol = espota` is commented out in `platformio.ini` — uncomment and set `upload_port` to the robot's IP to flash over WiFi instead of USB.
- Serial monitor: 115200 baud, prints connection status, OTA progress, and PID debug (`P`/`I`/`D` values on `PID` command).

## Architecture

Two FreeRTOS cores are used deliberately to avoid blocking the motor control loop:

- **Core 1** (default Arduino `loop()`): WebSocket I/O (`ConnectionManager::loop()`), OTA handling, PID steering loop (`SteeringManager::loop()`, driven at a 100ms / 10Hz cadence), and polling for/broadcasting finished distance measurements.
- **Core 0**: a dedicated FreeRTOS task (`DistanceManager::measurementTask`) that blockingly reads all 10 VL53L0X sensors (~330ms for all 10) whenever a measurement is requested. This exists because the VL53L0X `rangingTest()` I2C call is blocking and previously starved the 100ms PID loop, causing visible speed drops during measurement (see `STATUS.MD` for the full history of this bug and fix).

Cross-core communication uses a shared `measurementMutex` guarding `measurementRequested`/`measurementReady`/`latestMeasurements[10]` — not a queue, just a flag-based single-slot handoff (see Known gotchas).

### Modules (`lib/`)

- **ConnectionManager** (`lib/ConnectionManager/`) — WiFi station connect (`initWiFi`), ArduinoOTA setup (`initOta`), and a `WebSocketsServer` on port 88 (`initWebsocket`). Incoming text frames are forwarded verbatim as a `String` to a single global callback (`updateSteering` in `main.cpp`). `send()` is a direct synchronous `broadcastTXT` to all connected clients — there is no outgoing queue or backpressure handling.
  - WiFi SSID/password are hardcoded as plaintext constants in `ConnectionManager.h` — treat this file as sensitive, don't paste its contents elsewhere.
- **DistanceManager** (`lib/DistanceManager/`) — owns 10 `DistanceSensor` (VL53L0X) instances, each with a unique I2C address (`0x30`-`0x39`) assigned at boot via XSHUT pins (`initAddresses`), since all sensors share the same default I2C address out of the box. Exposes a non-blocking request/poll API (`requestMeasurement`, `isMeasurementReady`, `getLatestMeasurements`, `clearMeasurementReady`) backed by the Core-0 task described above. Sensor order/direction mapping: `FRONT, FRONT_LEFT_45, FRONT_LEFT, FRONT_RIGHT_45, FRONT_RIGHT, BACK, BACK_LEFT_45, BACK_LEFT, BACK_RIGHT_45, BACK_RIGHT` (index 0-9, see `sensorsOrder` / `SensorDirection` enum). `directionToString()` converts these to the exact strings sent over the wire.
- **Steering** (`lib/Steering/`) — `SteeringManager` maps a `DriveDirection` enum (`FORWARD`, `FORWARD_LEFT`, `FORWARD_RIGHT`, `LEFT`, `STOP`, `RIGHT`, `BACKWARD`, `BACKWARD_LEFT`, `BACKWARD_RIGHT`) + a target `speed` to independent per-wheel target speeds (turns scale the inner wheel by `TURNING_FACTOR = 0.7`), then runs a `PidController` per wheel (`Kp=0.15, Ki=0.05, Kd=0.0` defaults) closing the loop against quadrature encoder pulse counts read via the ESP-IDF `driver/pcnt` (pulse counter) peripheral. `STOP` resets both PIDs (clears integral windup) in addition to zeroing target speed.

## Communication schema (WebSocket, port 88, plain text frames)

No JSON — every message is a plain space-delimited ASCII string. The server is the robot; the frontend is the client.

### Frontend → Robot (commands), handled in `updateSteering()` (`src/main.cpp`)

| Message | Effect |
|---|---|
| `SPEED <uint16>` | Sets `SteeringManager` target speed (applies to all subsequent direction commands) |
| `PID <P> <I> <D>` | Sets PID gains (floats) for both wheel controllers, resets integral term |
| `MEASURE_START` | Enables continuous measurement mode (robot auto-requests a new 10-sensor sweep every 200ms) |
| `MEASURE_STOP` | Disables continuous measurement mode |
| `MEASURE` | Requests exactly one measurement sweep (non-blocking; result arrives async via `MEASURE ...` broadcasts) |
| Any other string | Treated as a `DriveDirection` name (`FORWARD`, `FORWARD_LEFT`, `FORWARD_RIGHT`, `LEFT`, `STOP`, `RIGHT`, `BACKWARD`, `BACKWARD_LEFT`, `BACKWARD_RIGHT`) — unrecognized values throw (`std::map::at`), crashing `updateSteering`. Also resets PIDs. |

### Robot → Frontend (broadcast, unsolicited)

- `MEASURE <DIRECTION>=<mm|OOR> ...` � a single frame per completed sweep, e.g. `MEASURE FRONT=412 FRONT_LEFT_45=OOR ...`, containing all 10 sensors in `sensorsOrder`. `OOR` means out of range / no obstacle (also reported for sensors that are offline). `DIRECTION` is one of the 10 `SensorDirection` strings above (see `directionToString`). The frontend must treat the frame as one atomic sweep.
- Nothing is sent for `SPEED`/`PID`/direction commands — they are fire-and-forget from the frontend's perspective.

## Known gotchas / fragile areas

- **`measurementReady` must be cleared after each consumption.** It's a single-slot flag, not a queue. `requestMeasurement()` sets it `false`; the Core-0 task sets it `true` when a sweep finishes; the consumer (`loop()` in `main.cpp`) must call `clearMeasurementReady()` after broadcasting, or it stays `true` forever — the 200ms continuous-mode re-trigger (`if (!isMeasurementReady()) requestMeasurement()`) then never fires again, and the unthrottled send-when-ready check re-broadcasts the same stale values every single loop iteration. (This exact bug was hit and fixed — see `STATUS.MD` and git history.)
- **Unrecognized `DriveDirection` strings crash `updateSteering`** via `std::map::at` throwing on a missing key — there's no validation before `directionStrings.at(direction)` in `SteeringManager::setDirection`.
- **No OTA auth/encryption configured** (`ArduinoOTA.begin()` with no password) — fine on a trusted home network, not safe to expose beyond that.
- **WebSocket has no auth** — any client on the network can connect to port 88 and drive the robot or spam measurement commands.
- Sensor measurement sweep takes ~330ms for all 10 sensors; continuous mode requests a new sweep every 200ms, so the achievable update rate is closer to ~2-3Hz, not 5Hz.

## Companion frontend

The browser control UI lives in a separate repo: `C:\Users\fwisn\Documents\the-sneaky-spy` (see its own `CLAUDE.md`). It connects directly to `ws://<robot-ip>:88` and implements the other half of the schema above.
