# Gadget-firmware (Arduino Nano ESP32)

PlatformIO firmware for a 2-wheeled robot built on an **Arduino Nano ESP32** (ESP32-S3, dual-core). It drives two DC gear motors with encoders via PID, reads 10x VL53L0X time-of-flight distance sensors, and exposes both over a WebSocket server to the companion frontend app (Gadget-UI, repo folder `Gadget-UI`, a separate repo — see "Companion frontend" below).

## Build / upload

- Build system: PlatformIO (`platformio.ini`), environment `arduino_nano_esp32`.
- Framework: Arduino, platform `espressif32` (not version-pinned yet). `build_flags` has `-Wall -Wextra`; `-DDEBUG_PID` (commented out) enables per-tick PID output on serial.
- Library deps: `adafruit/Adafruit_VL53L0X@1.2.4`, `links2004/WebSockets@2.7.3`.
- Upload: currently over USB (`upload_port = COM14`, DFU protocol, so no upload baud rate). OTA upload is wired up in firmware (`ArduinoOTA`); flash over WiFi with `pio run -e ota -t upload` (env `ota` extends the USB one, port `gadget.local`; override with `--upload-port <ip>`).
- Lint: `pio check` (cppcheck, our code only; keep it at zero medium/high findings). Build with `-Wall -Wextra`.
- Host tests: `pio test -e native` (Unity, env `native`) runs the hardware-free modules (currently `lib/Odometry` and `lib/SensorCalibration`, tests in `test/`). Needs a host `g++` on PATH (on Windows e.g. the MinGW bundled with CLion). Keep new pose/map math Arduino-free so it can be tested here; the `native` env ignores the hardware libs via `lib_ignore`.
- Serial monitor: 115200 baud, prints connection status, OTA progress, and PID debug (`P`/`I`/`D` values on `PID` command).

## Architecture

Two FreeRTOS cores are used deliberately to avoid blocking the motor control loop:

- **Core 1** (default Arduino `loop()`): WebSocket I/O + OTA (`ConnectionManager::loop()`), and `CommandHandler::loop()` (continuous-measurement scheduling, sweep broadcast). A separate **control task** (`SteeringManager::controlTask`, also Core 1, priority 2 so it preempts `loop()`) runs the PID every `STEERING_PERIOD_MS` (100ms, `vTaskDelayUntil`) and enforces the command timeout, so a stalled WebSocket write cannot delay either; `loop()` resets the board if that task has not ticked for 2s. The loop task is under the Arduino loop watchdog (`enableLoopWDT()`, 5s, resets the board; `ArduinoOTA` feeds it during uploads).
- **Core 0**: a dedicated FreeRTOS task (`DistanceManager::measurementTask`) that blockingly reads the 10 VL53L0X sensors one after another (~390ms per sweep) whenever a measurement is requested. This exists because the VL53L0X `rangingTest()` I2C call is blocking and previously starved the 100ms PID loop, causing visible speed drops during measurement (see `SENSOR_FLOW.md` for the sensor flow history).

Cross-core handoff: the Core-0 task sleeps on a FreeRTOS task notification, measures into a private buffer, and publishes it under `measurementMutex` (flags `measurementRequested`/`measurementInProgress`/`measurementReady` and `latestMeasurements[]` are all guarded by that mutex). A request made during a sweep triggers a fresh sweep afterwards. See Known gotchas.

### Modules (`lib/`)

- **ConnectionManager** (`lib/ConnectionManager/`) - WiFi station connect with auto-reconnect and modem sleep off (`initWiFi`), ArduinoOTA with hostname `gadget` (`gadget.local`), optional `OTA_PASSWORD`, progress/error messages and a stop-motors hook (`initOta`), and a `WebSocketsServer` on port 88 (`initWebsocket`). Frames up to `MAX_MESSAGE_LENGTH` (64) bytes are passed to a `std::function` callback. If `WS_TOKEN` is defined, a client must send `AUTH <token>` first or it is disconnected; `send()` goes only to ready (authenticated) clients, `sendTo(num, ...)` to one. Callbacks: `setOnDisconnect`, `setOnOtaStart`, `setOnClientReady`.
  - WiFi credentials (and optional `OTA_PASSWORD`, `WS_TOKEN`) live in `lib/ConnectionManager/secrets.h`, which is gitignored; copy `secrets.h.example` and fill it in. Never commit or paste its contents. (An old hotspot credential is still in git history, commit `14d8889`.)
- **DistanceManager** (`lib/DistanceManager/`) - owns 10 `DistanceSensor` (VL53L0X) instances in an array indexed by `SensorDirection`, each with a unique I2C address (`0x30`-`0x39`) assigned at boot via XSHUT pins (`initAddresses`). A sensor that fails to boot is marked offline (held in reset) and reports `OUT_OF_RANGE_MM` (`0xFFFF`); boot continues. Exposes a non-blocking request/poll API (`requestMeasurement`, `isMeasurementReady`, `isMeasurementBusy`, `getLatestMeasurements`, `clearMeasurementReady`) backed by the Core-0 task. Sensor order is the enum order `FRONT, FRONT_LEFT_45, FRONT_LEFT, FRONT_RIGHT_45, FRONT_RIGHT, BACK, BACK_LEFT_45, BACK_LEFT, BACK_RIGHT_45, BACK_RIGHT`; `directionToString()` yields the exact wire names.
- **Steering** (`lib/Steering/`) - `SteeringManager` maps a `DriveDirection` (`FORWARD`, `FORWARD_LEFT`, `FORWARD_RIGHT`, `LEFT`, `STOP`, `RIGHT`, `BACKWARD`, `BACKWARD_LEFT`, `BACKWARD_RIGHT`; names in `kDriveDirectionNames`, `DriveDirection.h`) + a target `speed` to per-wheel target speeds (turns scale the inner wheel by `TURNING_FACTOR = 0.7`), then runs a `PidController` per wheel (`Kp=0.15, Ki=0.05, Kd=0.0` defaults). Encoder pulses come from the ESP-IDF `driver/pcnt` peripheral; the PID uses real elapsed time, conditional-integration anti-windup, and reads the PCNT counter as a signed delta against the previous raw reading (`EncoderTracker`), exposed as `PidController::getSignedDelta()` (positive = robot-forward via `LEFT_ENCODER_SIGN`/`RIGHT_ENCODER_SIGN`, both -1); the counter is only cleared when |raw| exceeds 16000, near the int16 hardware limit, so at most the few pulses between that read and clear are lost. `setDirection()` returns false for unknown names and resets the PIDs only when the direction actually changes. `stop()` zeroes the motors immediately.
- **Odometry** (`lib/Odometry/`) - hardware-free differential-drive pose integration (`Odometry::update(leftTicks, rightTicks)`, arc/midpoint model, pose in mm / radians, REP-103 frame). `SteeringManager::tickLocked` feeds it the signed per-wheel tick deltas every control tick (`getPose()` / `resetPose()`, mutex-guarded); `-DDEBUG_POSE` prints the pose on serial every 500 ms. `EncoderTracker` (same lib) turns raw PCNT readings into deltas and tells the caller when to clear the counter. Robot constants (wheel, track, `TICKS_PER_WHEEL_REV`, sensor mounts) are in `lib/Steering/RobotGeometry.h`; see `ENCODERS.MD` for the encoder decoding.
- **SensorCalibration** (`lib/SensorCalibration/`) - hardware-free `correctReading(sensorIndex, rawMm)`: per-sensor piecewise-linear raw -> true distance curve from the 0.4 measurements (the VL53L0X bias grows with distance and differs per sensor). Applied to `MEASURE` frames only; `CAL` frames carry raw readings so recalibration is not circular. Out-of-range passes through. Host-tested (`test/test_sensor_calibration`).
- **Commands** (`lib/Commands/`) - `CommandHandler` parses frontend commands (strictly: `SPEED` digits only and clamped to `MAX_SPEED`; `PID` exactly 3 finite non-negative floats), dispatches to the managers, answers rejects with `ERR <reason>`, sends `STATE ...` to a client when it becomes ready, stops the robot and continuous measurement when a client disconnects, and schedules/broadcasts measurement sweeps. (The 1s command timeout, `COMMAND_TIMEOUT_MS`, lives in `SteeringManager`'s control task.) `main.cpp` only wires the modules together.

## Tools

`tools/analyze_sensors.py` (stdlib Python) turns the CSV downloaded from the Gadget-UI "Sensor calibration" panel into bias / noise / trusted-range tables and the C++ arrays for `lib/SensorCalibration/SensorCalibrationData.h` (`MAPPING.MD` 0.4). Usage: `python tools/analyze_sensors.py recording.csv [more.csv]`.

## Roadmap

The mapping backlog (odometry, occupancy grid, scan matching, ...) lives in `MAPPING.MD`.

## Communication schema (WebSocket, port 88, plain text frames)

No JSON — every message is a plain space-delimited ASCII string. The server is the robot; the frontend is the client.

### Frontend -> Robot (commands), handled in `CommandHandler::handle()` (`lib/Commands/`)

| Message | Effect |
|---|---|
| `AUTH <token>` | Only if the firmware defines `WS_TOKEN`: must be the first frame. Wrong token = disconnect. Handled in `ConnectionManager`. |
| `SPEED <0..2000>` | Sets `SteeringManager` target speed (digits only, clamped to `MAX_SPEED`; applies to all subsequent direction commands) |
| `PID <P> <I> <D>` | Sets PID gains (finite floats >= 0) for both wheel controllers, resets integral term |
| `MEASURE_START` | Enables continuous measurement mode (robot auto-requests a new 10-sensor sweep every 200ms) |
| `MEASURE_STOP` | Disables continuous measurement mode |
| `MEASURE` | Requests exactly one measurement sweep (non-blocking; result arrives async via `MEASURE ...` broadcasts) |
| `POSE_RESET` | Zeroes the odometry pose to (0, 0, 0) and immediately broadcasts the new `POSE`. |
| `CAL <distance_mm> <count>` | Sensor characterisation (`MAPPING.MD` 0.4): robot must stand still (else `ERR robot moving`); ranges `<count>` (1-500) sweeps back to back at the true wall distance `<distance_mm>` (1-2000) and streams them as `CAL` frames. Stops continuous measurement; `MEASURE`/`MEASURE_START` are rejected (`ERR calibration running`) until it ends. |
| `CAL_STOP` | Ends a running `CAL` early (answered with `CAL_DONE`). |
| A `DriveDirection` name | `FORWARD`, `FORWARD_LEFT`, `FORWARD_RIGHT`, `LEFT`, `STOP`, `RIGHT`, `BACKWARD`, `BACKWARD_LEFT`, `BACKWARD_RIGHT`. The robot stops by itself if the active direction is not refreshed within 1s, so the frontend repeats it every 500ms while held. |
| Anything else | Rejected: logged and answered with `ERR unknown command` (no crash). |

### Robot -> Frontend

- `SWEEP_POSE T0=<ms> X0=<mm> Y0=<mm> TH0=<mrad> T1=<ms> X1=<mm> Y1=<mm> TH1=<mrad>` — sent immediately before each `MEASURE` frame: the odometry pose (and the control-tick `millis()` it was computed in) when the sweep started and ended, so the UI can interpolate a pose per reading. Not sent for `CAL` frames. The poses are snapshotted by the Core-0 sweep task through the pose provider set with `DistanceManager::setPoseProvider` (`main.cpp` wires it to `SteeringManager::getStampedPose`).
- `MEASURE <DIRECTION>=<mm|OOR> ...` — a single frame per completed sweep, e.g. `MEASURE FRONT=412 FRONT_LEFT_45=OOR ...`, containing all 10 sensors in `sensorsOrder`. `OOR` means out of range / no obstacle (also reported for sensors that are offline). `DIRECTION` is one of the 10 `SensorDirection` strings above (see `directionToString`). The frontend must treat the frame as one atomic sweep.
- `POSE X=<mm> Y=<mm> TH=<mrad> T=<ms> Q=<0|1>` - odometry pose (REP-103: +x forward, +y left, TH counter-clockwise in milliradians, `T` = robot `millis()`), broadcast to ready clients every `POSE_INTERVAL_MS` (200 ms) and sent to a client right after its `STATE`. `Q` is always 0 until the slip guard (`MAPPING.MD` 1.5) exists; `RELOC=1` will be appended by 4.3.
- `CAL <distance_mm> <i>/<n> <DIRECTION>=<mm|OOR> ...` - one sweep of a `CAL` run (same sensor fields as `MEASURE`); `CAL_DONE <distance_mm> <done>` ends the run (also after `CAL_STOP`, with the number of sweeps actually done).
- `STATE SPEED=<n> P=<f> I=<f> D=<f> MEASURING=<0|1>` - sent to a single client when it becomes ready (connected, and authenticated if `WS_TOKEN` is set) so the UI can initialise from the robot.
- `ERR <reason>` - a command was rejected (`invalid SPEED`, `invalid PID`, `invalid CAL`, `robot moving`, `calibration running`, `unknown command`).
- Nothing is sent for accepted `SPEED`/`PID`/direction commands (fire-and-forget).

## Known gotchas / fragile areas

- **`measurementReady` must be cleared after each consumption** (`CommandHandler::broadcastSweep` does it). It is a single-slot flag, not a queue. `requestMeasurement()` sets it `false`; the Core-0 task sets it `true` when a sweep finishes. If it is not cleared, continuous mode never re-requests and the same sweep is re-broadcast on every loop iteration. Continuous mode also checks `isMeasurementBusy()` so it does not stack requests on a running sweep.
- **Unknown direction strings are rejected, not fatal** (`SteeringManager::setDirection` returns false). Do not reintroduce `std::map::at` (Arduino-ESP32 builds without exceptions, so a miss would abort and reboot the robot).
- **OTA and WebSocket auth are opt-in.** Without `OTA_PASSWORD` / `WS_TOKEN` in `secrets.h`, anyone on the network can flash the robot or drive it. Fine on a trusted home network only. The token is sent in clear text over `ws://`.
- **Failsafes live in firmware, not just the UI:** STOP on client disconnect, on OTA start, after 1s without a direction command, and a loop-watchdog reset. Re-verify them after changes to `CommandHandler`/`ConnectionManager`.
- **Do not try to speed up the sensor sweep by ranging sensors concurrently (see `SENSOR_FLOW.md`).** It was tried twice and is unreliable on this hardware. A sweep takes ~390ms, so continuous mode (requested every 200ms, `MEASUREMENT_INTERVAL_MS`) yields about 2-3 sweeps per second. Several side sensors (FRONT_LEFT, BACK_LEFT) are noisy at the edge of their range even when ranged one by one.

## Companion frontend

The browser control UI lives in a separate repo: `C:\Users\fwisn\Documents\Gadget-UI` (see its own `CLAUDE.md`). It connects directly to `ws://<robot-ip>:88` and implements the other half of the schema above.
