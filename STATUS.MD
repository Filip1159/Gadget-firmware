# Robot Firmware - Status / Backlog

Code review snapshot of the firmware (2026-10-20, branch `main`, including uncommitted changes). This is a worklist, not a changelog. Items are ordered by priority inside each section. File references use `path:function` rather than line numbers because the code is still moving.

Companion list for the frontend: `C:\Users\fwisn\Documents\the-sneaky-spy\STATUS.MD`. Items tagged **[cross-repo]** need a change on both sides.

Note: no `STATUS.MD` existed in this repo before this file (the old one that `CLAUDE.md` refers to was never committed). The history of the Core-0 measurement fix is only in git (`51b77e5` and nearby).

---

## Resolved (2026-10-20, build verified, not yet flashed/tested on the robot)

- 1.1 failsafe: `STOP` on WebSocket disconnect and on OTA start; 1s command timeout in `loop()` (`COMMAND_TIMEOUT_MS`). WiFi drops are covered by the timeout.
- 1.1 unknown commands: `SteeringManager::setDirection` uses `find()` and returns bool; `updateSteering` logs and ignores unknown strings.
- 1.1 `SPEED`: strictly parsed (digits only) and clamped to `MAX_SPEED` (2000); invalid values are rejected.
- Side effect (1.2): PIDs are now reset only when the direction actually changes, not on every repeated command.

Still open in 1.1: nothing. Remaining items below are unchanged.

---

## 1. Bugs

### 1.1 Safety-critical

- **Robot never stops on its own (no dead-man / failsafe).** `SteeringManager` keeps the last direction forever. Nothing stops the motors when:
  - the WebSocket client disconnects (`WStype_DISCONNECTED` in `ConnectionManager::initWebsocket` only prints),
  - WiFi drops (no `WiFi.onEvent` / reconnect handling after boot),
  - the frontend stops sending commands,
  - an OTA update starts (`ArduinoOTA.onStart` doesn't touch the motors).
  Fix: stop on disconnect and on OTA start, plus a command timeout (e.g. no direction command for ~1s => `STOP`). **[cross-repo]** The frontend currently re-sends direction every 500ms while a button is held, so a ~1s timeout fits its heartbeat, but the frontend also never sends `STOP` on button release (see the frontend STATUS), so both halves are needed.
- **Unknown command strings crash the firmware.** `SteeringManager::setDirection` calls `directionStrings.at(direction)`. Arduino-ESP32 is built without C++ exceptions (the build flags are not verified here), so the missing-key case most likely ends in `abort()` and a reboot rather than a catchable exception. Any client, or one frontend typo, can reboot the robot mid-drive. **[cross-repo]** The frontend's keyboard map sends `BRAKE` for `s`, and `"undefined"` for keys outside the map. Neither is valid. Fix: use `find()`, return a bool, and log or reply with an error.
- **`SPEED` is not validated and can wrap.** `main.cpp:updateSteering` does `setSpeed(message.substring(5).toInt())`, which narrows `long` to `uint16_t`. `SPEED -100` becomes 65436. `PidController::targetSpeed` is `int16_t`, so anything above 32767 flips to a negative target. `SPEED abc` silently becomes 0. Fix: parse safely and clamp to a defined range (frontend slider is 0-2000).

### 1.2 Control / PID

- **PID is reset on every repeated direction command.** `updateSteering`'s fallback branch calls `resetPids()` for every message, including a repeat of the current direction. With the frontend's 500ms repeat while a button is held, the integral term is wiped twice a second, which causes periodic speed ripple. Fix: only reset when the direction actually changes. Also consider not resetting on STOP->STOP.
- **Anti-windup in `PidController::loop` is dead code.** `if (output == 255 && error > 0)` compares the *unclamped* float output against exactly 255/0, which practically never matches, so the integral winds up freely. Fix: clamp first, then do conditional integration (skip integrating when saturated and the error pushes further into saturation), or use back-calculation.
- **Stale encoder counts after a stop.** When `targetSpeed == 0`, `loop()` returns early without reading or clearing the PCNT counter or the integral/previous error. Pulses that accumulate while coasting or being pushed are included in the first sample after the next command, so the first PID tick sees an inflated speed and undershoots. Fix: always read and clear the counter every tick, and reset state when stopped.
- **Speed is computed assuming exactly 100ms ticks** (`count / 0.1`, and `error * 0.1` in the integral/derivative). The loop in `main.cpp` is cooperative and can slip when `broadcastTXT`, `Serial`, or OTA blocks. Fix: measure real `dt` with `micros()`/`esp_timer_get_time()`. Ideally run the PID in its own FreeRTOS task or hardware timer so it isn't coupled to network I/O.
- **PCNT read + clear is not atomic** (`pcnt_get_counter_value` then `pcnt_counter_clear`), so pulses between the two calls are lost. Small error, but systematic at high speed. Alternative: keep a running count and diff against the previous reading instead of clearing.
- `PidController::direction` is uninitialised and `targetSpeed` defaults to 1000, so a wheel would run if `loop()` were called before `setTargetSpeed`. It's currently safe only because `SteeringManager::loop` always sets the target first. Default to `WHEEL_FORWARD` / 0.

### 1.3 Sensors

- **Wrong printf format in `DistanceSensor::setAddress`:** `Serial.printf("Failed to boot %s VL53L0X\n", address)` passes an `int` for `%s`, which is undefined behaviour and probably a crash exactly when you most need the message. Use `%#x`.
- **One dead sensor bricks the robot at boot.** The same function does `while (1) {}` on failure, so the PID/steering init never runs, and `ArduinoOTA.handle()` is never serviced (it lives in `loop()`), so you can't even flash a fix over WiFi, only over USB. Fix: mark the sensor offline, keep booting, and report an explicit "no data" value for it.
- **`DistanceSensor::measure` ignores the return value of `rangingTest` and leaves `measure` uninitialised.** If the call fails or times out, `RangeStatus`/`RangeMilliMeter` are garbage. Initialise the struct and check the return value/status.
- **Out-of-range is conflated with 1000mm.** Status 4 is mapped to a literal `1000`, but valid readings can exceed 1000mm (VL53L0X reads up to ~1200mm in default mode), so the frontend cannot tell "no obstacle" from "1000mm obstacle", and other non-zero `RangeStatus` values (e.g. signal/sigma failures) are passed through as if they were valid distances. **[cross-repo]** Define a wire sentinel (e.g. `-1`/`OOR`) and a named constant.
- **Lost / late measurement requests (`DistanceManager::measurementTask`).** The task clears `measurementRequested` after a sweep, so a request that arrives *during* a sweep is swallowed, and the "ready" result that follows is up to ~330ms old relative to the request. `measurementRequested` is also read outside the mutex, and `latestMeasurements` is written outside it. It's benign today only because a single consumer calls `getLatestMeasurements` right after `ready` is set. A proper handoff (see 2.4) removes the whole class of problem.
- The `measurementTask` loop polls every 10ms with `vTaskDelay`, which adds up to 10ms latency per request and a constant wakeup for nothing.
- Stack for the measurement task is a magic 4096 with no high-water-mark check (`uxTaskGetStackHighWaterMark`). Worth verifying it has headroom since `Serial.printf` and the Adafruit library run on it.

### 1.4 Connectivity / protocol

- **PID parsing is fragile** (`updateSteering`): relies on `String::indexOf(" ")` returning -1 for the last token and on `substring` clamping the unsigned wraparound. Missing tokens or extra whitespace silently produce zeros. Result: a malformed `PID` message can set `Kp=Ki=Kd=0` and the robot stops responding to control with no error. Use `sscanf`/`strtof` with validation.
- `message.startsWith("SPEED")` / `startsWith("PID")` also match things like `SPEEDY` or `PIDX`. Match the command token exactly.
- **Slow or stalled client can block the control loop.** `ConnectionManager::send` does 10 synchronous `broadcastTXT` calls per sweep (each a blocking TCP write) from the same loop that runs the PID. A bad WiFi moment delays PID ticks (see "assumes 100ms" above).
- **Measurement mode outlives the client.** `continuousMeasurement` stays `true` after the client disconnects, so the robot keeps sweeping and broadcasting to nobody. A client that reconnects shows "Start Measuring" while the robot is already streaming (**[cross-repo]**, see frontend). Fix: stop on last-client disconnect, and/or send state on connect.
- `Serial.println((char*)payload)` assumes the payload is NUL-terminated and short; there's no length cap before constructing `String`.
- **Debug output on the hot path.** `PidController::loop` builds a large `String` with ~8 concatenations and prints it, per wheel, every 100ms. That's heap churn/fragmentation over long runs, and on the Nano ESP32's USB-CDC `Serial` it may stall when no host is attached (worth verifying with the robot running on battery without a PC). Gate it behind a `DEBUG_PID` flag or the `PID` command.

### 1.5 Config / build

- **WiFi credentials are hardcoded in `lib/ConnectionManager/ConnectionManager.h`, a tracked file.** The current working-tree value (a home network) is uncommitted, but an earlier phone-hotspot credential is already in history (commit `14d8889`). Before committing: move to an untracked `secrets.h` (gitignored, with a `secrets.h.example`) or `build_flags` from an env var, and consider rotating the home network password if it was ever pushed anywhere. Do not paste the file contents into issues/chats.
- `platformio.ini` leaves `platform = espressif32` **unpinned**. The installed Arduino core is 3.0.x (IDF 5.x), which deprecates the legacy `driver/pcnt.h` API used here. A future platform update can break the build or the encoder code. Pin the platform version (e.g. `platform = espressif32@6.x.y`) and plan the PCNT migration (see 3.3).
- `upload_speed = 115200` is the current working-tree value (it was 921600 in HEAD). If that was a workaround for a flaky cable/port, note it here; otherwise USB uploads are needlessly slow.
- `.idea/` is untracked and not in `.gitignore`.
- `test/` is empty, so nothing is covered by automated tests.

---

## 2. Bad practices / code smells

1. **`main.cpp` is doing too much.** `updateSteering()` is actually the whole command dispatcher (speed, PID, measurement mode, steering), uses string slicing for parsing, and mutates globals. The name is misleading. It also holds the measurement scheduling, result broadcasting, and the heartbeat LED. Extract a `CommandHandler`/`CommandParser` (tokenise -> validate -> dispatch -> return ok/error), and let `DistanceManager` own continuous-mode scheduling.
2. **Raw owning pointers that are never freed** (`new` in constructors): `SteeringManager` (`left`/`right`), `DistanceManager` (`sensorsOrder`, 10 sensors, map), `DistanceSensor` (`Adafruit_VL53L0X`), `ConnectionManager` (`WiFiMulti`, `WebSocketsServer`). It's harmless for process-lifetime singletons, but unnecessary. Prefer plain members/arrays (`DistanceSensor sensors[10]`, `const SensorDirection kOrder[10]`).
3. **`std::map` for fixed 10-element / 9-element tables** (`sensors`, `directionStrings`): heap use plus `at()` that can abort. Use arrays indexed by enum, and a `constexpr` string table shared by `directionToString` and the parser.
4. `DistanceManager::directionToString(SensorDirection)` has no return after the `switch` (UB for an invalid enum, `-Wreturn-type`). The overload for `uint8_t` vs enum is also easy to call wrongly.
5. **Dead code:** `initPinsAsOutputs()` (never called, not declared), `measureAll()`, `measure(direction)`, `getSensorsOrder()`, the `WiFiMulti` instance and `#include <WiFiMulti.h>`/`<WiFiClientSecure.h>` in `ConnectionManager`, unused `ArduinoOTA` TODO comment about SPIFFS.
6. **Magic numbers everywhere:** `10` sensors (repeated ~10 times), `200`/`100`/`1000` ms intervals, `1000` out-of-range, `0.1` dt, `255` PWM max, `100` PCNT filter, `4096` stack. Name them (`SENSOR_COUNT`, `STEERING_PERIOD_MS`, ...).
7. **`#define` for addresses/pins** instead of `constexpr`; pin naming is inconsistent (`SteeringManager` passes `LEFT_2_PIN, LEFT_1_PIN` to `PidController(motorPin1, motorPin2)`, i.e. swapped, to compensate for wiring/direction; document it or fix the constants).
8. **Callbacks as raw function pointers + `String` by value** (`websocketCallback(String)`); use `std::function<void(const String&)>` or an interface so handlers can capture state instead of using globals.
9. `directionStrings` map is a non-static member, so a copy is built per `SteeringManager` instance.
10. The 1s LED heartbeat in `loop()` is useful but untied to any health signal (WiFi/WS/sensors). Could encode state instead (e.g. fast blink = no client).
11. **`CLAUDE.md` drift:** it says the serial monitor prints "OTA progress", but there is no `onProgress` handler, and `onError` prints only `Error[n]: ` with no message. Either add the handlers or fix the doc.

---

## 3. Useful refactors / improvements

### 3.1 Safety & robustness (do these first)

- Failsafe stop on disconnect / command timeout / OTA start (see 1.1).
- **Sensor-based protection:** the firmware already has all the data to refuse to drive into an obstacle (e.g. clamp or stop when `FRONT*` < N mm while moving forward; same for `BACK*`). Currently every decision is made by the human via the browser at 2-3Hz latency.
- OTA hardening: `ArduinoOTA.setPassword(...)` and `setHostname("robot")` (also gives you `robot.local` instead of IP juggling; add mDNS and the frontend could stop hardcoding the IP).
- WebSocket auth: at minimum a shared token in the first message or URL (`webSocket->setAuthorization`), since anyone on the LAN can drive the robot.
- Task watchdog: the PID loop should be fed by a monitored task so a hang (e.g. `while(1)` on sensor boot, blocked TCP write) resets the robot rather than leaving motors running.
- WiFi: `WiFi.setAutoReconnect(true)`, handle `ARDUINO_EVENT_WIFI_STA_DISCONNECTED`, and call `WiFi.setSleep(false)` to cut WebSocket latency from modem sleep.

### 3.2 Sensor pipeline (could make measurements ~10x faster)

- Each sweep is ~330ms because every sensor is ranged sequentially and blocking (~33ms timing budget each). With `Adafruit_VL53L0X::startRange()` / `isRangeComplete()` / `readRange()` (or continuous ranging) you can **start all 10 sensors, then collect all 10**, giving a sweep close to a single timing budget (~35-50ms). That would also make the Core-0 task unnecessary or much simpler, and removes the entire request/ready/clear flag protocol.
- Raise the I2C clock to 400kHz (`Wire.setClock(400000)`) if wiring allows.
- If the Core-0 task is kept: replace the flag + 10ms polling with `xTaskNotifyGive`/`ulTaskNotifyTake`, and the shared slot with a double buffer or a 1-deep FreeRTOS queue carrying a sequence number + timestamp, so consumers can tell fresh from stale.
- Add per-sensor health (online/offline, consecutive failures) and optional median/EMA filtering.

### 3.3 Motor control

- **Convert units.** Speed is "encoder pulses per second", and the slider range (0-2000) has no physical meaning. Add `PULSES_PER_REV`, gear ratio and wheel diameter constants and work in mm/s. That also unlocks odometry.
- Add **feed-forward** (target -> approximate PWM) so the PID only corrects error; reduces the slow ramp-up and the need for high Ki.
- Consider a **heading/sync term** so the two wheels stay matched when driving straight (today each wheel is an independent loop, so drift is expected). An IMU would be the real fix.
- Acceleration ramp on target changes; avoids slip and the current-spike on reversal (direction can flip between ticks with no dead time).
- Derivative on measurement (not error) and a small low-pass filter on speed. 100ms windows at low speeds give very coarse (quantised) speed samples; consider 20-50ms ticks or period-based measurement.
- Raise PWM frequency (`analogWriteFrequency`) above the audible range if the motors whine at the default.
- Migrate to the new PCNT API (`driver/pulse_cnt.h`) when moving to a newer core; at the same time it can use real quadrature decode and glitch filter config.
- Make `LEFT`/`RIGHT` spin speed and `TURNING_FACTOR` configurable (command or compile-time) instead of a hardcoded 0.7.

### 3.4 Protocol

- **Batch the measurement into one frame** (`MEASURE v0 v1 ... v9` or a small JSON/binary frame) instead of 10 frames per sweep. It cuts TCP writes and heap churn, removes the frontend's "partial sweep" repaint flicker, and makes a sweep atomic. **[cross-repo]**
- Add replies: `OK`/`ERR <reason>` for commands (or at least for `PID`/`SPEED`), and a state frame on connect (current speed, PID gains, measuring on/off, firmware version) so the UI can initialise from the robot instead of hardcoding the same defaults on both sides. **[cross-repo]**
- Consider a monotonic timestamp in measurement frames so the UI can show staleness.
- Make the command set explicit and versioned (a small table in one header + a section in `CLAUDE.md`), so adding a command does not mean editing a chain of `else if`.
- Optional: `STATUS` frame (battery voltage, WiFi RSSI, loop time, free heap) for diagnostics.

### 3.5 Tooling

- Add a `[env:native]` PlatformIO environment with Unity tests for pure logic: command parsing, PID math (after splitting hardware access behind a small interface), direction mapping. These are the parts that have bitten you already.
- Add `secrets.h`/example and `.gitignore` entries (`.idea/`, `secrets.h`).
- Optional `[env:ota]` in `platformio.ini` so switching between USB and OTA is a flag rather than commenting lines in/out.
- Add `-Wall -Wextra` in `build_flags` to catch the format-string and missing-return issues above at compile time.
