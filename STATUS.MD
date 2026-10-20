# Robot Firmware - Status / Backlog

Working list for the firmware. `[x]` = implemented, `[ ]` = open. File references use `path:function` rather than line numbers because the code is still moving.

Companion list for the frontend: `C:\Users\fwisn\Documents\the-sneaky-spy\STATUS.MD`. Items tagged **[cross-repo]** needed (or need) a change on both sides; the wire format changed, so **flash the firmware and deploy the frontend together**.

All `[x]` items are implemented and verified on the robot.

---

## 1. Bugs

### 1.1 Safety-critical

- [x] **Robot never stopped on its own.** Now: `STOP` on WebSocket disconnect and on OTA start, a 1s command timeout (`COMMAND_TIMEOUT_MS`, `CommandHandler::loop`), and a loop watchdog (`enableLoopWDT()`, 5s, resets the board, which releases the motor pins). WiFi drops are covered by the timeout. **[cross-repo]** frontend sends `STOP` on release and repeats the direction every 500ms while held.
- [x] **Unknown command strings crashed the firmware** (`std::map::at` aborts on Arduino-ESP32, which has no exceptions). `SteeringManager::setDirection` now does a table lookup and returns false; `CommandHandler` answers `ERR unknown command`. **[cross-repo]** frontend commands are typed, and the `BRAKE`/`"undefined"` senders are gone.
- [x] **`SPEED` was not validated** (negative wrapped to 65436). Now digits only, clamped to `MAX_SPEED` (2000), rejects with `ERR invalid SPEED`.

### 1.2 Control / PID

- [x] PID was reset on every repeated direction command. Now only on an actual direction change.
- [x] Anti-windup was dead code (`output == 255`). Now conditional integration on the clamped output.
- [x] Stale encoder counts after a stop. The PCNT counter is read and cleared every tick, also while stopped; PID state resets when stopped.
- [x] Speed assumed exactly 100ms ticks. Now uses real elapsed time (`micros()`, clamped to 20-500ms, falls back to 100ms).
- [x] `PidController` defaults (`direction = WHEEL_FORWARD`, `targetSpeed = 0`).
- [ ] **PCNT read + clear is not atomic** (`pcnt_get_counter_value` then `pcnt_counter_clear`), so a few pulses between the two calls are lost. Small but systematic at high speed. Alternative: keep a running count and diff against the previous reading instead of clearing.
- [ ] The PID still runs in the main loop, coupled to network I/O (a blocking TCP write delays a tick; the real `dt` now compensates for the maths but not for the jitter). Ideally its own FreeRTOS task or hardware timer.

### 1.3 Sensors

- [x] Wrong `printf` format in `DistanceSensor::setAddress` (`%s` with an int). Fixed (`%#x`, also prints the XSHUT pin).
- [x] One dead sensor bricked the robot at boot (`while(1)`, OTA never serviced). The sensor is now marked offline and held in reset so the others can still be addressed; boot continues; offline sensors report `OOR`.
- [x] `DistanceSensor::measure` ignored the `rangingTest` result and left the struct uninitialised. Now zero-initialised and checked.
- [x] Out-of-range was conflated with 1000mm. `OUT_OF_RANGE_MM = 0xFFFF` internally, `OOR` on the wire. **[cross-repo]**
- [ ] Other non-zero `RangeStatus` codes (sigma/signal failures) are still passed through as valid distances; only status 4 and bus errors become `OOR`. Real readings above 1000mm are now reported as-is.
- [x] Lost/late measurement requests and unguarded shared state (`DistanceManager::measurementTask`). The task sleeps on a task notification (no 10ms polling), measures into a private buffer, publishes under the mutex, and a request made mid-sweep triggers a fresh sweep. All flags are mutex-guarded instead of `volatile`. `isMeasurementBusy()` stops continuous mode from stacking requests.
- [x] Measurement task stack size is now observable: the high-water mark is printed once after the first sweep. 

### 1.4 Connectivity / protocol

- [x] PID parsing was fragile. Now `sscanf` with strict validation (exactly 3 finite, non-negative floats); bad input gets `ERR invalid PID` instead of silently zeroing gains.
- [x] `startsWith("SPEED")`/`("PID")` matched `SPEEDY`/`PIDX`. Prefixes now include the space.
- [ ] **A slow or stalled client can still block the control loop.** The 10 blocking `broadcastTXT` calls per sweep became one `sendTXT` per ready client per sweep, but each is still a synchronous TCP write from the loop that runs the PID.
- [x] Measurement mode outlived the client. Continuous measurement now stops on disconnect (any client; fine for the single-client use case) and the `STATE` frame tells a reconnecting client the real state. **[cross-repo]**
- [x] No length cap on text frames: anything over `MAX_MESSAGE_LENGTH` (64) is ignored.
- [x] PID debug output on the hot path (String concatenation + serial per wheel per tick) is behind `-DDEBUG_PID` (commented line in `platformio.ini`).

### 1.5 Config / build

- [x] **WiFi credentials were hardcoded in a tracked header.** Now in a gitignored `lib/ConnectionManager/secrets.h` (see `secrets.h.example`). [ ] An old hotspot credential is still in git history (commit `14d8889`); change that hotspot password if it is still in use.
- [ ] `platform = espressif32` is **unpinned**. The installed Arduino core is 3.0.x (IDF 5.x), which deprecates the legacy `driver/pcnt.h` API used here. Pin the platform version (e.g. `espressif32@6.x.y`) and plan the PCNT migration (3.3).
- [ ] `upload_speed = 115200` (it was 921600 in an older commit). If that is a workaround for a flaky cable/port, note it here; otherwise USB uploads are needlessly slow.
- [x] `.idea/` is in `.gitignore`.
- [ ] `test/` is empty: nothing is covered by automated tests (see 3.5).

---

## 2. Bad practices / code smells

- [x] `main.cpp` did too much. The command dispatcher, command timeout, continuous-measurement scheduling and sweep broadcast moved into `lib/Commands/CommandHandler`; `main.cpp` only wires modules and runs the steering/heartbeat timers.
- [ ] Raw owning pointers that are never freed (`new` in constructors): `SteeringManager` (`left`/`right`), `DistanceManager` (10 `DistanceSensor*`), `DistanceSensor` (`Adafruit_VL53L0X`), `ConnectionManager` (`WebSocketsServer`). Harmless for process-lifetime singletons; plain members would be cleaner.
- [x] `std::map` for fixed tables replaced by arrays/tables (`sensors[]` indexed by `SensorDirection`, `kSensorNames[]`, `kDriveDirectionNames[]`), no heap and no abort-on-miss.
- [x] `directionToString` had no return after the `switch`.
- [x] Dead code removed (`initPinsAsOutputs`, `measure`, `measureAll`, `getSensorsOrder`, unused `WiFiMulti`/`WiFiClientSecure`).
- [x] Magic numbers, mostly: `SENSOR_COUNT`, `STEERING_PERIOD_MS`, `MEASUREMENT_INTERVAL_MS`, `HEARTBEAT_PERIOD_MS`, `COMMAND_TIMEOUT_MS`, `MAX_SPEED`, `MAX_MESSAGE_LENGTH`, `OUT_OF_RANGE_MM`. [ ] Still literal: PWM max `255`, PCNT filter `100`, task stack `4096`, `TURNING_FACTOR 0.7` is a `#define` but not configurable.
- [ ] `#define` for addresses/pins instead of `constexpr`; pin naming is inconsistent (`SteeringManager` passes `LEFT_2_PIN, LEFT_1_PIN` to `PidController(motorPin1, motorPin2)`, i.e. swapped to compensate for wiring/direction; document it or fix the constants).
- [x] Callbacks are `std::function` (lambdas can capture state), not raw function pointers.
- [x] `directionStrings` was a per-instance `std::map`; now a shared `static const` table.
- [ ] The 1s LED heartbeat is untied to any health signal. Could encode state (fast blink = no client, etc.).
- [x] `CLAUDE.md` drift: it claimed OTA progress was printed. Progress and error messages now exist, and `CLAUDE.md` was rewritten to match the code (modules, wire format, failsafes, secrets).

---

## 3. Useful refactors / improvements

### 3.1 Safety & robustness

- [x] Failsafe stop on disconnect / command timeout / OTA start (see 1.1).
- [ ] **Sensor-based protection:** the firmware has the data to refuse to drive into an obstacle (stop or clamp when `FRONT*` < N mm while moving forward; same for `BACK*`). Currently every decision is made by the human at 2-3Hz latency. Needs on-robot tuning, so not done blind.
- [x] OTA hardening: hostname `robot` (`robot.local`) and optional password (`OTA_PASSWORD` in `secrets.h` + `upload_flags = --auth=...`). Off by default so the existing OTA flow is unchanged.
- [x] WebSocket auth, opt-in: define `WS_TOKEN` in `secrets.h`; clients must send `AUTH <token>` first, others are disconnected. [ ] The token travels in clear text over `ws://` (fine for a home LAN, not beyond it).
- [x] Watchdog: loop watchdog enabled (`enableLoopWDT()`); a hung `loop()` resets the board. [ ] The measurement task is not watched, and the PID is not in its own watched task (see 1.2).
- [x] WiFi: `setAutoReconnect(true)`, `setSleep(false)`, hostname.

### 3.2 Sensor pipeline

- [x] ~~Range sensors concurrently~~ **Tried and rolled back; do not retry (see README).** Details of the last attempt: `DistanceManager::sweep` runs 5 batches of 2 opposite-facing sensors (`kSweepBatches`): start both (`startRange`), poll `isRangeComplete`, read both; a sweep takes ~225ms instead of ~390ms. A sensor that has not finished within `SWEEP_TIMEOUT_MS` (200) reports `OOR` for that sweep. **Ranging all 10 at once does not work on this hardware:** 4 of the sensors failed to start (`VL53L0X_ERROR_TIME_OUT`, each stalling `startRange` for ~1.3s) and the rest returned 8191, so the UI showed only the first 5-6 readings. Batches of 5 or 2 adjacent sensors failed the same way; only opposite-facing pairs are clean. [ ] The Core-0 task is kept (the poll loop still blocks for ~one budget); continuous mode could now run faster than the 200ms interval.
- [ ] I2C clock at 400kHz: `Wire.setClock(400000)` is in `initAddresses` but commented out (it did not fix the parallel-ranging problem and is not needed at ~225ms per sweep).
- [x] Task notification instead of flag + polling; private buffer under a mutex. [ ] Not done: a sequence number / timestamp in the published sweep so consumers can tell fresh from stale.
- [ ] Per-sensor health (consecutive failures, auto-recover) and optional median/EMA filtering.

### 3.3 Motor control

- [ ] **Convert units.** Speed is "encoder pulses per second" and the slider range (0-2000) has no physical meaning. Add pulses/rev, gear ratio and wheel diameter and work in mm/s (also unlocks odometry).
- [ ] Feed-forward (target -> approximate PWM) so the PID only corrects error.
- [ ] Heading/sync term so the two wheels stay matched when driving straight (an IMU would be the real fix).
- [ ] Acceleration ramp on target changes; dead time on direction reversal.
- [ ] Derivative on measurement + a small low-pass filter on speed; 100ms windows at low speed are coarsely quantised (consider 20-50ms ticks or period-based measurement).
- [ ] Raise PWM frequency (`analogWriteFrequency`) above the audible range if the motors whine.
- [ ] Migrate to the new PCNT API (`driver/pulse_cnt.h`) when moving to a newer core.
- [ ] Make `TURNING_FACTOR` and spin speed configurable.

### 3.4 Protocol

- [x] **One batched `MEASURE FRONT=412 FRONT_LEFT_45=OOR ...` frame per sweep** instead of 10 frames. **[cross-repo]**
- [x] Replies and state: `ERR <reason>` for rejected commands, and a `STATE SPEED=.. P=.. I=.. D=.. MEASURING=..` frame to each client when it becomes ready, so the UI initialises from the robot. **[cross-repo]** [ ] Not done: `OK` acks for accepted commands, firmware version in `STATE`.
- [ ] Monotonic timestamp in measurement frames so the UI can show staleness per sweep.
- [ ] Make the command set a table (name -> handler) instead of an `if/else` chain in `CommandHandler::handle`.
- [ ] Optional `STATUS` frame (battery voltage, WiFi RSSI, loop time, free heap) for diagnostics.

### 3.5 Tooling

- [ ] Add a `[env:native]` PlatformIO environment with Unity tests for pure logic. Blocked on separating the logic from Arduino `String`/hardware (command parsing and PID maths would need small interfaces); no host C++ compiler is installed on this machine either.
- [x] `secrets.h` + example + `.gitignore` entries (`.idea/`, `secrets.h`).
- [ ] Optional `[env:ota]` in `platformio.ini` so switching between USB and OTA is a flag rather than commenting lines in/out.
- [x] `-Wall` in `build_flags` (and the warnings it raised are fixed). [ ] `-Wextra` not enabled yet.
