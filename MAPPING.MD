# Gadget mapping backlog

Goal: Gadget builds, keeps and shows a map of a room while it drives. The target is a roomba/lidar-like experience with ROS-inspired algorithms (odometry, occupancy grid, scan matching, particle filter, pose graph), using only the hardware it already has: two encoder-driven wheels and ten VL53L0X time-of-flight sensors. The map is shown in a browser panel in **Gadget-UI** (`C:\Users\fwisn\Documents\Gadget-UI`).

Non-goals: matching a real 2D lidar (360 beams, 10 Hz, 12 m). A VL53L0X reaches about 1.2 m at best (less in daylight or on dark/glossy surfaces), has a ~25 deg cone, and 10 fixed beams are a very sparse scan. The backlog is an experiment in how far this hardware can be pushed.

Each item: `ID (points, repo)`. Points are Fibonacci (1, 2, 3, 5, 8). Repo tags: `FW` firmware, `UI` Gadget-UI, `HW` hardware, `DOC` documentation, `TOOL` separate tool. Tick the box when the **Done when** condition holds.

## Accuracy ladder

| After phase | You get | Expected quality |
|---|---|---|
| 2 | Point cloud + pose trail | Smeared walls, drift visible after a few metres |
| 3-4 | Persistent occupancy grid, exportable | Usable outline of a small room when driving slowly or stop-and-go |
| 5 | Spin-scan pseudo-lidar | Clean rectangle-ish rooms from a few scan points |
| 6 | Gyro heading | Biggest drop in drift; straight walls stay straight |
| 7 | Scan matching, particle filter, localisation in a saved map | Corrects odometry drift, closest to the ROS experience |
| 8-10 | Loop closure, autonomy, ROS bridge | Stretch |

Recommended MVP cut line: **phases 0-4** (~95 points) give a persistent, exportable occupancy map. Phases 5-6 are what make it good. Phase 7 onwards is the ROS-like experiment.

## Architecture

```
encoders + 10 ToF sensors -> firmware (pose, pose-tagged sweeps) -> WebSocket :88 -> Gadget-UI (grid, matcher, persistence, map panel)
```

Design decisions:

1. **The robot is the sensor and odometry source, the browser is the mapper.** The grid, scan matcher and particle filter run in TypeScript in Gadget-UI: faster to iterate on, far more RAM/CPU than the ESP32, and the control loop stays untouched. The map moves on-board only at the very end (item 9.5).
2. **Persistence is browser-side first** (IndexedDB, export as ROS `map_server` PGM+YAML, PNG, JSON). A raw log (poses + sweeps) is recorded so algorithms can be replayed and tuned offline without driving again. Robot-side persistence (NVS, LittleFS) comes late.
3. **Scan-and-go before map-while-moving.** Early milestones only use sweeps taken while the robot is stationary, which removes motion skew (a sweep takes ~390 ms and the robot moves during it).
4. **Biggest accuracy wins, in order:** calibrated odometry, a gyro, a spin-scan (turn in place so the 10 sensors sample many bearings), then scan matching.

### Coordinate conventions (one world frame, ROS REP-103)

- World and robot frame: millimetres, **+x forward, +y left, theta counter-clockwise**, in milliradians on the wire. At power-on / `POSE_RESET` the robot is at (0, 0, 0). Origin of the robot frame is the centre of the robot (the midpoint between the wheels is assumed to coincide with it; verify in 0.2).
- Gadget-UI's `robot-geometry.ts` (`src/components/control-panel/robot-preview/`) uses canvas coordinates (x right, y down, front = -y, angles clockwise, front = 1.5 pi). Convert once, in one function: `x_robot = -y_ui`, `y_robot = -x_ui`, `bearing = 1.5 pi - angle_ui`. Example: UI FRONT (0, -90), angle 1.5 pi -> robot (90, 0), bearing 0.
- The UI geometry (135x180 mm body, 80 mm wheels, `SENSOR_MOUNTS`, `SENSOR_BEAM_ANGLE` 25 deg) is the starting source of truth for sensor positions. The wheel axle is assumed to be at the body centre.

### Protocol additions (proposed)

Plain space-delimited ASCII like the rest of the protocol, no JSON. Implementers also update the protocol tables in both repos' `CLAUDE.md` and `Gadget-UI/src/protocol.ts`. New commands must be handled in `CommandHandler::handle` (`lib/Commands/CommandHandler.cpp`) **before** the final `else`, which treats anything unknown as a direction name. Incoming frames are limited to 64 bytes (`MAX_MESSAGE_LENGTH`); outgoing frames are not.

| Message | Direction | Meaning |
|---|---|---|
| `POSE X=<mm> Y=<mm> TH=<mrad> T=<ms> Q=<0\|1>` | robot -> UI | Every 200 ms while a client is ready, and once on connect (with `STATE`). `T` = robot `millis()`. `Q=1` = wheel slip/stall suspected (item 1.5). `RELOC=1` is appended after a reboot that restored a saved pose (item 4.3). |
| `SWEEP_POSE T0=<ms> X0=<mm> Y0=<mm> TH0=<mrad> T1=<ms> X1=<mm> Y1=<mm> TH1=<mrad>` | robot -> UI | Sent immediately before each `MEASURE` frame: pose at sweep start and end, so the UI can interpolate a pose per sensor. The `MEASURE` frame itself is unchanged (backward compatible). |
| `POSE_RESET` | UI -> robot | Zero the pose to (0, 0, 0). |
| `SCAN360 <steps>` | UI -> robot | Spin-scan (phase 5). |
| `GOTO <x> <y>` | UI -> robot | Phase 9 only. |

## Backlog

### Phase 0: Foundations and calibration (~13 pts)

- [x] **0.1 (2, DOC) Housekeeping.** `CLAUDE.md:28` says the PCNT counter is cleared every tick, but the code diffs against `lastRawCount` and only clears at |raw| > 16000. `CLAUDE.md:19` refers to a `STATUS.MD` that is not in the repo. Fix both and add a pointer to this file. *Done when* `CLAUDE.md` matches the code.
- [x] **0.2 (3, FW) Robot constants.** Add `lib/Steering/RobotGeometry.h`: encoder pulses per wheel revolution (turn a wheel by hand one revolution and read the counter), wheel diameter (80 mm nominal, verify), track width (measure), derived pulses-per-mm, and the sensor mounts (copied from the UI geometry, converted to the frame above). *Done when* every constant is documented with how it was measured.
- [x] **0.3 (3, FW) Signed wheel ticks.** `PidController::loop` takes `abs()` of the tick delta, so direction is lost. Expose the signed delta for odometry without changing PID behaviour, and verify the sign per wheel (the motors are mirrored; use a per-wheel sign constant). *Done when* a serial print shows +/+ for `FORWARD`, -/- for `BACKWARD`, and opposite signs for `LEFT`/`RIGHT` spins.
- [x] **0.4 (3, HW+FW) Sensor characterisation.** Log 100 samples per sensor against a flat wall at 100 / 300 / 600 / 1000 mm. Record bias, noise and practical max range per sensor (FRONT_LEFT and BACK_LEFT are known to be noisy at the edge of range). Store a per-sensor offset and a "trusted max range". Do not change the sweep flow (see `SENSOR_FLOW.md`). *Tooling ready:* `CAL <mm> <count>` in the firmware, the "Sensor calibration" panel in Gadget-UI (records a CSV), and `tools/analyze_sensors.py` (prints the table and the C++ arrays); the correction curves are applied to `MEASURE` frames in firmware (`lib/SensorCalibration`), `CAL` frames stay raw. Results below. *Done when* the table is in this file.
- [x] **0.5 (2, FW) Host-side tests.** Add a PlatformIO `native` environment and put the pose math in a hardware-free module so it can be tested in `test/` (currently empty). *Done when* `pio test -e native` runs at least one odometry test.

### Phase 1: Odometry / dead reckoning (~19 pts)

- [ ] **1.1 (5, FW) `Odometry` class.** Accumulate signed ticks and integrate a differential-drive pose (x, y, theta) every control tick using the arc (midpoint) model. Update from `SteeringManager::tickLocked` under its mutex. Handle int16 PCNT wrap and the |raw| > 16000 clear without losing counts. The counter also runs while the robot is stopped, so coasting is counted (good). *Done when* a 1 m straight drive reports 1000 mm +/- 5%.
- [ ] **1.2 (3, FW) `POSE` broadcast.** `POSE` every 200 ms, `POSE` on connect (with `STATE`), `POSE_RESET` command, protocol tables updated. *Done when* the UI log shows `POSE` frames and `POSE_RESET` zeroes them.
- [ ] **1.3 (3, FW) `SWEEP_POSE`.** Snapshot the pose when a sweep starts (`DistanceManager::requestMeasurement` / the Core-0 task) and when it completes, send before `MEASURE`. *Done when* driving shows two different poses per sweep.
- [ ] **1.4 (5, FW+DOC) Odometry calibration.** UMBmark-style: drive a 1 m square (or 1 m out-and-back plus a 360 deg spin) and tune a wheel-diameter scale (distance error) and a track-width scale (rotation error). Record drift before/after. *Done when* a 1 m square closes within ~10 cm and a 360 deg spin within ~5 deg, with numbers in this file.
- [ ] **1.5 (3, FW) Slip/stall guard.** Flag samples where commanded speed is high but encoder delta is about zero (blocked or stuck wheel), freeze pose integration for them, expose as `Q=1`. *Done when* holding a wheel while driving sets `Q=1` and the pose stops advancing.

### Phase 2: Map v1, trace and point cloud in Gadget-UI (~21 pts)

- [ ] **2.1 (3, UI) Protocol.** In `src/protocol.ts` parse `POSE` and `SWEEP_POSE` (all-or-nothing validation, as per the UI `CLAUDE.md`), add `pose$` and pairing logic in `src/socket.service.ts` (a `MEASURE` frame is joined with the preceding `SWEEP_POSE`), specs. *Done when* `protocol.spec.ts` covers valid and malformed frames.
- [ ] **2.2 (5, UI) `map-panel` component.** Standalone, signals, HiDPI canvas with rAF-coalesced repaint (copy the pattern from `robot-preview/`), world-to-screen transform, pan (drag) and zoom (wheel), robot marker with heading, pose trail. Placed next to `<robot-preview>` in `control-panel.html`. *Done when* the marker follows the robot while driving.
- [ ] **2.3 (5, UI) Point cloud.** Project each sensor reading through its mount and the interpolated pose into world coordinates, draw dots coloured by age; OOR readings are skipped. *Done when* driving along a wall gives a roughly straight line of dots.
- [ ] **2.4 (3, UI) Panel controls.** Clear, toggle trail/points/rays, "stationary sweeps only" filter (drop sweeps where the pose moved more than a threshold), min/max range clip, backdrop grid with 1 m scale. *Done when* each control works and is covered by a spec.
- [ ] **2.5 (5, UI) Raw log record and replay.** Record `POSE` / `SWEEP_POSE` / `MEASURE` frames to a downloadable NDJSON file and replay a file into the same streams. This makes every later algorithm testable offline and gives Vitest real fixtures. *Done when* a recorded drive replays into the panel identically.

### Phase 3: Occupancy grid (~26 pts)

- [ ] **3.1 (5, UI) `OccupancyGrid`.** Pure TypeScript (no Angular): sparse chunked log-odds grid, 50 mm cells, grows on demand, `Int8Array` chunks, world<->cell conversion. *Done when* unit tests cover growth, bounds and conversion.
- [ ] **3.2 (5, UI) Ray casting.** Bresenham from sensor origin: cells up to the reading become free, the end cell occupied. OOR marks free only up to the sensor's trusted max range and never marks an occupied cell. Per-sensor weights (lower for the noisy ones). *Done when* a spec with a known wall marks the right cells.
- [ ] **3.3 (5, UI) Beam cone model.** Cast 3-5 rays across the 25 deg cone (free along all of them, occupied at the centre ray with higher weight than the edge rays), minimum-range cutoff, log-odds clamp. *Done when* a wall seen at an angle does not smear into a wedge of occupied cells.
- [ ] **3.4 (3, UI) Rendering.** Unknown = grey, free = light, occupied = dark, drawn from an offscreen canvas that repaints only dirty chunks. *Done when* a large map still repaints within a frame budget.
- [ ] **3.5 (3, UI) Noise filtering.** Occupied only after >= 2 independent hits, decay for isolated hits, ignore sweeps while `Q=1`. *Done when* a single spurious reading leaves no permanent obstacle.
- [ ] **3.6 (5, UI) Tests.** A simulated robot driving a square room with known walls (assert walls occupied, interior free), plus replay of a recorded log (2.5). *Done when* both run in `ng test`.

### Phase 4: Persistence and export (~16 pts)

- [ ] **4.1 (5, UI) Autosave.** Grid, pose and trail to IndexedDB, restored on reload, multiple named maps, "new map" and "clear". *Done when* reloading the page restores the map.
- [ ] **4.2 (3, UI) Export / import.** PNG, JSON (grid + metadata), and the **ROS `map_server` format** (PGM + `map.yaml`, resolution 0.05, origin), plus import of the same. *Done when* the PGM/YAML pair loads in a ROS `map_server` (or at least in an image viewer with a correct YAML).
- [ ] **4.3 (3, FW) Pose across reboots.** Persist the last pose in NVS. After boot, `POSE` carries `RELOC=1` so the UI knows it is only a guess (the robot may have been moved while off); the user places the robot and sends `POSE_RESET`. *Done when* a reboot keeps the pose and the UI shows it as uncertain.
- [ ] **4.4 (5, FW, optional) Robot-side map snapshot.** LittleFS partition (custom partition table; OTA slots must stay) and a coarse grid snapshot (e.g. 100x100 cells x 2 bits = 2.5 KB) streamed to a newly connected client, so a fresh browser gets the map from the robot. *Done when* a second browser shows the map without having driven the robot.

### Phase 5: Pseudo-lidar, spin scan (~13 pts)

- [ ] **5.1 (5, FW) `SCAN360 <steps>`.** Rotate in place by 360/N using odometry: stop, settle ~200 ms, wait for a sweep, tag it with the pose, next step. Runs in the control task, not `loop()`. Aborts on STOP, client disconnect, OTA start or timeout. The 1 s command timeout needs a documented scan-specific rule. All failsafes in `CLAUDE.md` must still hold. *Done when* the robot completes a 24-step spin and every failsafe still stops it mid-scan.
- [ ] **5.2 (3, UI) Scan UI.** `SCAN360` button and progress in the map panel; sweeps feed the grid as normal. *Done when* a scan fills in the map around the robot.
- [ ] **5.3 (5, UI) Polar scan assembly.** Merge the N sweeps into one dense angular scan (~10 x N points sorted by bearing) as a `LaserScan`-like object (`angle_min`, `angle_increment`, `ranges[]`). This is the input the scan matcher in phase 7 needs. *Done when* a 24-step scan in a box room shows a recognisable rectangle.

### Phase 6: Heading from a gyro (~16 pts)

Biggest single accuracy gain: wheel-only heading drifts fastest.

- [ ] **6.1 (3, HW) Pick and wire an IMU.** MPU-6050 or ICM-42688 (gyro only needed), or BNO055 (on-chip fusion), on the existing I2C bus. Check address conflicts with the VL53L0X range (0x30-0x39) and that sweep timing is unaffected: ranging is blocking I2C on Core 0, so use that task or a mutex-guarded bus. *Done when* the IMU is read while sweeps run with no change in sweep time or errors.
- [ ] **6.2 (5, FW) Gyro driver.** Bias calibration at boot (robot stationary ~2 s), yaw integration, note temperature drift. *Done when* a stationary robot's yaw drifts less than ~1 deg/min.
- [ ] **6.3 (5, FW) Fusion.** Take theta from the gyro (complementary filter or a small EKF against odometry theta); position still from the encoders. `POSE` format unchanged. *Done when* a 360 deg spin returns within ~2 deg.
- [ ] **6.4 (3, FW+DOC) Recalibrate.** Re-run 1.4 and record the improved drift. *Done when* before/after numbers are in this file.

### Phase 7: Scan matching and SLAM-lite (~29 pts)

- [ ] **7.1 (8, UI) Correlative scan matcher** (Hector/Karto-style). Around the odometry pose, search a small window in (dx, dy, dtheta) and maximise the summed grid occupancy at the scan endpoints, coarse-to-fine. Runs on dense scans only (5.3, or several accumulated sweeps). *Done when* a deliberately offset pose is pulled back to within one grid cell in a test.
- [ ] **7.2 (5, UI) Correction policy.** Apply a match only above a score threshold and when it beats the odometry pose by a margin; otherwise keep odometry. Show the score in the panel. *Done when* bad matches (featureless corridor) are rejected in a test.
- [ ] **7.3 (8, UI) Particle filter** (gmapping-style, 30-100 particles, each with its own small grid). Motion model + scan likelihood + resampling, in a web worker so the UI stays responsive. *Done when* it holds a stable map over a loop of the room that plain odometry smears.
- [ ] **7.4 (5, UI) Localisation in a saved map** (AMCL-lite). Load a map (4.1), spread particles around the pose guess, converge by driving or spinning. *Done when* a robot placed in a known room recovers its pose after a spin-scan.
- [ ] **7.5 (3, UI) Metrics.** Drift versus ground truth (taped floor marks) and map quality per algorithm (wall thickness, loop closure error). *Done when* the numbers are in this file.

### Phase 8: Loop closure (stretch, ~18 pts)

- [ ] **8.1 (5, UI) Keyframes.** Store pose + scan every N cm or degrees. *Done when* keyframes are saved and replayable.
- [ ] **8.2 (5, UI) Loop-closure detection.** Re-match a new keyframe against old ones within a radius. *Done when* returning to the start produces a loop edge.
- [ ] **8.3 (8, UI) Pose-graph optimisation.** Gauss-Newton on (x, y, theta) nodes with odometry and loop edges, then rebuild the grid from the corrected keyframes. *Done when* a loop around a room closes visibly better than without it.

### Phase 9: Autonomy using the map (stretch, ~31 pts)

- [ ] **9.1 (5, FW) Reactive safety.** Firmware-side stop/slow when FRONT/BACK is closer than a threshold during autonomous moves, never relying on the UI; document it as a new failsafe in `CLAUDE.md`. *Done when* the robot stops before a wall with the UI disconnected from the decision.
- [ ] **9.2 (5, UI) Wall following.** A controller in the UI that sends direction commands at the usual 500 ms refresh. *Done when* the robot follows a wall for a full lap.
- [ ] **9.3 (8, UI) Frontier exploration** over the occupancy grid. *Done when* it explores a small room until no frontiers remain.
- [ ] **9.4 (5, UI) A\* planner** on the inflated grid, `GOTO x y` through the same command refresh loop. *Done when* the robot drives around an obstacle to a goal.
- [ ] **9.5 (8, FW, stretch) On-board mapping.** Port the grid and controller to the firmware so the robot maps without a browser attached. *Done when* it maps a room with no client connected.

### Phase 10: ROS bridge (stretch, ~13 pts)

- [ ] **10.1 (8, TOOL) Bridge.** Small Node/Python process: WebSocket -> ROS 2 topics `/odom`, `/tf`, `/scan` (from the spin-scan) so `slam_toolbox` can run against the robot as a baseline. *Done when* `slam_toolbox` produces a map from a live spin-scan.
- [ ] **10.2 (5, DOC) Comparison.** Compare the browser SLAM map with the `slam_toolbox` map on the same raw log (2.5). *Done when* the comparison is written up here.

Total: ~215 points (phases 0-4, the MVP, ~95).

## Sensor characterisation results (item 0.4)

Recorded 2026-10-21 with `CAL` (100 sweeps per sensor and distance, flat wall, robot still, distance from a ruler). Bias = mean raw reading minus true distance, in mm; std at 1000 mm in mm.

| Sensor | Bias @100 | @300 | @600 | @1000 | Std @1000 |
|---|---|---|---|---|---|
| FRONT | +18.3 | +41.8 | +53.5 | +62.6 | 6.8 |
| FRONT_LEFT_45 | +39.2 | +53.3 | +57.7 | +78.2 | 4.5 |
| FRONT_LEFT | +11.4 | +30.3 | +39.2 | +49.3 | 6.5 |
| FRONT_RIGHT_45 | +17.8 | +38.1 | +45.8 | +61.8 | 5.8 |
| FRONT_RIGHT | +14.5 | +28.5 | +34.2 | +34.7 | 6.6 |
| BACK | +30.7 | +39.9 | +45.7 | +58.7 | 5.3 |
| BACK_LEFT_45 | +21.1 | +42.0 | +50.2 | +52.8 | 5.5 |
| BACK_LEFT | +29.2 | +55.6 | +72.5 | +87.3 | 6.8 |
| BACK_RIGHT_45 | +21.5 | +22.5 | +10.8 | -62.5 | 7.1 |
| BACK_RIGHT | +16.7 | +21.0 | -2.0 | -86.4 | 7.9 |

Findings:

- **Noise is low** (std 1.4-2.3 mm at 100-300 mm, 3-4 mm at 600 mm, 4.5-8 mm at 1000 mm) and every sensor passes the trusted-range rule up to 1000 mm, the farthest distance tested. Readings beyond 1000 mm were not tested; the sensors usually stay in range further.
- **The bias is not a constant offset.** Most sensors read long, by 11-39 mm at 100 mm growing to 35-87 mm at 1000 mm; the two BACK_RIGHT sensors read short at long range (-62 and -86 mm at 1000 mm). A flat offset would leave 20-60 mm of error, so the firmware applies a per-sensor piecewise-linear curve through the four measured points (`lib/SensorCalibration/SensorCalibrationData.h`, regenerate with `tools/analyze_sensors.py`). Each reference point is exact; between points it is an interpolation from only four measurements.
- **BACK_RIGHT / BACK_RIGHT_45 slope below 1 is unexplained.** It may be a real per-sensor gain error or a setup issue (robot not square to the wall, wrong zero point for that run). Re-measure one of them before trusting its curve beyond 600 mm.
- **Last sample of every run was a bogus all-OOR sweep** (sample 100 of 100 in all 40 runs: every sensor out of range). It does not affect means or std (OOR is excluded), only the OOR rate (1%). Cause not yet identified (to check: does `CAL 300 5` also end with an all-OOR sample 5?). The first 99 samples of each run are normal.
- FRONT_LEFT and BACK_LEFT, known to be noisy at the edge of their range, were no noisier than the others up to 1000 mm here.

## Risks and limits

- ToF range and sparse beams; dark, glossy, glass and sunlit surfaces read badly or as OOR.
- A ~390 ms sweep while moving smears readings (hence stop-and-go first, `SWEEP_POSE` interpolation later). Do not try to speed the sweep up by ranging concurrently (see `SENSOR_FLOW.md`).
- Encoder slip on carpet and in turns; heading drifts fastest (phase 6).
- Wi-Fi drops mid-scan: every failsafe stays in firmware (stop on disconnect/OTA/1 s timeout, loop watchdog).
- New wire commands must fit the 64-byte incoming limit and be handled before the direction-name fallback.
- The UI sensor geometry has not been verified against the real robot; do that in 0.2.
