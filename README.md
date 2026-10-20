# Gadget-firmware

Firmware for **Gadget**, a small 2-wheeled exploring robot built on an Arduino Nano ESP32. It is driven from the browser by the companion app, [Gadget-UI](https://github.com/Filip1159/Gadget-UI)

## Hardware

- Arduino Nano ESP32 (ESP32-S3, dual core)
- 2 DC gear motors with encoders
- 10 VL53L0X time-of-flight distance sensors (front, back, both sides and the four 45-degree corners)

## Features

- Per-wheel PID speed control with encoder feedback, running in its own FreeRTOS task
- Drive in 8 directions plus stop, with adjustable speed and live-tunable PID gains
- 10-sensor distance sweeps, one-shot or continuous, sent as a single frame
- WebSocket control server on port 88 (plain-text protocol, optional token)
- Over-the-air updates (`gadget.local`, optional password)
- Failsafes: the robot stops when a client disconnects, an update starts, no command arrives for 1s, or the control task stalls

WiFi credentials go in `lib/ConnectionManager/secrets.h` (copy `secrets.h.example`; it is gitignored).

#

```bash
pio run -t upload                          # build and flash over USB
```

```bash
pio run -e ota -t upload                   # flash over WiFi (gadget.local)
```

```bash
pio device monitor                         # serial monitor, 115200 baud
```
