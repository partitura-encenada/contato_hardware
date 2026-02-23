# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Contato** is an ESP32 firmware for a wearable motion-to-MIDI device developed for the Dance Course at UFRJ. It reads 6-axis IMU data from an MPU6050, maps gyro roll angle and touch input to MIDI notes, and transmits MIDI messages over BLE using the standard MIDI over BLE protocol (Apple/MIDI Association spec).

## Build Commands

All commands should be run from the `platformio/` directory:

```bash
# Build firmware
platformio run -e esp32doit-devkit-v1

# Build and flash to device
platformio run --target upload -e esp32doit-devkit-v1

# Open serial monitor (115200 baud)
platformio device monitor --speed 115200

# Clean build artifacts
platformio run --target clean
```

PlatformIO can also be invoked via the VSCode PlatformIO extension (recommended for development).

## Architecture

### Active Code
The `platformio/` directory contains the active firmware. The `arduino/` directory is legacy code (ESP-NOW P2P implementations) and is not actively developed.

### Key Files
- `platformio/src/main.cpp` — All firmware logic (~380 lines)
- `platformio/include/config.h` — Pin assignments, BLE UUIDs, MIDI defaults, timing constants
- `platformio/include/types.h` — `StatusPacket` and `MPUOffsets` structs

### BLE Service Structure
The device exposes a single BLE service (`03B80E5A...`) with these characteristics:
- `MIDI_CHAR` — Sends MIDI Note-On/Note-Off messages with 13-bit BLE timestamps
- `STATUS_CHAR` — Notifies clients with gyro angle, acceleration, and touch state
- `SECTIONS_CHAR` — R/W array of MIDI notes mapped to gyro sections
- `ACCEL_SENS_CHAR` — R/W percussion sensitivity threshold
- `DIR_CHAR` — R/W gyro direction inversion flag
- `CALIBRATE_CHAR` — Write-only trigger for MPU6050 calibration

### Main Loop Data Flow
1. MPU6050 DMP outputs quaternion → converted to Euler angles
2. Roll angle (±89°) maps to a note index in the `sections[]` array (default: 6 equal sections)
3. Touch sensor (GPIO T3, capacitive) triggers Note-On for the current section's note
4. Linear acceleration on X-axis above `accelSens` threshold triggers a percussion note (ch. 8)
5. Status is broadcast via `STATUS_CHAR` notify to connected clients

### Persistent Storage
Uses the Arduino `Preferences` library (ESP32 NVS), namespace `"mpu"`:
- `"offs"` — MPU6050 calibration offsets (`MPUOffsets` struct)
- `"sections"` — MIDI note array
- `"sens"` — Percussion acceleration threshold
- `"dir"` — Gyro direction inversion flag

### Dependencies (platformio.ini)
- `jrowberg/I2Cdevlib-MPU6050` — MPU6050 + DMP driver
- `h2zero/NimBLE-Arduino` — Lightweight BLE stack (replaced the older BLE-MIDI library)

## Development Notes

- The `claude` branch is the active development branch; `main` is stable.
- BLE advertising must be restarted after client disconnect — this is handled in `ServerCallbacks::onDisconnect`.
- The MPU6050 DMP runs at a fixed FIFO rate; the main loop polls every 3ms (`SENSOR_INTERVAL_MS` in config.h).
- Calibration takes several seconds and blocks the loop; the LED blinks to indicate it is running.
- Serial output at 115200 baud is the primary debugging tool — use `platformio device monitor`.
