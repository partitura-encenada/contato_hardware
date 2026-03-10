# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Build Commands

All commands run from the `platformio/` directory:

```bash
# Build
platformio run -e esp32doit-devkit-v1

# Build and flash
platformio run --target upload -e esp32doit-devkit-v1

# Serial monitor (115200 baud)
platformio device monitor --speed 115200

# Clean build artifacts
platformio run --target clean
```

PlatformIO can also be used via the VSCode extension.

## Project Layout

`platformio/` is the active firmware. `arduino/` is legacy (ESP-NOW P2P) and unmaintained.

```
platformio/
├── platformio.ini        # espressif32@6.11.0, esp32doit-devkit-v1
├── src/main.cpp          # all firmware logic
└── include/
    ├── config.h          # pins, BLE UUIDs, timing constants, MIDI defaults
    └── types.h           # StatusPacket, MPUOffsets structs
```

## Architecture

Contato is a BLE MIDI peripheral. It reads motion from an MPU6050 IMU and maps it to MIDI events transmitted via BLE to a connected client (`contato_gui`).

### Data Flow (loop)

1. MPU6050 DMP outputs quaternion at ~333 Hz → converted to Euler angles
2. Roll angle (±90°, clamped) maps to a section index into the notes array
3. Capacitive touch (GPIO T3) → Note-On on press, Note-Off on release (or sustained in legato mode until next touch/accel); glides to new note if section changes while held
4. Linear X-axis acceleration above `accelThreshold` → short percussion note (Note 36, ch 8) lasting `PERC_NOTE_MS`, gated by `ACCEL_DEBOUNCE_MS`; stops any legato-sustained note first
5. Forearm tilt angle (pronation/supination via Y-axis gravity) → Pitch Bend on ch 1, ±10° dead zone, when `tiltEnabled`
6. STATUS_CHAR notified with gyro angle, raw accel, touch state, and tilt angle

### Calibration

`calibrateAndSave()`:
- Sets LED HIGH to indicate activity
- Disables DMP (required so raw sensor reads during calibration are clean)
- Runs `CalibrateGyro(6)` and `CalibrateAccel(6)` from the MPU6050 library
- Re-enables DMP
- Saves offsets to NVS

Triggered two ways:
- **First boot** (no offsets in NVS): runs inside `setup()` before BLE starts
- **BLE write** to `CALIBRATE_CHAR`: sets `volatile bool calibrate_requested` flag, consumed at the top of `loop()` — never called from a BLE callback to avoid blocking the NimBLE task

### BLE GATT Contract

Service UUID: `03B80E5A-EDE8-4B33-A751-6CE34EC4C700`

| Characteristic | Properties | Wire format | Description |
|---|---|---|---|
| MIDI_CHAR | READ, WRITE_NR, NOTIFY | 5-byte Apple BLE MIDI | Note-On/Note-Off with 13-bit timestamp |
| STATUS_CHAR | NOTIFY | `<BBhhh`: uint8 state, uint8 touch, int16 gyro, int16 accel, int16 tilt | Sensor state ~50 Hz |
| SECTIONS_CHAR | READ, WRITE | N bytes (1–8), each a MIDI note number | Note per gyro section |
| ACCEL_SENS_CHAR | READ, WRITE | int16 on write, int32 on read | Percussion sensitivity threshold |
| DIR_CHAR | READ, WRITE | uint8 (0 = normal, 1 = flip) | Gyro direction inversion |
| CALIBRATE_CHAR | WRITE | any byte | Triggers MPU6050 calibration |
| TILT_CHAR | READ, WRITE | uint8 (0 = off, 1 = on) | Enable pitch bend via forearm tilt |
| LEGATO_CHAR | READ, WRITE | uint8 (0 = off, 1 = on) | Enable legato mode |

### NVS (Preferences namespace `"mpu"`)

| Key | Type | Description |
|---|---|---|
| `"offs"` | `MPUOffsets` struct (12 bytes) | Calibration offsets |
| `"sections"` | bytes | MIDI note array (up to 8 bytes) |
| `"sens"` | int32 | Acceleration threshold |
| `"dir"` | uint8 | Direction inversion flag |
| `"tilt_en"` | uint8 | Pitch bend enabled flag |
| `"legato"` | uint8 | Legato mode enabled flag |

Use `prefs.isKey()` before `prefs.getBytes()` to avoid NVS `NOT_FOUND` log errors on first boot.

### Hardware

| Component | Detail |
|---|---|
| MPU6050 | SDA=21, SCL=22 (ESP32 default I2C), 400 kHz |
| LED | GPIO 2 — HIGH when client connected, HIGH during calibration |
| Touch | GPIO T3, capacitive, threshold < 30 |

## Dependencies

- `jrowberg/I2Cdevlib-MPU6050` — MPU6050 + DMP driver (MotionApps 2.0)
- `h2zero/NimBLE-Arduino` — Lightweight BLE stack for ESP32

## Coding Conventions

- No exception handling, no defensive fallbacks, no input validation on BLE writes
- No bounds-checking beyond what is logically necessary (e.g. section index clamp)
- No helper wrappers around single operations
- Serial output at 115200 baud is the only debugging tool
- Active branch: `claude`; stable branch: `main`
