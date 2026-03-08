# Contato (hardware)
[![pt-br](https://img.shields.io/badge/lang-pt--br-green.svg)](README.md)

Embedded firmware for the **Contato** device, developed by the Dance Course at Universidade Federal do Rio de Janeiro in partnership with the UFRJ Technological Park.

The device is built around an **ESP32 DEVKIT V1** and an **MPU6050 IMU** (6-axis gyroscope + accelerometer). It captures the user's motion and transmits **MIDI messages over Bluetooth Low Energy (BLE)**, enabling musical interaction driven by body movement and capacitive touch.

## How it works

- The gyroscope roll angle (±90°) selects a MIDI note from a configurable set of sections.
- Touching the capacitive sensor triggers Note-On/Note-Off for the currently selected note. Holding touch while moving across sections glides to the new note.
- Acceleration spikes on the X-axis trigger a short percussion note (MIDI 36, channel 8).
- All configuration (notes, sensitivity, direction, calibration offsets) is persisted in the ESP32's non-volatile storage (NVS) and can be updated by a BLE client.

## Structure

```
contato_hardware/
├── arduino/          # legacy implementations (ESP-NOW P2P, no longer maintained)
└── platformio/       # active project
    ├── platformio.ini
    ├── src/
    │   └── main.cpp  # main firmware
    └── include/
        ├── config.h  # pins, BLE UUIDs, MIDI and timing constants
        └── types.h   # data structs (StatusPacket, MPUOffsets)
```

## Build and upload

Requires [PlatformIO](https://platformio.org/) (CLI or VSCode extension).

```bash
cd platformio

# Build
platformio run -e esp32doit-devkit-v1

# Build and flash to device
platformio run --target upload -e esp32doit-devkit-v1

# Serial monitor (115200 baud)
platformio device monitor --speed 115200
```

## Hardware

| Component | Description |
|---|---|
| ESP32 DEVKIT V1 | Main microcontroller |
| MPU6050 | 6-axis IMU (I2C, 400 kHz) — SDA=21, SCL=22 |
| GPIO 2 | LED indicator (on = client connected or calibrating) |
| GPIO T3 | Capacitive touch sensor |

## Calibration

On first boot with no saved offsets, the device calibrates the MPU6050 automatically and persists the offsets to NVS. Subsequent recalibrations can be triggered from the "Calibrar" button in the BLE client (`contato_gui`).
