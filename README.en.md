# Contato (hardware)
[![pt-br](https://img.shields.io/badge/lang-pt--br-green.svg)](README.md)

Embedded firmware for the **Contato** device, developed by the Dance Course at Universidade Federal do Rio de Janeiro in partnership with the UFRJ Technological Park.

The system is built around **ESP32 DEVKIT V1** modules. Each **Equip** (wearable) carries an **MPU6050 IMU** (gyroscope + accelerometer, 6 degrees of freedom) and a capacitive touch sensor, and sends motion data over **ESP-NOW** to a **Base** connected to the computer by USB. The **[contato_cli](../contato_cli)** software reads the Base over the serial port and converts the data into **MIDI**.

## Contents

* Architecture
* How it works
* Protocol
* Structure
* Build and upload
* Over-the-air update (OTA)
* Calibration
* Hardware

## Architecture

```text
Equip (ESP32 + MPU6050)
        ↓ ESP-NOW (channel 11)
Base (ESP32 USB)
        ↓ Serial (115200 baud)
contato_cli
        ↓ MIDI
DAW / Virtual Instruments
```

There is also the **Bridge** (`ponte.cpp`), an ESP32 connected to the PC and used only to send firmware to Equips and Bases over the air (OTA), without a cable.

## How it works

**Equip** (`equip_N.cpp`)

- Uses the MPU6050 DMP to get the quaternion and computes the roll angle (in degrees) and the linear acceleration on the X axis.
- Reads the capacitive sensor (pin `T3`); a touch is detected when the reading falls below `touch_sensitivity`.
- Only transmits after receiving the control command from the Base (`ativo = 1`). The blue LED (GPIO 2) lights up while transmission is active and the sensor is touched.
- The MPU6050 calibration offsets are hard-coded in each `equip_N.cpp`.

**Base** (`base_N.cpp`)

- Accepts packets only from the Equip whose MAC is set in `macTransmissor` and drops the rest.
- Answers commands received over serial:

| Command | Action |
|---|---|
| `START` | Enables serial output and tells the Equip to start transmitting (resent every 2 s) |
| `STOP` | Disables serial output and tells the Equip to stop |
| `ID?` | Replies `ID/<BASE_ID>` |

- Each reading is written to serial as a line `id/gyro/accel/touch`.

**TDMA master** (`TDMA.cpp`, also used as `src/main.cpp`)

- Broadcasts a *beacon* with the current slot, cycling through the Equip slots (`NUM_EQUIPS = 6`, `SLOT_US = 1500` µs).
- Each Equip transmits only when the beacon shows its own `MEU_SLOT`, avoiding collisions between several Equips on the same channel.

## Protocol

Equip-to-Base message (`message_t`):

| Field | Type | Description |
|---|---|---|
| `id` | `uint8_t` | Equip ID |
| `gyro` | `int16_t` | Roll angle, in degrees |
| `accel` | `int32_t` | Linear acceleration on the X axis |
| `touch` | `uint8_t` | 1 if the capacitive sensor is touched |

Other ESP-NOW packets (channel 11, 1 Mbps rate, no encryption):

- **Beacon** (`beacon_t`): `slot_atual` + `timestamp`, sent by the TDMA master.
- **Control** (`controle_t`): `ativo` (0/1), sent by the Base to the Equip.
- **OTA** (`ota_pacote_t`): `INICIO` (`0xAA`), `DADO` (`0xBB`) and `FIM` (`0xCC`) packets, with up to 230 data bytes each.

## 📁 Structure

```text
contato_hardware/
├── arduino/              # legacy implementations (ESP-NOW P2P, no longer maintained)
└── platformio/           # active project
    ├── platformio.ini
    ├── src/
    │   └── main.cpp      # copy of the chosen script (default: TDMA.cpp)
    ├── include/
    │   ├── config.h      # constants from the previous BLE version (unused by current scripts)
    │   ├── types.h       # data structs
    │   └── ota_receptor.h  # OTA receiver used by Equips, Bases and the TDMA master
    ├── lib/              # MPU6050 and MadgwickAHRS
    ├── scripts/          # firmware flashed to the devices
    │   ├── equip_1..6.cpp
    │   ├── base_1..6.cpp
    │   ├── ponte.cpp
    │   ├── TDMA.cpp
    │   ├── monitor.cpp
    │   ├── B/            # variants (equip_1B, equip_5B, base_1B, base_5B)
    │   └── upload_script.py
    └── util/             # calibration, benchmarks, templates and MAC table
```

## Build and upload

Requires [PlatformIO](https://platformio.org/) (CLI or VSCode extension).

The firmware to flash is chosen with the `SCRIPT` environment variable. Before the build, `scripts/upload_script.py` looks for `<SCRIPT>.cpp` in `util/`, `scripts/` and `scripts/B/` and copies it to `src/main.cpp`.

```powershell
cd platformio

# Equip 1
$env:SCRIPT="equip_1"; pio run --target upload -e esp32doit-devkit-v1

# Base 1
$env:SCRIPT="base_1"; pio run --target upload -e esp32doit-devkit-v1

# TDMA master
$env:SCRIPT="TDMA"; pio run --target upload -e esp32doit-devkit-v1

# Serial monitor (115200 baud)
pio device monitor --speed 115200
```

If `SCRIPT` is not set (for example, an IDE build), `src/main.cpp` is left as it is.

> **Warning:** `upload_script.py` **overwrites** `src/main.cpp` on every build with `SCRIPT` set.

Each Equip and Base has its own file (`equip_N.cpp`, `base_N.cpp`), differing in ID, slot and MAC addresses. To create a new device, copy `util/equip_modelo.cpp` or `util/base_modelo.cpp`. The Base MACs are listed in `platformio/util/README.md`.

## Over-the-air update (OTA)

The firmware includes `ota_receptor.h` and can be updated without a USB cable. `ponte.cpp` receives the binary from the PC and sends it over ESP-NOW to the target device, with a status reply at each step.

The flow is normally run through `contato_cli` (`contato ota`, `contato update-bases`), which compiles the script and sends it. For this, `ponte.cpp` must be flashed on the ESP32 connected to the PC.

## Calibration

`util/calibrate.cpp` computes the MPU6050 offsets. The `contato calibrate` command of `contato_cli` runs it on the Equip and prints the resulting offsets. Then copy the values into the `setXAccelOffset`, `setYAccelOffset`, `setZAccelOffset`, `setXGyroOffset`, `setYGyroOffset` and `setZGyroOffset` calls of the matching `equip_N.cpp`.

## Hardware

| Component | Description |
|---|---|
| ESP32 DEVKIT V1 | Microcontroller (Equip, Base, Bridge and TDMA master) |
| MPU6050 | 6-axis IMU (I2C, 400 kHz), present on the Equips |
| GPIO 2 | Blue LED for transmission indication |
| GPIO T3 | Capacitive touch sensor |
