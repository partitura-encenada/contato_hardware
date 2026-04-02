#pragma once

#include <Arduino.h>
#include <string>

#include "config.h"

struct __attribute__((packed)) StatusPacket {
    uint8_t state;
    uint8_t touch;
    int16_t gyro_x;
    int16_t accel_x;
    int16_t tilt;
};

struct MPUOffsets {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

struct PersistentConfig {
    std::string sections = std::string(DEFAULT_SECTION_COUNT, static_cast<char>(DEFAULT_NOTE));
    int32_t accel_threshold = DEFAULT_ACCEL_THRESHOLD;
    bool flip_gyro = DEFAULT_DIRECTION != 0;
    bool tilt_enabled = DEFAULT_TILT_ENABLED != 0;
    bool legato_enabled = DEFAULT_LEGATO_ENABLED != 0;
};

struct MotionSample {
    int gyro = 0;
    int accel = 0;
    int tilt = 0;
    bool touch = false;
    byte note = DEFAULT_NOTE;
};

struct RuntimeState {
    bool dmp_ready = false;
    bool touch_pressed = false;
    bool note_playing = false;
    bool percussion_playing = false;
    bool calibration_requested = false;
    uint8_t status = STATUS_STATE_IDLE;
    unsigned long last_status_ms = 0;
    unsigned long last_percussion_ms = 0;
    byte last_note = DEFAULT_NOTE;
    int last_pitch_bend = PITCH_BEND_CENTER;
};
