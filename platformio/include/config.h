#pragma once

#include <cstdint>

constexpr int LED_PIN = 2;
constexpr int TOUCH_PIN = T3;
constexpr int TOUCH_THRESHOLD = 30;
constexpr long I2C_CLOCK_HZ = 400000;

constexpr char DEVICE_NAME[] = "Contato";

constexpr char MAIN_SERVICE_UUID[] = "03B80E5A-EDE8-4B33-A751-6CE34EC4C700";
constexpr char MIDI_CHAR_UUID[] = "7772E5DB-3868-4112-A1A9-F2669D106BF3";
constexpr char SECTIONS_CHAR_UUID[] = "251beea3-1c81-454f-a9dd-8561ec692ded";
constexpr char ACCEL_SENS_CHAR_UUID[] = "c7f2b2e2-1a2b-4c3d-9f0a-123456abcdef";
constexpr char STATUS_CHAR_UUID[] = "f8d968fe-99d7-46c4-a61c-f38093af6ec8";
constexpr char DIR_CHAR_UUID[] = "a1b2c3d4-0001-4b33-a751-6ce34ec4c701";
constexpr char CALIBRATE_CHAR_UUID[] = "b4d0c9f8-3b9a-4a4e-93f2-2a8c9f5ee7a2";
constexpr char TILT_CHAR_UUID[] = "d2e3f4a5-0002-4b33-a751-6ce34ec4c702";
constexpr char LEGATO_CHAR_UUID[] = "e3f4a5b6-0003-4b33-a751-6ce34ec4c703";

constexpr char PREF_NAMESPACE[] = "mpu";
constexpr char PREF_KEY_OFFSETS[] = "offs";
constexpr char PREF_KEY_SECTIONS[] = "sections";
constexpr char PREF_KEY_ACCEL[] = "sens";
constexpr char PREF_KEY_DIRECTION[] = "dir";
constexpr char PREF_KEY_TILT[] = "tilt";
constexpr char PREF_KEY_LEGATO[] = "legato";

constexpr unsigned long STATUS_INTERVAL_MS = 3;
constexpr unsigned long ACCEL_DEBOUNCE_MS = 2000;

constexpr float GYRO_MAX_DEG = 89.0f;
constexpr float TILT_MAX_DEG = 89.0f;
constexpr int PITCH_BEND_DEADZONE_DEG = 10;
constexpr int PITCH_BEND_MAX_DEG = 45;
constexpr int PITCH_BEND_CENTER = 8192;
constexpr int PITCH_BEND_MAX_VALUE = 16383;

constexpr int DEFAULT_SECTION_COUNT = 6;
constexpr uint8_t DEFAULT_NOTE = 60;
constexpr uint8_t DEFAULT_DIRECTION = 1;
constexpr uint8_t DEFAULT_TILT_ENABLED = 0;
constexpr uint8_t DEFAULT_LEGATO_ENABLED = 0;
constexpr int32_t DEFAULT_ACCEL_THRESHOLD = 10000;

constexpr int32_t MIN_ACCEL_THRESHOLD = 100;
constexpr int32_t MAX_ACCEL_THRESHOLD = 32767;

constexpr uint8_t NOTE_CHANNEL = 1;
constexpr uint8_t PERC_NOTE = 36;
constexpr uint8_t PERC_CHANNEL = 8;

constexpr uint8_t STATUS_STATE_IDLE = 0;
constexpr uint8_t STATUS_STATE_CALIBRATING = 1;
