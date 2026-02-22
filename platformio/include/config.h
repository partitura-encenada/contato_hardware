#pragma once

// ─── Hardware ─────────────────────────────────────────────────────────────────
#define LED_PIN        2
#define TOUCH_PIN      T3
#define TOUCH_THRESHOLD 30   // capacitive reading below this = touched
#define I2C_CLOCK_HZ   400000

// ─── BLE ─────────────────────────────────────────────────────────────────────
static const char *DEVICE_NAME         = "Contato";

// Standard BLE MIDI service and characteristic UUIDs (Apple / MIDI Association spec).
// Using these lets any MIDI app discover the device as a MIDI peripheral.
static const char *MAIN_SERVICE_UUID   = "03B80E5A-EDE8-4B33-A751-6CE34EC4C700";
static const char *MIDI_CHAR_UUID      = "7772E5DB-3868-4112-A1A9-F2669D106BF3";

// Custom sensor/control characteristics, co-hosted in the same service.
static const char *SECTIONS_CHAR_UUID  = "251beea3-1c81-454f-a9dd-8561ec692ded";
static const char *ACCEL_SENS_CHAR_UUID= "c7f2b2e2-1a2b-4c3d-9f0a-123456abcdef";
static const char *STATUS_CHAR_UUID    = "f8d968fe-99d7-46c4-a61c-f38093af6ec8";
static const char *DIR_CHAR_UUID       = "a1b2c3d4-0001-4b33-a751-6ce34ec4c701";
static const char *CALIBRATE_CHAR_UUID = "b4d0c9f8-3b9a-4a4e-93f2-2a8c9f5ee7a2";

// ─── NVS keys ────────────────────────────────────────────────────────────────
static const char *PREF_NAMESPACE    = "mpu";
static const char *PREF_KEY_OFFS     = "offs";
static const char *PREF_KEY_SECTIONS = "sections";
static const char *PREF_KEY_SENS     = "sens";
static const char *PREF_KEY_DIR      = "dir";

// ─── Timing ──────────────────────────────────────────────────────────────────
// How often sensor data is sampled and sent via BLE notify (~333 Hz).
static const unsigned long STATUS_INTERVAL_MS = 3;

// Minimum time between accelerometer percussion events.
// Prevents rapid re-triggers from a single shake gesture.
static const unsigned long ACCEL_DEBOUNCE_MS = 2000;

// ─── Sensor ──────────────────────────────────────────────────────────────────
// Roll angle is clamped to ±GYRO_MAX_DEG to avoid near-vertical instability.
static const float GYRO_MAX_DEG = 89.0f;

// ─── Music defaults ───────────────────────────────────────────────────────────
// Used when no note sections have been saved to NVS yet.
static const int     DEFAULT_SECTION_COUNT    = 6;
static const uint8_t DEFAULT_NOTE             = 60;  // Middle C

// Percussion note: MIDI 36 = Bass Drum 1, channel 8 = GM percussion channel.
static const uint8_t PERC_NOTE    = 36;
static const uint8_t PERC_CHANNEL = 8;

// Default acceleration threshold (raw IMU X units ÷ 10) for percussion trigger.
static const int32_t DEFAULT_ACCEL_THRESHOLD = 10000;

// BLE-configurable range for the acceleration threshold.
static const int32_t MIN_ACCEL_THRESHOLD = 100;
static const int32_t MAX_ACCEL_THRESHOLD = 32767;
