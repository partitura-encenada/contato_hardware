#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <Wire.h>

#include <string>
#include <vector>

#include "MPU6050_6Axis_MotionApps20.h"
#include "config.h"
#include "types.h"

Preferences preferences;
MPU6050 mpu;

NimBLEServer *server_handle = nullptr;
NimBLECharacteristic *midi_characteristic = nullptr;
NimBLECharacteristic *sections_characteristic = nullptr;
NimBLECharacteristic *status_characteristic = nullptr;
NimBLECharacteristic *accel_characteristic = nullptr;
NimBLECharacteristic *direction_characteristic = nullptr;
NimBLECharacteristic *tilt_characteristic = nullptr;
NimBLECharacteristic *legato_characteristic = nullptr;
NimBLECharacteristic *calibrate_characteristic = nullptr;

PersistentConfig config;
RuntimeState runtime_state;
StatusPacket status_packet;
MPUOffsets saved_offsets;

uint8_t fifo_buffer[64];
Quaternion quaternion;
VectorInt16 raw_accel;
VectorInt16 linear_accel;
VectorFloat gravity;
float yaw_pitch_roll[3];


static float clamp_float(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int clamp_int(int value, int minimum, int maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void save_u8(const char *key, uint8_t value) {
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putUChar(key, value);
    preferences.end();
}

static void save_i32(const char *key, int32_t value) {
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putInt(key, value);
    preferences.end();
}

static void save_blob(const char *key, const void *data, size_t size) {
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putBytes(key, data, size);
    preferences.end();
}

static void send_midi_message(uint8_t status, uint8_t data1, uint8_t data2) {
    if (server_handle->getConnectedCount() == 0) {
        return;
    }

    const uint16_t timestamp = static_cast<uint16_t>(millis());
    uint8_t packet[5] = {
        static_cast<uint8_t>(0x80 | ((timestamp >> 7) & 0x3F)),
        static_cast<uint8_t>(0x80 | (timestamp & 0x7F)),
        status,
        data1,
        data2,
    };

    midi_characteristic->setValue(packet, sizeof(packet));
    midi_characteristic->notify();
}

static void play_note(byte note, uint8_t channel = NOTE_CHANNEL) {
    runtime_state.last_note = note;
    send_midi_message(0x90 | ((channel - 1) & 0x0F), note, 80);
}

static void stop_note(byte note, uint8_t channel = NOTE_CHANNEL) {
    runtime_state.last_note = note;
    send_midi_message(0x80 | ((channel - 1) & 0x0F), note, 80);
}

static void send_pitch_bend(int value) {
    value = clamp_int(value, 0, PITCH_BEND_MAX_VALUE);
    send_midi_message(
        0xE0 | ((NOTE_CHANNEL - 1) & 0x0F),
        value & 0x7F,
        (value >> 7) & 0x7F
    );
}

static void apply_offsets_to_mpu() {
    mpu.setXAccelOffset(saved_offsets.accel_x);
    mpu.setYAccelOffset(saved_offsets.accel_y);
    mpu.setZAccelOffset(saved_offsets.accel_z);
    mpu.setXGyroOffset(saved_offsets.gyro_x);
    mpu.setYGyroOffset(saved_offsets.gyro_y);
    mpu.setZGyroOffset(saved_offsets.gyro_z);
}

static void read_persistent_config() {
    preferences.begin(PREF_NAMESPACE, true);

    preferences.getBytes(PREF_KEY_OFFSETS, &saved_offsets, sizeof(MPUOffsets));
    config.accel_threshold = preferences.getInt(PREF_KEY_ACCEL, DEFAULT_ACCEL_THRESHOLD);
    config.flip_gyro = preferences.getUChar(PREF_KEY_DIRECTION, DEFAULT_DIRECTION) != 0;
    config.tilt_enabled = preferences.getUChar(PREF_KEY_TILT, DEFAULT_TILT_ENABLED) != 0;
    config.legato_enabled = preferences.getUChar(PREF_KEY_LEGATO, DEFAULT_LEGATO_ENABLED) != 0;

    const size_t sections_size = preferences.getBytes(PREF_KEY_SECTIONS, nullptr, 0);
    if (sections_size > 0) {
        std::vector<char> buffer(sections_size);
        preferences.getBytes(PREF_KEY_SECTIONS, buffer.data(), sections_size);
        config.sections.assign(buffer.begin(), buffer.end());
    }

    preferences.end();
    apply_offsets_to_mpu();
}

static void publish_config_to_ble() {
    sections_characteristic->setValue(config.sections);

    const int16_t accel_threshold = static_cast<int16_t>(config.accel_threshold);
    accel_characteristic->setValue(reinterpret_cast<const uint8_t *>(&accel_threshold), sizeof(accel_threshold));

    const uint8_t direction = config.flip_gyro ? 1 : 0;
    direction_characteristic->setValue(&direction, sizeof(direction));

    const uint8_t tilt_enabled = config.tilt_enabled ? 1 : 0;
    tilt_characteristic->setValue(&tilt_enabled, sizeof(tilt_enabled));

    const uint8_t legato_enabled = config.legato_enabled ? 1 : 0;
    legato_characteristic->setValue(&legato_enabled, sizeof(legato_enabled));
}

static void calibrate_and_store_offsets() {
    runtime_state.status = STATUS_STATE_CALIBRATING;
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);

    saved_offsets.accel_x = mpu.getXAccelOffset();
    saved_offsets.accel_y = mpu.getYAccelOffset();
    saved_offsets.accel_z = mpu.getZAccelOffset();
    saved_offsets.gyro_x = mpu.getXGyroOffset();
    saved_offsets.gyro_y = mpu.getYGyroOffset();
    saved_offsets.gyro_z = mpu.getZGyroOffset();

    save_blob(PREF_KEY_OFFSETS, &saved_offsets, sizeof(MPUOffsets));
    runtime_state.status = STATUS_STATE_IDLE;
}

static int note_section_from_gyro(int gyro, size_t section_count) {
    int section = static_cast<int>((-gyro + GYRO_MAX_DEG) / (2.0f * GYRO_MAX_DEG) * section_count);
    return clamp_int(section, 0, static_cast<int>(section_count) - 1);
}

static MotionSample sample_motion() {
    mpu.dmpGetQuaternion(&quaternion, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &quaternion);
    mpu.dmpGetAccel(&raw_accel, fifo_buffer);
    mpu.dmpGetYawPitchRoll(yaw_pitch_roll, &quaternion, &gravity);
    mpu.dmpGetLinearAccel(&linear_accel, &raw_accel, &gravity);

    MotionSample sample;
    sample.gyro = static_cast<int>(yaw_pitch_roll[2] * -180.0f / M_PI);
    sample.gyro = static_cast<int>(clamp_float(sample.gyro, -GYRO_MAX_DEG, GYRO_MAX_DEG));

    sample.tilt = static_cast<int>(yaw_pitch_roll[1] * 180.0f / M_PI);
    sample.tilt = static_cast<int>(clamp_float(sample.tilt, -TILT_MAX_DEG, TILT_MAX_DEG));

    if (config.flip_gyro) {
        sample.gyro = -sample.gyro;
    }

    sample.touch = touchRead(TOUCH_PIN) < TOUCH_THRESHOLD;
    sample.accel = linear_accel.x / 10;

    const int section_index = note_section_from_gyro(sample.gyro, config.sections.size());
    sample.note = static_cast<byte>(config.sections[section_index]);
    return sample;
}

static void update_melodic_note(const MotionSample &sample) {
    if (sample.touch) {
        if (!runtime_state.touch_pressed) {
            if (runtime_state.note_playing && sample.note != runtime_state.last_note) {
                stop_note(runtime_state.last_note);
            }
            if (!runtime_state.note_playing || sample.note != runtime_state.last_note) {
                play_note(sample.note);
                runtime_state.note_playing = true;
            }
            runtime_state.touch_pressed = true;
            return;
        }

        if (sample.note != runtime_state.last_note) {
            if (runtime_state.note_playing) {
                stop_note(runtime_state.last_note);
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            play_note(sample.note);
            runtime_state.note_playing = true;
        }
        return;
    }

    if (runtime_state.touch_pressed) {
        runtime_state.touch_pressed = false;
        if (!config.legato_enabled && runtime_state.note_playing) {
            stop_note(runtime_state.last_note);
            runtime_state.note_playing = false;
        }
    }
}

static void update_percussion(const MotionSample &sample) {
    const unsigned long now = millis();
    const bool threshold_crossed = abs(sample.accel) > config.accel_threshold;
    const bool debounce_complete = (now - runtime_state.last_percussion_ms) >= ACCEL_DEBOUNCE_MS;

    if (!runtime_state.percussion_playing && threshold_crossed && debounce_complete) {
        if (config.legato_enabled && runtime_state.note_playing) {
            stop_note(runtime_state.last_note);
            runtime_state.note_playing = false;
        }
        play_note(PERC_NOTE, PERC_CHANNEL);
        runtime_state.percussion_playing = true;
        runtime_state.last_percussion_ms = now;
    }

    if (runtime_state.percussion_playing && debounce_complete) {
        stop_note(PERC_NOTE, PERC_CHANNEL);
        runtime_state.percussion_playing = false;
    }
}

static void update_pitch_bend(const MotionSample &sample) {
    if (!config.tilt_enabled) {
        if (runtime_state.last_pitch_bend != PITCH_BEND_CENTER) {
            send_pitch_bend(PITCH_BEND_CENTER);
            runtime_state.last_pitch_bend = PITCH_BEND_CENTER;
        }
        return;
    }

    int bend = PITCH_BEND_CENTER;
    const int absolute_tilt = abs(sample.tilt);
    if (absolute_tilt > PITCH_BEND_DEADZONE_DEG) {
        const int tilt_without_deadzone = sample.tilt > 0
            ? absolute_tilt - PITCH_BEND_DEADZONE_DEG
            : -(absolute_tilt - PITCH_BEND_DEADZONE_DEG);
        const int max_effective_tilt = PITCH_BEND_MAX_DEG - PITCH_BEND_DEADZONE_DEG;
        const int clamped_tilt = clamp_int(tilt_without_deadzone, -max_effective_tilt, max_effective_tilt);
        bend = PITCH_BEND_CENTER + (clamped_tilt * PITCH_BEND_CENTER) / max_effective_tilt;
    }

    if (bend != runtime_state.last_pitch_bend) {
        send_pitch_bend(bend);
        runtime_state.last_pitch_bend = bend;
    }
}

static void notify_status(const MotionSample &sample) {
    if (server_handle->getConnectedCount() == 0) {
        return;
    }

    status_packet.state = runtime_state.status;
    status_packet.touch = sample.touch ? 1 : 0;
    status_packet.gyro_x = static_cast<int16_t>(sample.gyro);
    status_packet.accel_x = static_cast<int16_t>(sample.accel);
    status_packet.tilt = static_cast<int16_t>(sample.tilt);
    status_characteristic->setValue(reinterpret_cast<const uint8_t *>(&status_packet), sizeof(StatusPacket));
    status_characteristic->notify();
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
        digitalWrite(LED_PIN, HIGH);
    }

    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        digitalWrite(LED_PIN, LOW);
        NimBLEDevice::startAdvertising();
    }
};

class SectionsCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
        config.sections = characteristic->getValue();
        save_blob(PREF_KEY_SECTIONS, config.sections.data(), config.sections.size());
    }
};

class AccelCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
        const std::string value = characteristic->getValue();
        int16_t threshold = 0;
        memcpy(&threshold, value.data(), sizeof(int16_t));
        config.accel_threshold = clamp_int(threshold, MIN_ACCEL_THRESHOLD, MAX_ACCEL_THRESHOLD);
        save_i32(PREF_KEY_ACCEL, config.accel_threshold);
        publish_config_to_ble();
    }
};

class DirectionCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
        config.flip_gyro = static_cast<uint8_t>(characteristic->getValue()[0]) != 0;
        save_u8(PREF_KEY_DIRECTION, config.flip_gyro ? 1 : 0);
        publish_config_to_ble();
    }
};

class TiltCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
        config.tilt_enabled = static_cast<uint8_t>(characteristic->getValue()[0]) != 0;
        save_u8(PREF_KEY_TILT, config.tilt_enabled ? 1 : 0);
        runtime_state.last_pitch_bend = PITCH_BEND_CENTER;
        send_pitch_bend(PITCH_BEND_CENTER);
        publish_config_to_ble();
    }
};

class LegatoCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
        config.legato_enabled = static_cast<uint8_t>(characteristic->getValue()[0]) != 0;
        save_u8(PREF_KEY_LEGATO, config.legato_enabled ? 1 : 0);
        publish_config_to_ble();
    }
};

class CalibrateCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *, NimBLEConnInfo &) override {
        runtime_state.calibration_requested = true;
    }
};

static void initialize_ble() {
    NimBLEDevice::init(DEVICE_NAME);

    server_handle = NimBLEDevice::createServer();
    server_handle->setCallbacks(new ServerCallbacks());

    NimBLEService *service = server_handle->createService(MAIN_SERVICE_UUID);

    midi_characteristic = service->createCharacteristic(
        MIDI_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY
    );

    sections_characteristic = service->createCharacteristic(SECTIONS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    accel_characteristic = service->createCharacteristic(ACCEL_SENS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    direction_characteristic = service->createCharacteristic(DIR_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    tilt_characteristic = service->createCharacteristic(TILT_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    legato_characteristic = service->createCharacteristic(LEGATO_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    status_characteristic = service->createCharacteristic(STATUS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    calibrate_characteristic = service->createCharacteristic(CALIBRATE_CHAR_UUID, NIMBLE_PROPERTY::WRITE);

    sections_characteristic->setCallbacks(new SectionsCallbacks());
    accel_characteristic->setCallbacks(new AccelCallbacks());
    direction_characteristic->setCallbacks(new DirectionCallbacks());
    tilt_characteristic->setCallbacks(new TiltCallbacks());
    legato_characteristic->setCallbacks(new LegatoCallbacks());
    calibrate_characteristic->setCallbacks(new CalibrateCallbacks());

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(MAIN_SERVICE_UUID);
    advertising->enableScanResponse(true);
    advertising->setName(DEVICE_NAME);
    advertising->start();
}

static void initialize_imu() {
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    runtime_state.dmp_ready = true;
}

void setup() {
    Serial.begin(115200);
    delay(100);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    initialize_imu();
    read_persistent_config();
    initialize_ble();
    publish_config_to_ble();
}

void loop() {
    if (!runtime_state.dmp_ready) {
        return;
    }

    if (runtime_state.calibration_requested) {
        calibrate_and_store_offsets();
        runtime_state.calibration_requested = false;
    }

    const unsigned long now = millis();
    if ((now - runtime_state.last_status_ms) < STATUS_INTERVAL_MS) {
        return;
    }
    runtime_state.last_status_ms = now;

    if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
        return;
    }

    const MotionSample sample = sample_motion();
    update_melodic_note(sample);
    update_percussion(sample);
    update_pitch_bend(sample);
    notify_status(sample);
}
