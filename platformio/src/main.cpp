#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <Preferences.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <NimBLEDevice.h>

#include "config.h"
#include "types.h"

Preferences prefs;

static NimBLEServer *pServer = nullptr;
static NimBLECharacteristic *pMidiChar = nullptr;
static NimBLECharacteristic *pSectionsChar = nullptr;
static NimBLECharacteristic *pStatusChar = nullptr;
static NimBLECharacteristic *pAccelSensChar = nullptr;
static NimBLECharacteristic *pDirChar = nullptr;
static NimBLECharacteristic *pTiltChar = nullptr;
static NimBLECharacteristic *pLegatoChar = nullptr;
static NimBLECharacteristic *pCalibrateChar = nullptr;

StatusPacket statusPkt;
RuntimeState app;

MPU6050 mpu;
uint8_t fifo_buffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
float ypr[3];

static float clamp(float v, float max_v, float min_v) {
    if (v > max_v) return max_v;
    if (v < min_v) return min_v;
    return v;
}

static void setBoolChar(NimBLECharacteristic *ch, bool value) {
    uint8_t raw = value ? 1 : 0;
    ch->setValue(&raw, sizeof(raw));
}

static void setInt16Char(NimBLECharacteristic *ch, int16_t value) {
    ch->setValue((uint8_t *)&value, sizeof(value));
}

static void saveUChar(const char *key, uint8_t value) {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putUChar(key, value);
    prefs.end();
}

static void saveInt(const char *key, int32_t value) {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putInt(key, value);
    prefs.end();
}

static void saveBytes(const char *key, const void *data, size_t size) {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putBytes(key, data, size);
    prefs.end();
}

static void sendMidiMessage(uint8_t status, uint8_t data1, uint8_t data2) {
    if (!pServer || !pServer->getConnectedCount()) return;
    uint16_t ts = (uint16_t)millis();
    uint8_t pkt[5] = {
        (uint8_t)(0x80 | ((ts >> 7) & 0x3F)),
        (uint8_t)(0x80 | (ts & 0x7F)),
        status, data1, data2
    };
    pMidiChar->setValue(pkt, sizeof(pkt));
    pMidiChar->notify();
}

static void playNote(byte n, uint8_t channel = NOTE_CHANNEL) {
    app.lastNote = n;
    sendMidiMessage(0x90 | ((channel - 1) & 0x0F), n, 80);
    Serial.printf("Tocando nota: %d (ch %u)\n", n, (unsigned)channel);
}

static void stopNote(byte n, uint8_t channel = NOTE_CHANNEL) {
    app.lastNote = n;
    sendMidiMessage(0x80 | ((channel - 1) & 0x0F), n, 80);
    Serial.printf("Parando nota: %d (ch %u)\n", n, (unsigned)channel);
}

static void sendPitchBend(int value, uint8_t channel = NOTE_CHANNEL) {
    value = (int)clamp((float)value, PITCH_BEND_MAX_VALUE, 0.0f);
    sendMidiMessage(
        0xE0 | ((channel - 1) & 0x0F),
        value & 0x7F,
        (value >> 7) & 0x7F
    );
}

static void printOffsets(const MPUOffsets &o) {
    Serial.printf("Offsets do acelerômetro: X=%d Y=%d Z=%d\n", o.accelX, o.accelY, o.accelZ);
    Serial.printf("Offsets do giroscópio:   X=%d Y=%d Z=%d\n", o.gyroX,  o.gyroY,  o.gyroZ);
}

static bool calibrateAndSaveOffsets() {
    Serial.println("Iniciando calibração");
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);

    MPUOffsets offs;
    offs.accelX = mpu.getXAccelOffset();
    offs.accelY = mpu.getYAccelOffset();
    offs.accelZ = mpu.getZAccelOffset();
    offs.gyroX  = mpu.getXGyroOffset();
    offs.gyroY  = mpu.getYGyroOffset();
    offs.gyroZ  = mpu.getZGyroOffset();

    Serial.println("Calibração concluída. Offsets:");
    printOffsets(offs);
    saveBytes(PREF_KEY_OFFS, &offs, sizeof(MPUOffsets));
    size_t wrote = sizeof(MPUOffsets);

    if (wrote == sizeof(MPUOffsets)) {
        Serial.println("Offsets salvos na NVS.");
        return true;
    }
    return false;
}

static void notifyStatus(int gyro, int accel, bool touch, int tilt) {
    if (!pServer || !pServer->getConnectedCount()) return;
    statusPkt.state   = app.status;
    statusPkt.touch   = (uint8_t)touch;
    statusPkt.gyro_x  = (int16_t)gyro;
    statusPkt.accel_x = (int16_t)accel;
    statusPkt.tilt    = (int16_t)tilt;
    pStatusChar->setValue((uint8_t *)&statusPkt, sizeof(StatusPacket));
    pStatusChar->notify();
}

static void loadStoredConfig() {
    prefs.begin(PREF_NAMESPACE, true);

    MPUOffsets offsets;
    bool haveOffsets = prefs.getBytes(PREF_KEY_OFFS, &offsets, sizeof(MPUOffsets)) == sizeof(MPUOffsets);
    if (haveOffsets) {
        mpu.setXAccelOffset(offsets.accelX);
        mpu.setYAccelOffset(offsets.accelY);
        mpu.setZAccelOffset(offsets.accelZ);
        mpu.setXGyroOffset(offsets.gyroX);
        mpu.setYGyroOffset(offsets.gyroY);
        mpu.setZGyroOffset(offsets.gyroZ);
        Serial.println("Offsets carregados e aplicados:");
        printOffsets(offsets);
    }

    app.accelThreshold = prefs.getInt(PREF_KEY_SENS, DEFAULT_ACCEL_THRESHOLD);
    app.flipGyro = prefs.getUChar(PREF_KEY_DIR, DEFAULT_DIR) != 0;
    app.tiltEnabled = prefs.getUChar(PREF_KEY_TILT, DEFAULT_TILT_ENABLED) != 0;
    app.legatoEnabled = prefs.getUChar(PREF_KEY_LEGATO, DEFAULT_LEGATO_ENABLED) != 0;

    size_t sectionsSize = prefs.getBytes(PREF_KEY_SECTIONS, nullptr, 0);
    std::string sections;
    if (sectionsSize > 0) {
        std::vector<char> buf(sectionsSize);
        prefs.getBytes(PREF_KEY_SECTIONS, buf.data(), sectionsSize);
        sections.assign(buf.begin(), buf.end());
    }

    prefs.end();

    if (!haveOffsets) {
        Serial.println("Nenhum offset salvo; calibrando agora...");
        calibrateAndSaveOffsets();
    }

    if (sections.empty()) {
        sections.assign(DEFAULT_SECTION_COUNT, (char)DEFAULT_NOTE);
    }

    pSectionsChar->setValue(sections);
    setInt16Char(pAccelSensChar, (int16_t)app.accelThreshold);
    setBoolChar(pDirChar, app.flipGyro);
    setBoolChar(pTiltChar, app.tiltEnabled);
    setBoolChar(pLegatoChar, app.legatoEnabled);
}

static int noteSectionFromGyro(int gyro, size_t count) {
    if (count == 0) return 0;
    int section = (int)((-gyro + GYRO_MAX_DEG) / (2.0f * GYRO_MAX_DEG) * count);
    if (section >= (int)count) section = (int)count - 1;
    if (section < 0) section = 0;
    return section;
}

static void updatePitchBend(int tilt) {
    if (!app.tiltEnabled) {
        if (app.lastPitchBend != PITCH_BEND_CENTER) {
            sendPitchBend(PITCH_BEND_CENTER);
            app.lastPitchBend = PITCH_BEND_CENTER;
        }
        return;
    }

    int bend = PITCH_BEND_CENTER;
    int absTilt = abs(tilt);

    if (absTilt > PITCH_BEND_DEADZONE_DEG) {
        int signedTilt = tilt > 0 ? absTilt - PITCH_BEND_DEADZONE_DEG : -(absTilt - PITCH_BEND_DEADZONE_DEG);
        int maxTilt = PITCH_BEND_MAX_DEG - PITCH_BEND_DEADZONE_DEG;
        if (signedTilt > maxTilt) signedTilt = maxTilt;
        if (signedTilt < -maxTilt) signedTilt = -maxTilt;
        bend = PITCH_BEND_CENTER + (signedTilt * PITCH_BEND_CENTER) / maxTilt;
    }

    if (bend != app.lastPitchBend) {
        sendPitchBend(bend);
        app.lastPitchBend = bend;
    }
}

static void updateMelodicNote(byte currentNote, bool touch) {
    if (touch) {
        if (!app.touchPressed) {
            if (app.notePlaying && currentNote != app.lastNote) stopNote(app.lastNote);
            if (!app.notePlaying || currentNote != app.lastNote) {
                playNote(currentNote);
                app.notePlaying = true;
            }
            app.touchPressed = true;
            return;
        }

        if (currentNote != app.lastNote) {
            if (app.notePlaying) stopNote(app.lastNote);
            vTaskDelay(pdMS_TO_TICKS(10));
            playNote(currentNote);
            app.notePlaying = true;
        }
        return;
    }

    if (!app.touchPressed) return;
    app.touchPressed = false;
    if (!app.legatoEnabled && app.notePlaying) {
        stopNote(app.lastNote);
        app.notePlaying = false;
    }
}

static void updatePercussion(int accel) {
    unsigned long now = millis();
    if (!app.accelPlaying && abs(accel) > app.accelThreshold && (now - app.lastAccel) >= ACCEL_DEBOUNCE_MS) {
        Serial.printf("Accel trigger: value=%d threshold=%ld\n", accel, (long)app.accelThreshold);
        if (app.legatoEnabled && app.notePlaying) {
            stopNote(app.lastNote);
            app.notePlaying = false;
        }
        playNote(PERC_NOTE, PERC_CHANNEL);
        app.accelPlaying = true;
        app.lastAccel = now;
    }

    if (app.accelPlaying && (now - app.lastAccel) >= ACCEL_DEBOUNCE_MS) {
        stopNote(PERC_NOTE, PERC_CHANNEL);
        app.accelPlaying = false;
    }
}

static void handleCalibrationIfNeeded() {
    if (!app.calibrationRequested) return;
    app.status = STATUS_STATE_CALIBRATING;
    notifyStatus(0, 0, false, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    calibrateAndSaveOffsets();
    app.status = STATUS_STATE_IDLE;
    app.calibrationRequested = false;
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
        Serial.println("Client conectado");
        digitalWrite(LED_PIN, HIGH);
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        Serial.println("Client desconectado, anunciando");
        digitalWrite(LED_PIN, LOW);
        NimBLEDevice::startAdvertising();
    }
};

class SectionsCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string val = pChar->getValue();
        saveBytes(PREF_KEY_SECTIONS, val.data(), val.size());
        Serial.println("SECTIONS salvo na NVS.");
    }
};

class NotifyCharCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *pChar, NimBLEConnInfo &, uint16_t subValue) override {
        Serial.printf("Cliente inscrito em %s (subValue=%u)\n",
                      pChar->getUUID().toString().c_str(), subValue);
    }
};

class DirCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string v = pChar->getValue();
        if (v.size() < 1) return;
        app.flipGyro = (uint8_t)v[0] != 0;
        saveUChar(PREF_KEY_DIR, app.flipGyro ? 1 : 0);
        Serial.printf("DIR recebido: %u → flip_gyro=%s (salvo)\n", (unsigned)v[0], app.flipGyro ? "true" : "false");
    }
};

class TiltCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string v = pChar->getValue();
        if (v.size() < 1) return;
        app.tiltEnabled = (uint8_t)v[0] != 0;
        saveUChar(PREF_KEY_TILT, app.tiltEnabled ? 1 : 0);
        if (!app.tiltEnabled) {
            sendPitchBend(PITCH_BEND_CENTER);
            app.lastPitchBend = PITCH_BEND_CENTER;
        }
        Serial.printf("TILT recebido: %u → pitch bend=%s (salvo)\n", (unsigned)v[0], app.tiltEnabled ? "true" : "false");
    }
};

class LegatoCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string v = pChar->getValue();
        if (v.size() < 1) return;
        app.legatoEnabled = (uint8_t)v[0] != 0;
        saveUChar(PREF_KEY_LEGATO, app.legatoEnabled ? 1 : 0);
        Serial.printf("LEGATO recebido: %u → legato=%s (salvo)\n", (unsigned)v[0], app.legatoEnabled ? "true" : "false");
    }
};

class AccelSensCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string val = pChar->getValue();
        if (val.size() != 2) {
            return;
        }
        int16_t received = 0;
        memcpy(&received, val.data(), sizeof(int16_t));
        if (received < MIN_ACCEL_THRESHOLD || received > MAX_ACCEL_THRESHOLD) {
            return;
        }
        app.accelThreshold = (int32_t)received;
        saveInt(PREF_KEY_SENS, app.accelThreshold);
        Serial.printf("Novo accel threshold recebido: %ld (salvo)\n", (long)app.accelThreshold);
    }
};

class CalibrateCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *, NimBLEConnInfo &) override {
        app.calibrationRequested = true;
    }
};

void setup() {
    Serial.begin(115200);
    delay(100);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    mpu.initialize();
    Serial.println(mpu.testConnection() ? "Conexão com MPU bem-sucedida" : "Conexão com MPU FALHOU");

    uint8_t dev_status = mpu.dmpInitialize();
    if (dev_status != 0) return;
    mpu.setDMPEnabled(true);

    NimBLEDevice::init(DEVICE_NAME);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pMainService = pServer->createService(MAIN_SERVICE_UUID);

    // Característica BLE MIDI (especificação padrão: READ | WRITE_NR | NOTIFY)
    pMidiChar = pMainService->createCharacteristic(
        MIDI_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);

    pSectionsChar = pMainService->createCharacteristic(
        SECTIONS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pSectionsChar->setCallbacks(new SectionsCharCallbacks());

    pAccelSensChar = pMainService->createCharacteristic(
        ACCEL_SENS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pAccelSensChar->setCallbacks(new AccelSensCharCallbacks());

    pDirChar = pMainService->createCharacteristic(
        DIR_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pDirChar->setCallbacks(new DirCharCallbacks());

    pTiltChar = pMainService->createCharacteristic(
        TILT_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pTiltChar->setCallbacks(new TiltCharCallbacks());

    pLegatoChar = pMainService->createCharacteristic(
        LEGATO_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pLegatoChar->setCallbacks(new LegatoCharCallbacks());

    pStatusChar = pMainService->createCharacteristic(
        STATUS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    pStatusChar->setCallbacks(new NotifyCharCallbacks());

    pCalibrateChar = pMainService->createCharacteristic(
        CALIBRATE_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    pCalibrateChar->setCallbacks(new CalibrateCharCallbacks());

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(MAIN_SERVICE_UUID);
    pAdvertising->enableScanResponse(true);
    pAdvertising->setName(DEVICE_NAME);
    pAdvertising->start();

    loadStoredConfig();
    Serial.println("Anúncio BLE iniciado");
    app.dmpReady = true;
}

void loop() {
    if (!app.dmpReady) return;
    handleCalibrationIfNeeded();

    unsigned long now = millis();
    if (now - app.lastSent < STATUS_INTERVAL_MS) return;
    app.lastSent = now;

    if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) return;

    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifo_buffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    int gyro = (int)(ypr[2] * -180.0f / M_PI);
    gyro = (int)clamp((float)gyro, GYRO_MAX_DEG, -GYRO_MAX_DEG);
    int tilt = (int)(ypr[1] * 180.0f / M_PI);
    tilt = (int)clamp((float)tilt, TILT_MAX_DEG, -TILT_MAX_DEG);

    if (app.flipGyro) gyro = -gyro;

    bool touch = touchRead(TOUCH_PIN) < TOUCH_THRESHOLD;
    int accel = aaReal.x / 10;

    std::string notes = pSectionsChar->getValue();
    int section = noteSectionFromGyro(gyro, notes.length());
    byte currentNote = notes.empty() ? DEFAULT_NOTE : (byte)notes[section];

    updateMelodicNote(currentNote, touch);
    updatePercussion(accel);
    updatePitchBend(tilt);
    notifyStatus(gyro, accel, touch, tilt);
}
