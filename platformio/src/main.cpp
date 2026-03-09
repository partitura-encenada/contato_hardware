#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <NimBLEDevice.h>

#include "config.h"
#include "types.h"

// ─── Globals ─────────────────────────────────────────────────────────────────

Preferences prefs;

static NimBLEServer        *pServer        = nullptr;
NimBLECharacteristic       *pMidiChar      = nullptr;
NimBLECharacteristic       *pSectionsChar  = nullptr;
NimBLECharacteristic       *pStatusChar    = nullptr;
NimBLECharacteristic       *pAccelSensChar = nullptr;
NimBLECharacteristic       *pDirChar       = nullptr;
NimBLECharacteristic       *pCalibrateChar = nullptr;

MPU6050     mpu;
uint8_t     fifo_buffer[64];
Quaternion  q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
float       ypr[3];
bool        dmp_ready = false;

int32_t       accelThreshold      = DEFAULT_ACCEL_THRESHOLD;
unsigned long lastSent            = 0;
unsigned long lastAccel           = 0;
bool          touchFlag           = false;
bool          accelFlag           = false;
byte          lastNote            = 0;
volatile bool calibrate_requested = false;

// ─── MIDI helpers ─────────────────────────────────────────────────────────────

static void sendMidi(uint8_t status, uint8_t d1, uint8_t d2) {
    if (!pServer->getConnectedCount()) return;
    uint16_t ts = (uint16_t)millis();
    uint8_t pkt[5] = {
        (uint8_t)(0x80 | ((ts >> 7) & 0x3F)),  // header: timestamp bits [12:7]
        (uint8_t)(0x80 | (ts & 0x7F)),          // timestamp bits [6:0]
        status, d1, d2
    };
    pMidiChar->setValue(pkt, sizeof(pkt));
    pMidiChar->notify();
}

static void noteOn(byte note, uint8_t ch = 1) {
    lastNote = note;
    sendMidi(0x90 | ((ch - 1) & 0x0F), note, 80);
    Serial.printf("Note On:  %d ch%u\n", note, (unsigned)ch);
}

static void noteOff(byte note, uint8_t ch = 1) {
    sendMidi(0x80 | ((ch - 1) & 0x0F), note, 0);
    Serial.printf("Note Off: %d ch%u\n", note, (unsigned)ch);
}

// ─── Calibration ─────────────────────────────────────────────────────────────

static void calibrateAndSave() {
    Serial.println("Calibrando...");
    digitalWrite(LED_PIN, HIGH);

    mpu.setDMPEnabled(false);
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);
    mpu.setDMPEnabled(true);

    MPUOffsets offs;
    offs.accelX = mpu.getXAccelOffset();
    offs.accelY = mpu.getYAccelOffset();
    offs.accelZ = mpu.getZAccelOffset();
    offs.gyroX  = mpu.getXGyroOffset();
    offs.gyroY  = mpu.getYGyroOffset();
    offs.gyroZ  = mpu.getZGyroOffset();

    prefs.begin(PREF_NAMESPACE, false);
    prefs.putBytes(PREF_KEY_OFFS, &offs, sizeof(MPUOffsets));
    prefs.end();

    Serial.printf("Offsets: AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d\n",
        offs.accelX, offs.accelY, offs.accelZ,
        offs.gyroX,  offs.gyroY,  offs.gyroZ);

    digitalWrite(LED_PIN, pServer->getConnectedCount() ? HIGH : LOW);
}

// ─── BLE Callbacks ───────────────────────────────────────────────────────────

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
        Serial.println("Client conectado");
        digitalWrite(LED_PIN, HIGH);
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        Serial.println("Client desconectado");
        digitalWrite(LED_PIN, LOW);
        NimBLEDevice::startAdvertising();
    }
};

class SectionsCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string val = pChar->getValue();
        prefs.begin(PREF_NAMESPACE, false);
        prefs.putBytes(PREF_KEY_SECTIONS, val.data(), val.size());
        prefs.end();
        Serial.print("Sections:");
        for (unsigned char c : val) Serial.printf(" %u", (unsigned)c);
        Serial.println();
    }
};

class AccelSensCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string val = pChar->getValue();
        int16_t received;
        memcpy(&received, val.data(), sizeof(int16_t));
        accelThreshold = (int32_t)received;
        pAccelSensChar->setValue(accelThreshold);
        prefs.begin(PREF_NAMESPACE, false);
        prefs.putInt(PREF_KEY_SENS, accelThreshold);
        prefs.end();
        Serial.printf("Accel threshold: %ld\n", (long)accelThreshold);
    }
};

class DirCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string v = pChar->getValue();
        uint8_t b = (uint8_t)v[0];
        prefs.begin(PREF_NAMESPACE, false);
        prefs.putUChar(PREF_KEY_DIR, b);
        prefs.end();
        Serial.printf("Direção: %u\n", (unsigned)b);
    }
};

class CalibrateCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *, NimBLEConnInfo &) override {
        calibrate_requested = true;
    }
};

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(100);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);

    Serial.println("Inicializando MPU6050...");
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU OK" : "MPU FALHOU");
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);

    // ── Load NVS config ──────────────────────────────────────────────────────
    prefs.begin(PREF_NAMESPACE, true);

    MPUOffsets offsets;
    bool have_offsets = prefs.isKey(PREF_KEY_OFFS);
    if (have_offsets) {
        prefs.getBytes(PREF_KEY_OFFS, &offsets, sizeof(MPUOffsets));
        mpu.setXAccelOffset(offsets.accelX);
        mpu.setYAccelOffset(offsets.accelY);
        mpu.setZAccelOffset(offsets.accelZ);
        mpu.setXGyroOffset(offsets.gyroX);
        mpu.setYGyroOffset(offsets.gyroY);
        mpu.setZGyroOffset(offsets.gyroZ);
        Serial.printf("Offsets: AX=%d AY=%d AZ=%d GX=%d GY=%d GZ=%d\n",
            offsets.accelX, offsets.accelY, offsets.accelZ,
            offsets.gyroX,  offsets.gyroY,  offsets.gyroZ);
    }

    accelThreshold = prefs.getInt(PREF_KEY_SENS, DEFAULT_ACCEL_THRESHOLD);

    uint8_t stored_dir = prefs.getUChar(PREF_KEY_DIR, 0);

    uint8_t sec_buf[8];
    size_t sections_size = prefs.isKey(PREF_KEY_SECTIONS)
        ? prefs.getBytes(PREF_KEY_SECTIONS, sec_buf, sizeof(sec_buf))
        : 0;
    std::string saved_sections((char *)sec_buf, sections_size);

    prefs.end();
    // ─────────────────────────────────────────────────────────────────────────

    if (!have_offsets) {
        Serial.println("Sem offsets salvos, calibrando...");
        calibrateAndSave();
    }

    // ── BLE setup ─────────────────────────────────────────────────────────────
    NimBLEDevice::init(DEVICE_NAME);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *svc = pServer->createService(MAIN_SERVICE_UUID);

    pMidiChar = svc->createCharacteristic(
        MIDI_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);

    pSectionsChar = svc->createCharacteristic(
        SECTIONS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pSectionsChar->setCallbacks(new SectionsCallbacks());

    pAccelSensChar = svc->createCharacteristic(
        ACCEL_SENS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pAccelSensChar->setCallbacks(new AccelSensCallbacks());

    pDirChar = svc->createCharacteristic(
        DIR_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pDirChar->setCallbacks(new DirCallbacks());

    pStatusChar = svc->createCharacteristic(
        STATUS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

    pCalibrateChar = svc->createCharacteristic(
        CALIBRATE_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    pCalibrateChar->setCallbacks(new CalibrateCallbacks());

    svc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(MAIN_SERVICE_UUID);
    adv->enableScanResponse(true);
    adv->setName(DEVICE_NAME);
    adv->start();

    if (sections_size > 0) {
        pSectionsChar->setValue(saved_sections);
    } else {
        Serial.println("Sem seções salvas, usando padrão.");
        std::string def(DEFAULT_SECTION_COUNT, (char)DEFAULT_NOTE);
        pSectionsChar->setValue(def);
    }

    pAccelSensChar->setValue(accelThreshold);
    pDirChar->setValue(stored_dir);

    Serial.println("BLE pronto");
    dmp_ready = true;
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
    if (!dmp_ready) return;

    if (calibrate_requested) {
        calibrate_requested = false;
        if (pServer->getConnectedCount()) {
            StatusPacket pkt = { 1, 0, 0, 0 };
            pStatusChar->setValue((uint8_t *)&pkt, sizeof(StatusPacket));
            pStatusChar->notify();
        }
        calibrateAndSave();
    }

    unsigned long now = millis();
    if (now - lastSent < SENSOR_INTERVAL_MS) return;
    lastSent = now;

    if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) return;

    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifo_buffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    // Roll angle mapped to degrees, clamped to ±GYRO_MAX_DEG
    int gyro = (int)(ypr[2] * -180.0f / M_PI);
    if (gyro >  (int)GYRO_MAX_DEG) gyro =  (int)GYRO_MAX_DEG;
    if (gyro < -(int)GYRO_MAX_DEG) gyro = -(int)GYRO_MAX_DEG;

    std::string dir = pDirChar->getValue();
    if (dir.size() >= 1 && dir[0]) gyro = -gyro;

    bool touch = touchRead(TOUCH_PIN) < TOUCH_THRESHOLD;
    int  accel = aaReal.x / 10;

    // Map gyro angle to section index
    std::string notes = pSectionsChar->getValue();
    int section = 0;
    if (!notes.empty()) {
        section = (int)((-gyro + GYRO_MAX_DEG) / (2.0f * GYRO_MAX_DEG) * (int)notes.size());
        if (section < 0)                    section = 0;
        if (section >= (int)notes.size())   section = (int)notes.size() - 1;
    }
    byte currentNote = notes.empty() ? DEFAULT_NOTE : (byte)notes[section];

    // ── Touch: note on/off, glide across sections ─────────────────────────────
    if (touch) {
        if (!touchFlag) {
            noteOn(currentNote);
            touchFlag = true;
        } else if (currentNote != lastNote) {
            noteOff(lastNote);
            delay(10);
            noteOn(currentNote);
        }
    } else if (touchFlag) {
        noteOff(lastNote);
        touchFlag = false;
    }

    // ── Percussion: short note triggered by accel spike ───────────────────────
    if (accelFlag && (now - lastAccel) >= PERC_NOTE_MS) {
        noteOff(PERC_NOTE, PERC_CHANNEL);
        accelFlag = false;
    }
    if (!accelFlag && abs(accel) > accelThreshold && (now - lastAccel) >= ACCEL_DEBOUNCE_MS) {
        noteOn(PERC_NOTE, PERC_CHANNEL);
        accelFlag = true;
        lastAccel = now;
    }

    // ── Status notify ─────────────────────────────────────────────────────────
    if (pServer->getConnectedCount()) {
        StatusPacket pkt = { 0, (uint8_t)touch, (int16_t)gyro, (int16_t)accel };
        pStatusChar->setValue((uint8_t *)&pkt, sizeof(StatusPacket));
        pStatusChar->notify();
    }
}
