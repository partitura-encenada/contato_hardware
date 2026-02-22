#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32_Nimble.h>
#include <NimBLEDevice.h>

#include "config.h"
#include "types.h"

// ─── Globals ─────────────────────────────────────────────────────────────────

Preferences prefs;

BLEMIDI_CREATE_INSTANCE(DEVICE_NAME, MIDI);
static NimBLEServer         *pServer        = nullptr;
NimBLECharacteristic        *pSectionsChar  = nullptr;
NimBLECharacteristic        *pStatusChar    = nullptr;
NimBLECharacteristic        *pAccelSensChar = nullptr;
NimBLECharacteristic        *pDirChar       = nullptr;
NimBLECharacteristic        *pCalibrateChar = nullptr;

StatusPacket statusPkt;

MPU6050     mpu;
uint16_t    fifo_count;
uint8_t     fifo_buffer[64];
Quaternion  q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
float       ypr[3];       // [yaw, pitch, roll]
bool        dmp_ready = false;

int32_t       accelThreshold = DEFAULT_ACCEL_THRESHOLD;
unsigned long lastSent       = 0;
unsigned long lastAccel      = 0;
bool          touchFlag      = false;
bool          accelFlag      = false;
byte          lastNote       = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static float clamp(float v, float max_v, float min_v) {
    if (v > max_v) return max_v;
    if (v < min_v) return min_v;
    return v;
}

static void playNote(byte n, uint8_t channel = 1) {
    lastNote = n;
    if (pServer->getConnectedCount())
        MIDI.sendNoteOn(n, 80, channel);
    Serial.printf("Tocando nota: %d (ch %u)\n", n, (unsigned)channel);
}

static void stopNote(byte n, uint8_t channel = 1) {
    lastNote = n;
    if (pServer->getConnectedCount())
        MIDI.sendNoteOff(n, 80, channel);
    Serial.printf("Parando nota: %d (ch %u)\n", n, (unsigned)channel);
}

static void printOffsets(const MPUOffsets &o) {
    Serial.printf("Offsets do acelerômetro: X=%d Y=%d Z=%d\n", o.accelX, o.accelY, o.accelZ);
    Serial.printf("Offsets do giroscópio:   X=%d Y=%d Z=%d\n", o.gyroX,  o.gyroY,  o.gyroZ);
}

// Runs MPU calibration and saves offsets to NVS. Returns true on success.
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

    prefs.begin(PREF_NAMESPACE, false);
    size_t wrote = prefs.putBytes(PREF_KEY_OFFS, &offs, sizeof(MPUOffsets));
    prefs.end();  // always close, regardless of outcome

    if (wrote == sizeof(MPUOffsets)) {
        Serial.println("Offsets salvos na NVS.");
        return true;
    }
    Serial.printf("ERRO salvando offsets: escreveu %u de %u bytes\n",
                  (unsigned)wrote, (unsigned)sizeof(MPUOffsets));
    return false;
}

// ─── BLE Callbacks ───────────────────────────────────────────────────────────
// LED connect/disconnect is handled via MIDI transport callbacks in setup().
// Re-advertising on disconnect is handled by the library (advertiseOnDisconnect).

class SectionsCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string val = pChar->getValue();
        Serial.print("Escrita em SECTIONS (bytes):");
        for (unsigned char c : val)
            Serial.printf(" %u", (unsigned)c);
        Serial.println();
        prefs.begin(PREF_NAMESPACE, false);
        prefs.putBytes(PREF_KEY_SECTIONS, val.data(), val.size());
        prefs.end();
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
        uint8_t b = (uint8_t)v[0];
        prefs.begin(PREF_NAMESPACE, false);
        prefs.putUChar(PREF_KEY_DIR, b);
        prefs.end();
        Serial.printf("DIR recebido: %u → flip_gyro=%s (salvo)\n",
                      (unsigned)b, b ? "true" : "false");
    }
};

class AccelSensCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &) override {
        std::string val = pChar->getValue();
        if (val.size() != 2) {
            Serial.println("Escrita em accel sens com tamanho inesperado.");
            return;
        }
        int16_t received = 0;
        memcpy(&received, val.data(), sizeof(int16_t));
        if (received < MIN_ACCEL_THRESHOLD || received > MAX_ACCEL_THRESHOLD) {
            Serial.printf("Accel threshold fora do intervalo válido: %d\n", (int)received);
            return;
        }
        accelThreshold = (int32_t)received;
        prefs.begin(PREF_NAMESPACE, false);
        prefs.putInt(PREF_KEY_SENS, accelThreshold);
        prefs.end();
        Serial.printf("Novo accel threshold recebido: %ld (salvo)\n", (long)accelThreshold);
    }
};

class CalibrateCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *, NimBLEConnInfo &) override {
        calibrateAndSaveOffsets();
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
    Serial.println(mpu.testConnection()
        ? "Conexão com MPU bem-sucedida"
        : "Conexão com MPU FALHOU");

    uint8_t dev_status = mpu.dmpInitialize();
    if (dev_status != 0) {
        Serial.printf("Falha ao inicializar DMP (status=%u)\n", dev_status);
        return;
    }
    mpu.setDMPEnabled(true);
    Serial.println("DMP inicializado e ativado.");

    // ── Load all persisted settings in one NVS session ──
    prefs.begin(PREF_NAMESPACE, true);

    MPUOffsets offsets;
    bool have_offsets = prefs.getBytes(PREF_KEY_OFFS, &offsets, sizeof(MPUOffsets)) == sizeof(MPUOffsets);
    if (have_offsets) {
        mpu.setXAccelOffset(offsets.accelX);
        mpu.setYAccelOffset(offsets.accelY);
        mpu.setZAccelOffset(offsets.accelZ);
        mpu.setXGyroOffset(offsets.gyroX);
        mpu.setYGyroOffset(offsets.gyroY);
        mpu.setZGyroOffset(offsets.gyroZ);
        Serial.println("Offsets carregados e aplicados:");
        printOffsets(offsets);
    }

    accelThreshold = prefs.getInt(PREF_KEY_SENS, DEFAULT_ACCEL_THRESHOLD);
    Serial.printf("Accel threshold carregado da NVS: %ld\n", (long)accelThreshold);

    uint8_t stored_dir = prefs.getUChar(PREF_KEY_DIR, 1);

    std::string saved_sections;
    size_t sections_size = prefs.getBytes(PREF_KEY_SECTIONS, nullptr, 0);
    if (sections_size > 0) {
        std::vector<char> buf(sections_size);
        prefs.getBytes(PREF_KEY_SECTIONS, buf.data(), sections_size);
        saved_sections.assign(buf.begin(), buf.end());
    }

    prefs.end();
    // ── End NVS session ──

    // Run MPU calibration only if no saved offsets exist.
    // Skipping this when offsets are loaded avoids overwriting known-good values.
    if (!have_offsets) {
        Serial.println("Nenhum offset salvo; calibrando agora...");
        mpu.CalibrateGyro(6);
        mpu.CalibrateAccel(6);
    }

    // ── BLE/MIDI setup ──
    // MIDI.begin() calls NimBLEDevice::init(), creates the server, sets its own
    // transport callbacks, and starts advertising. We must call it first.
    MIDI.begin(MIDI_CHANNEL_OMNI);

    // Hook LED to the MIDI transport's connect/disconnect events.
    // advertiseOnDisconnect(true) is already set by the library — no need to
    // call NimBLEDevice::startAdvertising() ourselves on disconnect.
    BLEMIDI.setHandleConnected([]() {
        Serial.println("Client conectado");
        digitalWrite(LED_PIN, HIGH);
    });
    BLEMIDI.setHandleDisconnected([]() {
        Serial.println("Client desconectado, anunciando");
        digitalWrite(LED_PIN, LOW);
    });

    pServer = NimBLEDevice::getServer();

    NimBLEService *pMainService = pServer->createService(MAIN_SERVICE_UUID);

    pSectionsChar = pMainService->createCharacteristic(
        SECTIONS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pSectionsChar->setCallbacks(new SectionsCharCallbacks());

    pAccelSensChar = pMainService->createCharacteristic(
        ACCEL_SENS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pAccelSensChar->setCallbacks(new AccelSensCharCallbacks());

    pDirChar = pMainService->createCharacteristic(
        DIR_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pDirChar->setCallbacks(new DirCharCallbacks());

    pStatusChar = pMainService->createCharacteristic(
        STATUS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    pStatusChar->setCallbacks(new NotifyCharCallbacks());

    pCalibrateChar = pMainService->createCharacteristic(
        CALIBRATE_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    pCalibrateChar->setCallbacks(new CalibrateCharCallbacks());

    pMainService->start();

    if (!saved_sections.empty()) {
        pSectionsChar->setValue(saved_sections);
    } else {
        Serial.println("Nenhuma nota encontrada ao iniciar; usando padrão.");
        std::string def(DEFAULT_SECTION_COUNT, (char)DEFAULT_NOTE);
        pSectionsChar->setValue(def);
    }

    pDirChar->setValue(stored_dir != 0);

    Serial.println("Anúncio BLE iniciado");
    dmp_ready = true;
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
    if (!dmp_ready) return;

    unsigned long now = millis();
    if (now - lastSent < STATUS_INTERVAL_MS) return;
    lastSent = now;

    if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) return;

    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifo_buffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    // Map roll angle [-89°, +89°] for note selection
    int gyro = (int)(ypr[2] * -180.0f / M_PI);
    gyro = (int)clamp((float)gyro, GYRO_MAX_DEG, -GYRO_MAX_DEG);

    // Optionally invert direction (configured via BLE)
    std::string dir = pDirChar->getValue();
    if (dir.size() >= 1 && (bool)dir[0]) gyro = -gyro;

    bool touch = touchRead(TOUCH_PIN) < TOUCH_THRESHOLD;
    int  accel = aaReal.x / 10;

    // Map gyro position to a note index in the configured note array
    std::string notes = pSectionsChar->getValue();
    int section = (int)((-gyro + GYRO_MAX_DEG) / (2.0f * GYRO_MAX_DEG) * notes.length());
    if (section >= (int)notes.length())
        section = (int)notes.length() - 1;  // clamp to valid index
    if (section < 0) section = 0;

    byte currentNote = notes.empty() ? DEFAULT_NOTE : (byte)notes[section];

    // Touch note logic
    if (touch) {
        if (!touchFlag) {
            playNote(currentNote);
            touchFlag = true;
        }
        if (currentNote != lastNote) {
            stopNote(lastNote);
            vTaskDelay(10);
            playNote(currentNote);
        }
    } else {
        if (touchFlag) {
            stopNote(lastNote);
            touchFlag = false;
        }
    }

    // Accelerometer percussion logic
    if (!accelFlag && abs(accel) > accelThreshold && (now - lastAccel) >= ACCEL_DEBOUNCE_MS) {
        Serial.printf("Accel trigger: value=%d threshold=%ld\n", accel, (long)accelThreshold);
        playNote(PERC_NOTE, PERC_CHANNEL);
        accelFlag = true;
        lastAccel = now;
    }
    if (accelFlag && (now - lastAccel) >= ACCEL_DEBOUNCE_MS) {
        stopNote(PERC_NOTE, PERC_CHANNEL);
        accelFlag = false;
    }

    // Send status notification if a client is connected
    if (pServer->getConnectedCount()) {
        statusPkt.gyro_x  = (int16_t)gyro;
        statusPkt.accel_x = (int16_t)accel;
        statusPkt.touch   = (uint8_t)touch;
        pStatusChar->setValue((uint8_t *)&statusPkt, sizeof(StatusPacket));
        pStatusChar->notify();
        delay(10);
    }
}
