#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <NimBLEDevice.h>

#include "config.h"
#include "types.h"

// ─── Variáveis Globais ───────────────────────────────────────────────────────

Preferences prefs;

static NimBLEServer         *pServer        = nullptr;
NimBLECharacteristic        *pMidiChar      = nullptr;
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
float       ypr[3];       // [yaw, pitch, roll] — guinada, arfagem, rolagem
bool        dmp_ready = false;

int32_t          accelThreshold     = DEFAULT_ACCEL_THRESHOLD;
unsigned long    lastSent           = 0;
unsigned long    lastAccel          = 0;
bool             touchFlag          = false;
bool             accelFlag          = false;
byte             lastNote           = 0;
volatile bool    calibrate_requested = false;

// ─── Auxiliares ──────────────────────────────────────────────────────────────

static float clamp(float v, float max_v, float min_v) {
    if (v > max_v) return max_v;
    if (v < min_v) return min_v;
    return v;
}

// Pacote BLE MIDI: cabeçalho de timestamp de 2 bytes + status MIDI + bytes de dados.
// Timestamp derivado de millis(); 6 bits altos no cabeçalho, 7 bits baixos no byte ts.
static void sendMidiMessage(uint8_t status, uint8_t data1, uint8_t data2) {
    if (!pServer || !pServer->getConnectedCount()) return;
    uint16_t ts = (uint16_t)millis();
    uint8_t pkt[5] = {
        (uint8_t)(0x80 | ((ts >> 7) & 0x3F)),  // cabeçalho
        (uint8_t)(0x80 | (ts & 0x7F)),          // timestamp
        status, data1, data2
    };
    pMidiChar->setValue(pkt, sizeof(pkt));
    pMidiChar->notify();
}

static void playNote(byte n, uint8_t channel = 1) {
    lastNote = n;
    sendMidiMessage(0x90 | ((channel - 1) & 0x0F), n, 80);
    Serial.printf("Tocando nota: %d (ch %u)\n", n, (unsigned)channel);
}

static void stopNote(byte n, uint8_t channel = 1) {
    lastNote = n;
    sendMidiMessage(0x80 | ((channel - 1) & 0x0F), n, 80);
    Serial.printf("Parando nota: %d (ch %u)\n", n, (unsigned)channel);
}

static void printOffsets(const MPUOffsets &o) {
    Serial.printf("Offsets do acelerômetro: X=%d Y=%d Z=%d\n", o.accelX, o.accelY, o.accelZ);
    Serial.printf("Offsets do giroscópio:   X=%d Y=%d Z=%d\n", o.gyroX,  o.gyroY,  o.gyroZ);
}

// Executa a calibração do MPU e salva os offsets na NVS. Retorna true em caso de sucesso.
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

    // prefs.begin(PREF_NAMESPACE, false);
    // size_t wrote = prefs.putBytes(PREF_KEY_OFFS, &offs, sizeof(MPUOffsets));
    // prefs.end();
    return true;
}

// ─── Callbacks BLE ───────────────────────────────────────────────────────────

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
        Serial.print("Escrita em SECTIONS (bytes):");
        for (unsigned char c : val)
            Serial.printf(" %u", (unsigned)c);
        Serial.println();
        // prefs.begin(PREF_NAMESPACE, false);
        // prefs.putBytes(PREF_KEY_SECTIONS, val.data(), val.size());
        // prefs.end();
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
        // prefs.begin(PREF_NAMESPACE, false);
        // prefs.putUChar(PREF_KEY_DIR, b);
        // prefs.end();
        Serial.printf("DIR recebido: %u → flip_gyro=%s\n",
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
        pAccelSensChar->setValue((uint8_t *)&accelThreshold, sizeof(int32_t));
        // prefs.begin(PREF_NAMESPACE, false);
        // prefs.putInt(PREF_KEY_SENS, accelThreshold);
        // prefs.end();
        Serial.printf("Novo accel threshold recebido: %ld\n", (long)accelThreshold);
    }
};

class CalibrateCharCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *, NimBLEConnInfo &) override {
        calibrate_requested = true;
    }
};

// ─── Configuração ────────────────────────────────────────────────────────────

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

    // ── Limpa a NVS (memória persistente) ────────────────────────────────────
    prefs.begin(PREF_NAMESPACE, false);
    prefs.clear();
    prefs.end();
    Serial.println("NVS limpa.");
    // ─────────────────────────────────────────────────────────────────────────

    // // ── Carrega todas as configurações persistidas em uma sessão NVS ──
    // prefs.begin(PREF_NAMESPACE, true);

    // MPUOffsets offsets;
    // bool have_offsets = prefs.getBytes(PREF_KEY_OFFS, &offsets, sizeof(MPUOffsets)) == sizeof(MPUOffsets);
    // if (have_offsets) {
    //     mpu.setXAccelOffset(offsets.accelX);
    //     mpu.setYAccelOffset(offsets.accelY);
    //     mpu.setZAccelOffset(offsets.accelZ);
    //     mpu.setXGyroOffset(offsets.gyroX);
    //     mpu.setYGyroOffset(offsets.gyroY);
    //     mpu.setZGyroOffset(offsets.gyroZ);
    //     Serial.println("Offsets carregados e aplicados:");
    //     printOffsets(offsets);
    // }

    // accelThreshold = prefs.getInt(PREF_KEY_SENS, DEFAULT_ACCEL_THRESHOLD);
    // Serial.printf("Accel threshold carregado da NVS: %ld\n", (long)accelThreshold);

    // uint8_t stored_dir = prefs.getUChar(PREF_KEY_DIR, 1);

    // std::string saved_sections;
    // size_t sections_size = prefs.getBytes(PREF_KEY_SECTIONS, nullptr, 0);
    // if (sections_size > 0) {
    //     std::vector<char> buf(sections_size);
    //     prefs.getBytes(PREF_KEY_SECTIONS, buf.data(), sections_size);
    //     saved_sections.assign(buf.begin(), buf.end());
    // }

    // prefs.end();
    // // ── Fim da sessão NVS ──

    uint8_t stored_dir = 1;
    std::string saved_sections;

    Serial.println("Nenhum offset salvo; calibrando agora...");
    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);

    // ── Configuração BLE ──
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

    pStatusChar = pMainService->createCharacteristic(
        STATUS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    pStatusChar->setCallbacks(new NotifyCharCallbacks());

    pCalibrateChar = pMainService->createCharacteristic(
        CALIBRATE_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    pCalibrateChar->setCallbacks(new CalibrateCharCallbacks());

    pMainService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(MAIN_SERVICE_UUID);
    pAdvertising->enableScanResponse(true);
    pAdvertising->setName(DEVICE_NAME);
    pAdvertising->start();

    if (!saved_sections.empty()) {
        pSectionsChar->setValue(saved_sections);
    } else {
        Serial.println("Nenhuma nota encontrada ao iniciar; usando padrão.");
        std::string def(DEFAULT_SECTION_COUNT, (char)DEFAULT_NOTE);
        pSectionsChar->setValue(def);
    }

    pAccelSensChar->setValue((uint8_t *)&accelThreshold, sizeof(int32_t));
    pDirChar->setValue(stored_dir != 0);

    Serial.println("Anúncio BLE iniciado");
    dmp_ready = true;
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
    if (!dmp_ready) return;

    if (calibrate_requested) {
        calibrate_requested = false;
        calibrateAndSaveOffsets();
    }

    unsigned long now = millis();
    if (now - lastSent < STATUS_INTERVAL_MS) return;
    lastSent = now;

    if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) return;

    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifo_buffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    // Mapeia o ângulo de rolagem [-89°, +89°] para seleção de nota
    int gyro = (int)(ypr[2] * -180.0f / M_PI);
    gyro = (int)clamp((float)gyro, GYRO_MAX_DEG, -GYRO_MAX_DEG);

    // Inverte direção opcionalmente (configurado via BLE)
    std::string dir = pDirChar->getValue();
    if (dir.size() >= 1 && (bool)dir[0]) gyro = -gyro;

    bool touch = touchRead(TOUCH_PIN) < TOUCH_THRESHOLD;
    int  accel = aaReal.x / 10;

    // Mapeia a posição do giroscópio para índice de nota no array configurado
    std::string notes = pSectionsChar->getValue();
    int section = (int)((-gyro + GYRO_MAX_DEG) / (2.0f * GYRO_MAX_DEG) * notes.length());
    if (section >= (int)notes.length())
        section = (int)notes.length() - 1;  // limita ao índice válido
    if (section < 0) section = 0;

    byte currentNote = notes.empty() ? DEFAULT_NOTE : (byte)notes[section];

    // Lógica de nota por toque
    if (touch) {
        if (!touchFlag) {
            playNote(currentNote);
            touchFlag = true;
        }
        if (currentNote != lastNote) {
            stopNote(lastNote);
            delay(10);
            playNote(currentNote);
        }
    } else {
        if (touchFlag) {
            stopNote(lastNote);
            touchFlag = false;
        }
    }

    // Lógica de percussão por acelerômetro
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

    // Envia notificação de status se um cliente estiver conectado
    if (pServer->getConnectedCount()) {
        statusPkt.gyro_x  = (int16_t)gyro;
        statusPkt.accel_x = (int16_t)accel;
        statusPkt.touch   = (uint8_t)touch;
        pStatusChar->setValue((uint8_t *)&statusPkt, sizeof(StatusPacket));
        pStatusChar->notify();
    }
}
