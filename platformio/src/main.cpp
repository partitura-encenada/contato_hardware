// updated main.cpp
// based on your original file (you provided /mnt/data/main.cpp). See original for provenance. :contentReference[oaicite:2]{index=2}

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32_Nimble.h>
#include <NimBLEDevice.h>

#define LED_PIN 2 //

Preferences prefs;
static const char *PREF_NAMESPACE = "mpu";
static const char *PREF_KEY_OFFS = "offs";
static const char *PREF_KEY_SECTIONS = "sections";
static const char *PREF_KEY_SENS = "sens";
static const char *PREF_KEY_DIR = "dir";

static const char *DEVICE_NAME = "Contato";
static const char *MAIN_SERVICE_UUID = "886520c2-76cb-4924-a033-729914e5bd76";
static const char *SECTIONS_CHAR_UUID = "251beea3-1c81-454f-a9dd-8561ec692ded";
static const char *ACCEL_SENS_CHAR_UUID = "c7f2b2e2-1a2b-4c3d-9f0a-123456abcdef";
static const char *GYRO_CHAR_UUID = "f8d968fe-99d7-46c4-a61c-f38093af6ec8";
static const char *ACCEL_CHAR_UUID = "d3b8a1f1-9c4f-4c9b-8f1e-abcdef123456";
static const char *TOUCH_CHAR_UUID = "55558523-eca8-4b78-ae20-97ed68c68c26";
static const char *DIR_CHAR_UUID = "a1b2c3d4-0001-4b33-a751-6ce34ec4c701";
static const char *LEGATO_CHAR_UUID = "a1b2c3d4-0002-4b33-a751-6ce34ec4c702";
static const char *CALIBRATE_CHAR_UUID = "b4d0c9f8-3b9a-4a4e-93f2-2a8c9f5ee7a2";

BLEMIDI_CREATE_INSTANCE(DEVICE_NAME, MIDI);
static NimBLEServer *pServer;
NimBLECharacteristic *pSectionsChar = nullptr;
NimBLECharacteristic *pGyroChar = nullptr;
NimBLECharacteristic *pAccelChar = nullptr;
NimBLECharacteristic *pAccelSensChar = nullptr;
NimBLECharacteristic *pTouchChar = nullptr;
NimBLECharacteristic *pDirChar = nullptr;
NimBLECharacteristic *pLegatoChar = nullptr;
NimBLECharacteristic *pCalibrateChar = nullptr;

MPU6050 mpu;
struct MPUOffsets
{
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};
uint16_t packet_size;
uint16_t fifo_count;
uint8_t fifo_buffer[64];
Quaternion q;        // [w, x, y, z]         Quaternion
VectorInt16 aa;      // [x, y, z]            Aceleração bruta
VectorInt16 aaReal;  // [x, y, z]            Aceleração sem gravidade
VectorFloat gravity; // [x, y, z]            Vetor gravidade
bool dmp_ready = false;
float ypr[3]; // [yaw, pitch, roll]

const unsigned long NOTE_INTERVAL_MS = 3;
const unsigned long ACCEL_DURATION_MS = 2000;
unsigned long lastSent = 0;
unsigned long lastAccel = 0;
bool touchFlag = false;
bool accelFlag = false;
byte lastNote;

// NEW: accel threshold (persisted)
int32_t accelThreshold = 10000; // default 10000 (will be loaded from prefs if present)

// helper functions
float clamp(float v, float max_v, float min_v)
{
  if (v > max_v)
    return max_v;
  if (v < min_v)
    return min_v;
  return v;
}

void playNote(byte n, uint8_t channel = 1)
{
  lastNote = n;
  if (pServer->getConnectedCount())
    MIDI.sendNoteOn(n, 80, channel);
  Serial.printf("Tocando nota: %d (ch %u)\n", n, (unsigned)channel);
}

void stopNote(byte n, uint8_t channel = 1)
{
  lastNote = n;
  if (pServer->getConnectedCount())
    MIDI.sendNoteOff(n, 80, channel);
  Serial.printf("Parando nota: %d (ch %u)\n", n, (unsigned)channel);
}

void printOffsets(const MPUOffsets &o)
{
  Serial.printf("Offsets do acelerômetro: X=%d Y=%d Z=%d\n", (int)o.accelX, (int)o.accelY, (int)o.accelZ);
  Serial.printf("Offsets do giroscópio: X=%d Y=%d Z=%d\n", (int)o.gyroX, (int)o.gyroY, (int)o.gyroZ);
}

bool calibrateAndSaveOffsets()
{
  Serial.println("Iniciando calibração");
  mpu.CalibrateGyro(6);
  mpu.CalibrateAccel(6);

  MPUOffsets offs;
  offs.accelX = (int16_t)mpu.getXAccelOffset();
  offs.accelY = (int16_t)mpu.getYAccelOffset();
  offs.accelZ = (int16_t)mpu.getZAccelOffset();
  offs.gyroX = (int16_t)mpu.getXGyroOffset();
  offs.gyroY = (int16_t)mpu.getYGyroOffset();
  offs.gyroZ = (int16_t)mpu.getZGyroOffset();

  Serial.println("Calibração concluída. Offsets:");
  printOffsets(offs);

  prefs.begin(PREF_NAMESPACE, false);
  size_t wrote = prefs.putBytes(PREF_KEY_OFFS, &offs, sizeof(MPUOffsets));
  if (wrote == sizeof(MPUOffsets))
  {
    Serial.println("Offsets salvos na NVS.");
    return true;
  }
  else
  {
    Serial.printf("ERRO salvando offsets: escreveu %u de %u\n", (unsigned)wrote, (unsigned)sizeof(MPUOffsets));
    return false;
  }
}

// ----------------------------- Callbacks BLE -----------------------------
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
  {
    Serial.println("Client conectado");
    digitalWrite(LED_PIN, HIGH); // acende LED ao conectar
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    Serial.println("Client desconectado, anunciando:");
    digitalWrite(LED_PIN, LOW);       // apaga LED ao desconectar
    NimBLEDevice::startAdvertising(); // reinicia anúncio
  }
};

class SectionsCharCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
  {
    std::string val = pChar->getValue();
    Serial.print("Escrita em SECTIONS (bytes): ");
    for (unsigned char c : val)
    {
      Serial.printf("%u ", (unsigned)c);
    }
    Serial.println();

    // Salva imediatamente em armazenamento persistente
    prefs.putBytes(PREF_KEY_SECTIONS, val.data(), val.size());
    Serial.println("SECTIONS salvo na NVS.");
  }
};

class NotifyCharCallbacks : public NimBLECharacteristicCallbacks
{
  void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
  {
    Serial.printf("Cliente inscrito em %s (subValue=%u)\n", pCharacteristic->getUUID().toString().c_str(), subValue);
  }
};

class DirCharCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
  {
    std::string v = pChar->getValue();
    if (v.size() >= 1)
    {
      uint8_t b = (uint8_t)v[0];
      // persist immediately
      prefs.putUChar(PREF_KEY_DIR, b);
      Serial.printf("DIR write received: %u → flip_gyro=%s (saved)\n", (unsigned)b, b ? "true" : "false");
      // update the characteristic value so reads return the same byte
      pChar->setValue(&b, 1);
    }
  }
};

// class LegatoCharCallbacks : public NimBLECharacteristicCallbacks
// {
//   void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
//   {
//     std::string v = pChar->getValue();
//     if (v.size() >= 1)
//     {
//       uint8_t b = (uint8_t)v[0];
//       // persist immediately
//       prefs.putUChar(PREF_KEY_LEGATO, b);
//       Serial.printf("LEGATO write received: %u → legato_mode=%s (saved)\n", (unsigned)b, b ? "true" : "false");
//       // update characteristic so clients that read immediately will see current value
//       pChar->setValue(&b, 1);
//     }
//   }
// };

class CalibrateCharCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
  {
    calibrateAndSaveOffsets();
  }
};

class AccelSensCharCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
  {
    std::string val = pChar->getValue();
    if (val.size() == 2)
    {
      int16_t vv = 0;
      memcpy(&vv, val.data(), sizeof(int16_t));
      accelThreshold = vv;
      // Salva imediatamente em armazenamento persistente
      prefs.begin(PREF_NAMESPACE, false);
      prefs.putInt(PREF_KEY_SENS, accelThreshold);
      prefs.end();
      Serial.printf("Novo accel threshold (16-bit) recebido: %ld (salvo)\n", (long)accelThreshold);
    }
    else
    {
      Serial.println("Escrita em accel sens com tamanho inesperado.");
    }
  }
};
// ----------------------------- Fim callbacks BLE -----------------------------

void setup()
{
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Inicializando MPU6050...");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "Conexão com MPU bem-sucedida" : "Conexão com MPU FALHOU");
  uint8_t dev_status = mpu.dmpInitialize();
  if (dev_status == 0)
  {
    mpu.setDMPEnabled(true);
    Serial.println("DMP inicializado e ativado.");

    MPUOffsets offsets;
    prefs.begin(PREF_NAMESPACE, true);
    size_t size = prefs.getBytes(PREF_KEY_OFFS, &offsets, sizeof(MPUOffsets));
    if (size == sizeof(MPUOffsets))
    {
      // aplica offsets
      mpu.setXAccelOffset((int)offsets.accelX);
      mpu.setYAccelOffset((int)offsets.accelY);
      mpu.setZAccelOffset((int)offsets.accelZ);
      mpu.setXGyroOffset((int)offsets.gyroX);
      mpu.setYGyroOffset((int)offsets.gyroY);
      mpu.setZGyroOffset((int)offsets.gyroZ);
      Serial.println("Offsets carregados e aplicados:");
      printOffsets(offsets);
    }
    // load accel threshold (if present)
    accelThreshold = prefs.getInt(PREF_KEY_SENS, accelThreshold);
    Serial.printf("Accel threshold carregado da NVS: %ld\n", (long)accelThreshold);

    // Inicializa NimBLE
    NimBLEDevice::init(DEVICE_NAME);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    NimBLEService *pMainService = pServer->createService(MAIN_SERVICE_UUID);

    pSectionsChar = pMainService->createCharacteristic(
        SECTIONS_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pSectionsChar->setCallbacks(new SectionsCharCallbacks());

    pAccelSensChar = pMainService->createCharacteristic(
        ACCEL_SENS_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pAccelSensChar->setCallbacks(new AccelSensCharCallbacks());

    pDirChar = pMainService->createCharacteristic(
        DIR_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    pDirChar->setCallbacks(new DirCharCallbacks());

    // pLegatoChar = pMainService->createCharacteristic(
    //     LEGATO_CHAR_UUID,
    //     NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    // pLegatoChar->setCallbacks(new LegatoCharCallbacks());

    pGyroChar = pMainService->createCharacteristic(
        GYRO_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY);
    pGyroChar->setCallbacks(new NotifyCharCallbacks());

    pTouchChar = pMainService->createCharacteristic(
        TOUCH_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY);
    pTouchChar->setCallbacks(new NotifyCharCallbacks());

    pAccelChar = pMainService->createCharacteristic(
        ACCEL_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY);
    pAccelChar->setCallbacks(new NotifyCharCallbacks());

    pCalibrateChar = pMainService->createCharacteristic(
        CALIBRATE_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE);
    pCalibrateChar->setCallbacks(new CalibrateCharCallbacks());

    pMainService->start();

    std::string saved_sections;
    size = prefs.getBytes(PREF_KEY_SECTIONS, nullptr, 0);
    if (size != 0)
    {
      std::vector<char> buffer(size);
      size_t read = prefs.getBytes(PREF_KEY_SECTIONS, buffer.data(), size);
      prefs.end();
      saved_sections.assign(buffer.begin(), buffer.end());
      pSectionsChar->setValue(buffer);
    }
    else
    {
      prefs.end();
      Serial.println("Nenhuma nota encontrada ao iniciar.");
      std::string def;
      for (int i = 0; i < 6; ++i)
        def.push_back((char)60);
      pSectionsChar->setValue(def);
    }

    size = prefs.getBytes(PREF_KEY_DIR, nullptr, 0);
    if (size != 0)
    {
      uint8_t stored_dir = prefs.getUChar(PREF_KEY_DIR, 0);
      pDirChar->setValue(stored_dir != 0);
    }
    else
    {
      pDirChar->setValue(true);
    }

    // size = prefs.getBytes(PREF_KEY_LEGATO, nullptr, 0);
    // if (size != 0)
    // {
    //   uint8_t stored_dir = prefs.getUChar(PREF_KEY_LEGATO, 0);
    //   pLegatoChar->setValue(stored_dir != 0);
    // }
    // else
    // {
    //   pLegatoChar->setValue(true);
    // }
    // Inicializa Service MIDI
    MIDI.begin(MIDI_CHANNEL_OMNI);
    pServer = NimBLEDevice::getServer();
    pServer->setCallbacks(new ServerCallbacks());
    Serial.println("Anúncio BLE iniciado");
  }
}

void loop()
{
  unsigned long now = millis();
  if (now - lastSent >= NOTE_INTERVAL_MS)
  {
    lastSent = now;
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer))
    {
      mpu.dmpGetQuaternion(&q, fifo_buffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetAccel(&aa, fifo_buffer);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      int gyro = ypr[2] * -180 / M_PI;
      gyro = clamp(gyro, 90, -90);
      bool touch = true ? touchRead(T3) < 30 : false;
      int accel = aaReal.x / 10;

      std::string notes = pSectionsChar->getValue();
      int section = (-gyro + 90) / 180.0 * notes.length();
      if (section > notes.length())
        section = notes.length();
      byte currentNote = notes[section];

      // Lógica de seleção de notas
      if (touch)
      {

        // Início do toque
        if (!touchFlag)
        {
          playNote(currentNote);
          touchFlag = true;
        }

        // Decorrer do toque
        if (currentNote != lastNote)
        {
          stopNote(lastNote);
          vTaskDelay(10);
          playNote(currentNote);
        }
      }
      else
      {
        // Liberação do toque
        if (touchFlag)
        {
          stopNote(lastNote);
          touchFlag = false;
        }
      }

      // Lógica do acelerômetro
      if (abs(accel) > accelThreshold && (now - lastAccel) >= ACCEL_DURATION_MS && !accelFlag)
      {
        Serial.printf("Accel triggered: value=%d threshold=%d\n", accel, accelThreshold);
        playNote(currentNote, 8); // channel 8
        accelFlag = true;
        lastAccel = now;
      }
      if (accelFlag && (now - lastAccel) >= ACCEL_DURATION_MS)
      {
        stopNote(lastNote, 8);
        accelFlag = false;
      }

      if (pServer->getConnectedCount())
      {
        pGyroChar->setValue(gyro);
        pGyroChar->notify();

        pTouchChar->setValue((uint8_t)touch);
        pTouchChar->notify();

        // notify accel (send raw int value)
        pAccelChar->setValue((int)accel);
        pAccelChar->notify();
      }
    }
  }
}
