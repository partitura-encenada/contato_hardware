// ... (keep your original headers)
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

static const char *DEVICE_NAME = "Contato";
static const char *MAIN_SERVICE_UUID = "886520c2-76cb-4924-a033-729914e5bd76";
static const char *SECTIONS_CHAR_UUID = "251beea3-1c81-454f-a9dd-8561ec692ded";
static const char *GYRO_CHAR_UUID = "f8d968fe-99d7-46c4-a61c-f38093af6ec8";
static const char *TOUCH_CHAR_UUID = "55558523-eca8-4b78-ae20-97ed68c68c26";
// NEW: calibrate characteristic UUID (must match client)
static const char *CALIBRATE_CHAR_UUID = "b4d0c9f8-3b9a-4a4e-93f2-2a8c9f5ee7a2";

BLEMIDI_CREATE_INSTANCE(DEVICE_NAME, MIDI);
static NimBLEServer *pServer;
NimBLECharacteristic *pSectionsChar = nullptr;
NimBLECharacteristic *pGyroChar = nullptr;
NimBLECharacteristic *pTouchChar = nullptr;
// NEW: calibrate characteristic pointer
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

const unsigned long NOTE_INTERVAL_MS = 10;
unsigned long lastSent = 0;
bool touchFlag = false;
byte lastNote;

// helper functions unchanged...
void playNote(byte n){
  lastNote = n;
  if (pServer->getConnectedCount()) MIDI.sendNoteOn(n, 80, 1);
  Serial.printf("Tocando nota: %d\n", n);
}

void stopNote(byte n){
  lastNote = n;
  if (pServer->getConnectedCount()) MIDI.sendNoteOff(n, 80, 1);
  Serial.printf("Parando nota: %d\n", n);
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
  prefs.end();
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
    prefs.begin(PREF_NAMESPACE, false);
    prefs.putBytes(PREF_KEY_SECTIONS, val.data(), val.size());
    prefs.end();
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

class CalibrateCharCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override
  {
    calibrateAndSaveOffsets();
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
    prefs.end();
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
    else
    {
      Serial.println("Nenhum offset salvo encontrado. Executando calibração e salvando resultados...");
      if (!calibrateAndSaveOffsets())
      {
        Serial.println("Falha na calibração/salvamento.");
      }
      else
      {
        Serial.printf("Erro ao inicializar DMP (dev_status=%u). Verificação/calibração de offsets ignorada.\n", dev_status);
      }
    }
  }

  // Inicializa NimBLE
  NimBLEDevice::init(DEVICE_NAME);
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  NimBLEService *pMainService = pServer->createService(MAIN_SERVICE_UUID);

  pSectionsChar = pMainService->createCharacteristic(
      SECTIONS_CHAR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  pSectionsChar->setCallbacks(new SectionsCharCallbacks());

  pGyroChar = pMainService->createCharacteristic(
      GYRO_CHAR_UUID,
      NIMBLE_PROPERTY::NOTIFY);
  pGyroChar->setCallbacks(new NotifyCharCallbacks());

  pTouchChar = pMainService->createCharacteristic(
      TOUCH_CHAR_UUID,
      NIMBLE_PROPERTY::NOTIFY);
  pTouchChar->setCallbacks(new NotifyCharCallbacks());

  // NEW: create calibrate characteristic (write)
  pCalibrateChar = pMainService->createCharacteristic(
      CALIBRATE_CHAR_UUID,
      NIMBLE_PROPERTY::WRITE);
  pCalibrateChar->setCallbacks(new CalibrateCharCallbacks());

  pMainService->start();

  std::string saved_sections;
  prefs.begin(PREF_NAMESPACE, true);
  size_t size = prefs.getBytes(PREF_KEY_SECTIONS, nullptr, 0);
  if (size == 0)
  {
    prefs.end();
    Serial.println("Nenhuma nota encontrada ao iniciar.");
    std::string def;
    for (int i = 0; i < 6; ++i)
      def.push_back((char)60);
    pSectionsChar->setValue(def);
  }
  else
  {
    std::vector<char> buffer(size);
    size_t read = prefs.getBytes(PREF_KEY_SECTIONS, buffer.data(), size);
    prefs.end();
    saved_sections.assign(buffer.begin(), buffer.end());
    pSectionsChar->setValue(buffer);
  }

  // Inicializa Service MIDI 
  MIDI.begin(MIDI_CHANNEL_OMNI);
  pServer = NimBLEDevice::getServer();
  pServer->setCallbacks(new ServerCallbacks());
  Serial.println("Anúncio BLE iniciado");
}

void loop()
{
  unsigned long now = millis();
  if (now - lastSent >= NOTE_INTERVAL_MS) {
    lastSent = now;
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)){ 
      mpu.dmpGetQuaternion(&q, fifo_buffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetAccel(&aa, fifo_buffer);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      int gyro = (int)(ypr[2] * 180/M_PI);
      bool touch = true ? touchRead(T3) < 50 : false;

      std::string notes = pSectionsChar->getValue();
      int section = (-gyro + 90.0f) / 180 * notes.length();
      if (section == notes.length()) section = notes.length() - 1;
      byte currentNote = notes[section];
      
      // Lógica de seleção de notas
      if (touch){

        // Início do toque
        if (!touchFlag){
          playNote(currentNote);
          touchFlag = true;
        }

        // Decorrer do toque
        if (currentNote != lastNote){
          stopNote(lastNote);
          vTaskDelay(10);
          playNote(currentNote);
        }
      } else {
        // Liberação do toque
        if (touchFlag){
          stopNote(lastNote);
          touchFlag = false;
        }
      }

      if (pServer->getConnectedCount()){
        pGyroChar->setValue(gyro);
        pGyroChar->notify();
        pTouchChar->setValue(touch);
        pTouchChar->notify();
      }
    }
  }
}
