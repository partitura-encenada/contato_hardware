// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <NimBLEDevice.h>
#include "Wire.h"
#include "../util/util.h"
#include <WiFi.h>
#include <EEPROM.h>

//  https://www.uuidgenerator.net/
#define SERVICE_UUID "cc5b7017-78da-4891-a348-569271d5f67c"
#define TOUCH_CHARACTERISTIC_UUID "62c84a29-95d6-44e4-a13d-a9372147ce21"
#define GYRO_CHARACTERISTIC_UUID "9b7580ed-9fc2-41e7-b7c2-f63de01f0692"
#define ACCEL_CHARACTERISTIC_UUID "f62094cf-21a7-4f71-bb3f-5a5b17bb134e"

const int   touch_sensitivity = 30; //20  

static NimBLEServer* pServer;
MPU6050 mpu;

uint8_t     dev_status;      
uint16_t    packet_size;   
uint16_t    fifo_count;    
uint8_t     fifo_buffer[64];
Quaternion  q;              // [w, x, y, z]         Quaternion 
VectorInt16 aa;             // [x, y, z]            Accel
VectorInt16 aaReal;         // [x, y, z]            Accel sem gravidade
VectorFloat gravity;        // [x, y, z]            Gravidade
bool        dmp_ready = false;  
float       ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll


class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

        /**
         *  Args: connection handle, intervalo de conexão mínimo (incremento de 1.25ms), intervalo de conexão máximo (incremento de 1.25ms)
         *  latência (intervalos que podem sofrer skip), timeout (incremento de 10ms).
        */
        pServer->updateConnParams(connInfo.getConnHandle(), 30, 40, 0, 180);
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("Client disconnected - start advertising\n");
        NimBLEDevice::startAdvertising();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
    }
} serverCallbacks;

void setup() {
    // Inicializar Wire, Serial (caso monitorando) e MPU
    Wire.begin();
    Wire.setClock(400000); // Clock I2C 400khz. Comente caso erro na compilação 
    Serial.begin(115200);
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));  
    WiFi.mode(WIFI_STA); // Wi-fi Station, apenas para obter endereço MAC
    Util::PrintMACAddr(); 
    
    // DMP
    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);           
 
    if (dev_status == 0) { // Sucesso
        //  Usando EEPROM para salvar offsets na memória persistente assim que a célula 0 estiver vazia
        EEPROM.begin(128); // 128 bytes de espaço
        if (EEPROM.readShort(0) == 0)
        {
            // Calibra durante 6 segundos
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            int16_t* offsets = mpu.GetActiveOffsets();
            for (int i = 0; i < 6; i++)
            {
                EEPROM.writeShort(i*16, offsets[i]);
            }
        }
        else 
        {   
            // Aplicando offsets ao IMU
            mpu.setZAccelOffset(EEPROM.readShort(2 * 16));
            mpu.setXGyroOffset(EEPROM.readShort(3 * 16));
            mpu.setYGyroOffset(EEPROM.readShort(4 * 16));
            mpu.setZGyroOffset(EEPROM.readShort(5 * 16));
        }
        // EEPROM.writeShort(0,0); // Esvazia a célula 0 para reiniciar escrita no EEPROM
        EEPROM.end();


        dmp_ready = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); // Erro
    }

    // BLE
    NimBLEDevice::init("Contato");
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);
    NimBLEDevice::setMTU(24); // Não sei exatamente o que faz, parece estar associado com o tamanho do pacote enviado
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);

    NimBLEService* pSensorService = pServer->createService(SERVICE_UUID);

    // Cria características a partir de seus UUID. Ter os UUID "hard-coded" não é uma boa ideia mas é a solução provisória
    NimBLECharacteristic* pTouchCharacteristic = pSensorService->createCharacteristic(TOUCH_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* pGyroCharacteristic = pSensorService->createCharacteristic(GYRO_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* pAccelCharacteristic = pSensorService->createCharacteristic(ACCEL_CHARACTERISTIC_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLE2904* pGyro2904 = pGyroCharacteristic->create2904();
    pGyro2904->setFormat(NimBLE2904::FORMAT_SINT16);
    pSensorService->start();

    /** Anuncia a conexão */
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName("Contato");
    pAdvertising->addServiceUUID(pSensorService->getUUID());
    pAdvertising->enableScanResponse(false);
    pAdvertising->start();
    Serial.printf("Anunciando:\n");
}
    
void loop() {
    if (!dmp_ready) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)){ 
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifo_buffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        if (pServer->getConnectedCount()) {
            NimBLEService* pSvc = pServer->getServiceByUUID(SERVICE_UUID);
            if (pSvc) {
                NimBLECharacteristic* pTouchChr = pSvc->getCharacteristic(TOUCH_CHARACTERISTIC_UUID);
                NimBLECharacteristic* pGyroChr = pSvc->getCharacteristic(GYRO_CHARACTERISTIC_UUID);                
                NimBLECharacteristic* pAccelChr = pSvc->getCharacteristic(ACCEL_CHARACTERISTIC_UUID);                
                if (pTouchChr) {
                    pTouchChr->setValue(1 ? touchRead(T3) < touch_sensitivity : 0);
                    pTouchChr->notify();
                }
                if (pGyroChr) {
                    pGyroChr->setValue((int)(ypr[2] * 180/M_PI));
                    pGyroChr->notify();
                }
                if (pAccelChr) {
                    pAccelChr->setValue((int)aaReal.x);
                    pAccelChr->notify();
                }
            }
        }
    }
    delay(10);
}