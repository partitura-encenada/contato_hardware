// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "../util/util.h"
#include <EEPROM.h>

// Constantes e pseudo-constantes
const int   touch_sensitivity = 20; //20  

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

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // Função callback de envio

// }

void setup() {
    setCpuFrequencyMhz(80);
    // Inicializar Wire, Serial (caso monitorando) e MPU
    WiFi.mode(WIFI_STA); // Wi-fi Station
    Util::PrintMACAddr(); 
    Wire.begin();
    Wire.setClock(400000); // Clock I2C 400khz. Comente caso erro na compilação 
    Serial.begin(115200);
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));  

    // DMP
    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);       
 
    if (dev_status == 0) { // Sucesso

        //  TENTANDO SALVAR OFFSETS MEMORIA PERMANENTE DO ESP XD
        EEPROM.begin(128);
        if (EEPROM.readShort(0) == 0)
        {
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
            mpu.setZAccelOffset(EEPROM.readShort(2 * 16));
            mpu.setXGyroOffset(EEPROM.readShort(3 * 16));
            mpu.setYGyroOffset(EEPROM.readShort(4 * 16));
            mpu.setZGyroOffset(EEPROM.readShort(5 * 16));
        }
        // EEPROM.writeShort(0,0); // Descomente para resetar a calibração
        EEPROM.end();

        dmp_ready = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); // Erro
        Serial.print(dev_status); // 1 = "initial memory load failed"; 2 = "DMP configuration updates failed"
    }
}

void loop() {
    if (!dmp_ready) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) { 
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifo_buffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        Serial.println(
            String(ypr[0] * 180/M_PI) + '\t' +
            String(ypr[1] * 180/M_PI) + '\t' +
            String(ypr[2] * 180/M_PI) + '\t' +
            String(aaReal.x) + '\t') + 
            String(touchRead(T3));
        delay(10);
    }  
}
