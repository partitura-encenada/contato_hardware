// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h" 

// Constantes e pseudo-constantes
#define DEBUG
// #define AUTO_CALLIBRATION
const int   touch_sensitivity = 30; //20  
const int   callibration_time = 6; //6  
const int   CANAL_ESPECIFICO = 1;

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
uint8_t     broadcastAddress[] = {0x78, 0xe3, 0x6d, 0xd8, 0x16, 0xd4};

typedef struct { // Struct da mensagem, deve ser igual ao da base 
    int id = 9;
    int roll;
    int accel;
    int touch;
} message_t;
message_t message;
esp_now_peer_info_t peerInfo;

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // Função callback de envio

// }

void setup() {
    setCpuFrequencyMhz(80);
    // Inicializar Wire, Serial (caso monitorando) e MPU
    Wire.begin();
    Wire.setClock(400000); // Clock I2C 400khz. Comente caso erro na compilação 
    #ifdef DEBUG
        Serial.begin(115200);
    #endif
    mpu.initialize();
    #ifdef DEBUG
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));  
    #endif    

    // DMP
    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);       
    #ifndef AUTO_CALLIBRATION
        mpu.setZAccelOffset(1718);
        mpu.setXGyroOffset(-30);    
        mpu.setYGyroOffset(-20);    
        mpu.setZGyroOffset(19); 
    #endif
 
    if (dev_status == 0) { // Sucesso
        #ifdef AUTO_CALLIBRATION
            mpu.CalibrateAccel(callibration_time);
            mpu.CalibrateGyro(callibration_time);
            mpu.PrintActiveOffsets();
        #endif
        dmp_ready = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); // Erro
        Serial.print(dev_status); // 1 = "initial memory load failed"; 2 = "DMP configuration updates failed"
    }

    // ESP_NOW
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(82);
    WiFi.channel(CANAL_ESPECIFICO); // Para canal fixo
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // esp_now_register_send_cb(OnDataSent); // Registro função callback de envio 
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = CANAL_ESPECIFICO;  
    peerInfo.encrypt = false;       
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
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
        // message.yaw =   ypr[0] * 180/M_PI;      // -180º >=     yaw     <= +180º
        // message.pitch = ypr[1] * 180/M_PI;      // -180º >=     pitch   <= +180º
        message.roll =  ypr[2] * 180/M_PI;      // -180º >=     roll    <= +180º
        message.accel = aaReal.x;
        message.touch = (touchRead(T3) < 30) ? 1 : 0; //mudança message.touch = 1 ? touchRead(T3) < 20 : 0;
        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message)); // Casta pointer para uint8_t e envia mensagem para peer 

    }  
}
