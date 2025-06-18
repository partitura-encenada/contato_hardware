// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"

// Constantes e pseudo-constantes
#define DEBUG
#define AUTO_CALLIBRATION
const int   touch_sensitivity = 20; //20  
const int   callibration_time = 6; //6

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
uint8_t     broadcastAddress[] = {0xb0, 0xa7, 0x32, 0xdc, 0xdd, 0x88};
// Base 3 ---- 0xB0, 0xA7, 0x32, 0xDE, 0xAF, 0x18
// Base 4 ---- 0x40, 0x22, 0xD8, 0x4F, 0x5F, 0xD8
// Base 5 ---- 0xA8, 0x42, 0xE3, 0x45, 0x95, 0xE8
// Base 6 ---- 0xB0, 0xA7, 0x32, 0xD7, 0x58, 0x7C
// Base 7 ---- 0xA0, 0xDD, 0x6C, 0x0F, 0xBB, 0x3C
// Base 8 ---- 0x8C, 0x4B, 0x14, 0x9A, 0x69, 0x70
// Base 9 ---- 0x8C, 0x4B, 0x14, 0xDA, 0xBA, 0x00
// 

typedef struct { // Struct da mensagem, deve ser igual ao da base 
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  int16_t accel;
  int8_t touch;
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
        mpu.setXGyroOffset(220);    // Cada sensor tem um valor diferente
        mpu.setYGyroOffset(76);     // Para consultar os valores, calibre e 
        mpu.setZGyroOffset(-85);    // mpu.PrintActiveOffsets
        mpu.setXAccelOffset();      // valores na ordem 
        mpu.setYAccelOffset();      //
        mpu.setZAccelOffset(1788);  //
    #endif
 
    if (dev_status == 0) { // Sucesso
        #ifdef AUTO_CALLIBRATION
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
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
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    // esp_now_register_send_cb(OnDataSent); // Registro função callback de envio 
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
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
        message.yaw =   ypr[0] * 180/M_PI;      // -180º >=     yaw     <= +180º
        message.pitch = ypr[1] * 180/M_PI;      // -180º >=     pitch   <= +180º
        message.roll =  ypr[2] * 180/M_PI;      // -180º >=     roll    <= +180º
        message.accel = aaReal.x;
        message.touch = touch();
        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message)); // Casta pointer para uint8_t e envia mensagem para peer 
        
        #ifdef DEBUG
            Serial.println(
                String(message.yaw) + '\t' +
                String(message.pitch) + '\t' +
                String(message.roll) + '\t' +
                String(message.accel) + '\t' +
                String(message.touch));
            //delay(50);
        #endif
    }  
}

int touch(){
    int sum_n = 0;
    int n = 10;
    for(int i = 0; i < n; i++){
      sum_n += touchRead(T3);
    }
    int avg =  sum_n/n;
    if (avg  < 20){
      return 1; // normal
    }
    if (avg < 60){
      return 2; // pianissimo
    }
    else
    {
      return 0; // off
    }
  }
