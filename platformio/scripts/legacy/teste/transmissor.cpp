// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h" 

// Constantes e pseudo-constantes
#define DEBUG
// #define AUTO_CALLIBRATION
const int   touch_sensitivity = 30;
const int   callibration_time = 6;
const int   CANAL_ESPECIFICO = 5;

MPU6050 mpu;

uint8_t     dev_status;      
uint16_t    packet_size;   
uint16_t    fifo_count;    
uint8_t     fifo_buffer[64];
Quaternion  q;              
VectorInt16 aa;             
VectorInt16 aaReal;         
VectorFloat gravity;        
bool        dmp_ready = false;  
float       ypr[3];          
uint8_t     broadcastAddress[] = {0xb0, 0xa7, 0x32, 0xd7, 0x58, 0x7c};

typedef struct { 
    int id = 4;
    int roll;       // roll em graus
    float accel;    // aceleração total em m/s²
    int touch;      // estado do touch
    float gyroX;    // giroscópio X em rad/s
    float gyroY;    // giroscópio Y em rad/s
    float gyroZ;    // giroscópio Z em rad/s
} message_t;
message_t message;
esp_now_peer_info_t peerInfo;

// Função para definir o canal
esp_err_t setChannel(int channel) {
  esp_wifi_set_promiscuous(true);  
  esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  #ifdef DEBUG
    uint8_t primaryChan;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);
    Serial.print("Canal real configurado: ");
    Serial.println(primaryChan);
  #endif

  return result;
}

void setup() {
    setCpuFrequencyMhz(80);
    Wire.begin();
    Wire.setClock(400000); 
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
        mpu.setZAccelOffset(1794); 
        mpu.setXGyroOffset(44);    
        mpu.setYGyroOffset(2);     
        mpu.setZGyroOffset(-3);    
    #endif
 
    if (dev_status == 0) { 
        #ifdef AUTO_CALLIBRATION
            mpu.CalibrateAccel(callibration_time);
            mpu.CalibrateGyro(callibration_time);
            mpu.PrintActiveOffsets();
        #endif
        dmp_ready = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); 
        Serial.print(dev_status); 
    }

    // Configura Wi-Fi e ESP-NOW
    WiFi.mode(WIFI_STA);           
    setChannel(CANAL_ESPECIFICO);
    esp_wifi_set_max_tx_power(82);

    Serial.print("MAC deste dispositivo: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    esp_now_peer_info_t peerInfo = {};
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

        // Roll em graus
        message.roll =  ypr[2] * 180/M_PI;

        // Conversão LSB → m/s² para cada eixo
        float ax = (aaReal.x / 16384.0) * 9.80665;
        float ay = (aaReal.y / 16384.0) * 9.80665;
        float az = (aaReal.z / 16384.0) * 9.80665;

        // Aceleração total (magnitude do vetor)
        message.accel = sqrt(ax*ax + ay*ay + az*az);

        // Touch
        message.touch = (touchRead(T3) < touch_sensitivity) ? 1 : 0;

        // Giroscópio em rad/s
        message.gyroX = (aa.x / 131.0) * PI / 180.0;
        message.gyroY = (aa.y / 131.0) * PI / 180.0;
        message.gyroZ = (aa.z / 131.0) * PI / 180.0;

        // Debug serial
        #ifdef DEBUG
            Serial.print("Touch raw: "); Serial.print(touchRead(T3));
            Serial.print(" | Accel total (m/s²): "); Serial.print(message.accel);
            Serial.print(" | Gyro X: "); Serial.print(message.gyroX);
            Serial.print(" Y: "); Serial.print(message.gyroY);
            Serial.print(" Z: "); Serial.println(message.gyroZ);
        #endif

        // Envia a mensagem via ESP-NOW
        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));

        delay(20);
    }  
}
