//LIBRARIES
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include <EEPROM.h>

//MPU Initialization
MPU6050 mpu;
bool dmpReady = false;  
uint8_t dev_status;      
uint16_t fifo_count;    
uint8_t fifo_buffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll continer

uint8_t broadcastAddress[] = {0xcc, 0xdb, 0xa7, 0x91, 0x5d, 0xbc};

typedef struct struct_message {
    int id; 
    int gyro;
    int accel;
    int touch;
} struct_message;
struct_message message;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Pacote enviado" : "Falha no envio");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  dev_status = mpu.dmpInitialize();

  if (dev_status == 0) { // Sucesso
      //  Usando EEPROM para salvar offsets na memória persistente assim que a célula 0 estiver vazia
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
          // Aplicando offsets ao IMU
          mpu.setZAccelOffset(EEPROM.readShort(2 * 16));
          mpu.setXGyroOffset(EEPROM.readShort(3 * 16));
          mpu.setYGyroOffset(EEPROM.readShort(4 * 16));
          mpu.setZGyroOffset(EEPROM.readShort(5 * 16));
      }
      // EEPROM.writeShort(0,0); // Descomente para resetar a calibração
      EEPROM.end();
    }

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() {
    message.id = 5; 
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) { 
      
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifo_buffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); 
    
        message.gyro = ypr[2] * 180/M_PI;
        message.accel = aaReal.x;
        message.touch = 1 ? touchRead(T3) < 30 : 0;
        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
    }
    delay(10);
}

