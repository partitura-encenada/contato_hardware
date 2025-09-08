/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include <EEPROM.h>

const int   touch_sensitivity = 30; //20  

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

uint8_t broadcastAddress[] = {0x40, 0x22, 0xd8, 0x4f, 0x5f, 0xd8};
// Base 3: 0xcc, 0xdb, 0xa7, 0x91, 0x47, 0xe8
// Base 4: 0xb0, 0xa7, 0x32, 0xd7, 0x58, 0x7c
// Base 5: 0x40, 0x22, 0xd8, 0x4f, 0x5f, 0xd8
// Base 6: 0x84, 0xcc, 0xa8, 0x5d, 0x63, 0x90
// Base 7: 0xcc, 0xdb, 0xa7, 0x91, 0x6d, 0x9c
// Base 8: 0xd8, 0xbc, 0x38, 0xe5, 0x3f, 0x8c
// Base 9: 0x78, 0xe3, 0x6d, 0xd8, 0x16, 0xd4

typedef struct struct_message {
    int id; 
    int yaw;
    int pitch;
    int roll;
    int accel_x;
    int accel_y;
    int accel_z;
    int touch;
} struct_message;

struct_message message;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Pacote enviado" : "Falha no envio");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock
  Serial.begin(115200);
  Serial.println(F("Inicializando dispositivos I2C..."));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("Conexão com o MPU6050 bem sucedida!") : F("Conexão com o MPU6050 falhou!"));
  Serial.println(F("Inicializando DMP..."));
  dev_status = mpu.dmpInitialize();

  if (dev_status == 0) { // Sucesso
      Serial.println(F("Habilitando DMP..."));
      mpu.setDMPEnabled(true);

      //  Usando EEPROM para salvar offsets na memória persistente assim que a célula 0 estiver vazia
      EEPROM.begin(128);
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
    }
  else
    {
      Serial.print(F("Inicialização do DMP falhou! (código "));
      Serial.print(dev_status);
      Serial.println(F(")"));
    }

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro inicializando ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha em adicionar peer");
    return;
  }
}


void loop() {
    message.id = 5;  // NAO É ASSIM QUE SE IDENTIFICA DISPOSITIVOS --- ta bravo é?
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) { 
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifo_buffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); 
    
        // message.yaw = ypr[0] * 180/M_PI;
        // message.pitch = ypr[1] * 180/M_PI;
        message.roll = ypr[2] * 180/M_PI;
        message.accel_x = aaReal.x;
        // message.accel_y = aaReal.y;
        // message.accel_z = aaReal.z;
        message.touch = 1 ? touchRead(T3) < touch_sensitivity : 0;
        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
    }
    delay(10);
}

