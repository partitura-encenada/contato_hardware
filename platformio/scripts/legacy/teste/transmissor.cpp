// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h"
#include "esp_wifi_internal.h"

// --- Configuração do Long Range ---
#define LONG_RANGE_LEVEL 2   // escolha entre 1, 2, 3 ou 4

#ifndef WIFI_PHY_RATE_LR_L1
  #define WIFI_PHY_RATE_LR_L1  0x0B
  #define WIFI_PHY_RATE_LR_L2  0x0C
  #define WIFI_PHY_RATE_LR_L3  0x0D
  #define WIFI_PHY_RATE_LR_L4  0x0E
#endif

void setLongRange(int level) {
  wifi_phy_rate_t rate;
  switch (level) {
    case 1: rate = (wifi_phy_rate_t)WIFI_PHY_RATE_LR_L1; break;
    case 2: rate = (wifi_phy_rate_t)WIFI_PHY_RATE_LR_L2; break;
    case 3: rate = (wifi_phy_rate_t)WIFI_PHY_RATE_LR_L3; break;
    case 4: rate = (wifi_phy_rate_t)WIFI_PHY_RATE_LR_L4; break;
    default: return;
  }
  esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, rate);
  Serial.print("Transmissor em modo Long Range L");
  Serial.println(level);
}

// Constantes e pseudo-constantes
#define DEBUG
const int   touch_sensitivity = 20;  
const int   callibration_time = 6; 
const int   CANAL_ESPECIFICO = 5;

MPU6050 mpu;

uint8_t     dev_status;      
uint16_t    packet_size;   
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
    int roll;
    int accel;
    int touch;
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
  dev_status = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);       

  mpu.setZAccelOffset(1794); 
  mpu.setXGyroOffset(44);    
  mpu.setYGyroOffset(2);     
  mpu.setZGyroOffset(-3);    

  if (dev_status == 0) {
    dmp_ready = true;
    packet_size = mpu.dmpGetFIFOPacketSize();
  }

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

  // --- aplica o modo LR selecionado ---
  setLongRange(LONG_RANGE_LEVEL);
}

void loop() {
  if (!dmp_ready) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) { 
    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifo_buffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    message.roll =  ypr[2] * 180/M_PI;
    message.accel = aaReal.x;
    message.touch = (touchRead(T3) < touch_sensitivity) ? 1 : 0;

    esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
    delay(20);
  }  
}
