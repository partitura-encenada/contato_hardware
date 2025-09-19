#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_private/wifi.h"  // necessário para setar taxa fixa

// --- Configuração do Long Range ---
#define LONG_RANGE_LEVEL 3   // escolha entre 1, 2, 3 ou 4

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
  Serial.print("Receptor em modo Long Range L");
  Serial.println(level);
}

// Estrutura de dados recebida
typedef struct struct_message {
    int id; 
    int gyro;
    int accel;
    int touch;
} struct_message;

struct_message MIDImessage;

// Callback de recepção
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&MIDImessage, incomingData, sizeof(MIDImessage));
  Serial.println(String(MIDImessage.id) + "/" +
                 String(MIDImessage.gyro) + "/" +
                 String(MIDImessage.accel) + "/" +
                 String(MIDImessage.touch));
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(82);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  uint8_t primaryChan;
  wifi_second_chan_t secondChan;
  esp_wifi_get_channel(&primaryChan, &secondChan);
  Serial.print("Receptor no canal: ");
  Serial.println(primaryChan);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // --- aplica o modo LR selecionado ---
  setLongRange(LONG_RANGE_LEVEL);
}

void loop() {
  // Nada aqui
}