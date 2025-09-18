#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// Estrutura de dados recebida (deve bater com o transmissor)
typedef struct struct_message {
    int id; 
    int gyro;
    int accel;
    int touch;
} struct_message;

struct_message MIDImessage;

// Callback de recepção
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  // MAC do transmissor esperado
  uint8_t macTransmissor[] = {0xcc, 0xdb, 0xa7, 0xa0, 0x08, 0x84}; //MAC usado no transmissor
// {0xf8, 0xb3, 0xb7, 0x50, 0xcc, 0xec} Contato003
// {0xcc, 0xdb, 0xa7, 0xa0, 0x08, 0x84} Contato004
// {0x3c, 0x8a, 0x1f, 0x80, 0x76, 0xa4} Contato005
// {0x94, 0x54, 0xc5, 0x6f, 0xa9, 0xa8} Contato006
// {0x3c, 0x8a, 0x1f, 0xa3, 0x52, 0x18} Contato007
// {0xf8, 0xb3, 0xb7, 0x2b, 0x09, 0x48} Contato008
// {0xcc, 0xdb, 0xa7, 0x91, 0x4d, 0x7c} Contato009

  if (memcmp(mac_addr, macTransmissor, 6) != 0) {
    return; // Ignora pacotes de outros dispositivos
  }
}
 
void setup() {
  Serial.begin(115200);

  // Configura Wi-Fi como estação
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(82);

  //Fixa o canal no mesmo do transmissor
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // Canal do transmissor
  esp_wifi_set_promiscuous(false);

  // Confirma canal em uso
  uint8_t primaryChan;
  wifi_second_chan_t secondChan;
  esp_wifi_get_channel(&primaryChan, &secondChan);
  Serial.print("Receptor no canal: ");
  Serial.println(primaryChan);

  // Inicia ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }

  // Registra callback de recepção
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nada aqui: apenas recebe via callback
}