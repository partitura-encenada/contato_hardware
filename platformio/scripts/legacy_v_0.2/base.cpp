#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "../util/util.h"

typedef struct { // Struct da mensagem, deve ser igual ao da base 
  int16_t roll;
  int16_t accel;
  int8_t touch;
} message_t;
message_t message;
uint32_t id;
// Função callback receber
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&message, incomingData, sizeof(message));
  memcpy(&id, mac_addr, sizeof(id));
  
  Serial.println(String(id)+'/'+
                String(message.roll)+'/'+
                String(message.accel)+'/'+
                String(message.touch));
}

void setup() {
  Serial.begin(115200); // Inicializa conexão Serial
  WiFi.mode(WIFI_STA); // Wi-fi Station
  Util::PrintMACAddr(); 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv); // Registra função callback "receber"
}
 
void loop() { 
}

