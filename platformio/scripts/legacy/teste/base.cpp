#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// Estrutura de dados recebida (deve bater com o transmissor)
typedef struct struct_message {
    int id;       // ID do dispositivo
    int roll;     // Roll em graus
    float accel;  // Aceleração total em m/s²
    int touch;    // Estado do touch
    float gyroX;  // Giroscópio X em rad/s
    float gyroY;  // Giroscópio Y em rad/s
    float gyroZ;  // Giroscópio Z em rad/s
} struct_message;

struct_message MIDImessage;

// Controle de tempo para evitar prints muito rápidos
unsigned long ultimoPrint = 0;
const unsigned long intervaloPrint = 20; // 20ms entre prints

// Callback de recepção
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  // MAC do transmissor esperado
  uint8_t macTransmissor[] = {0xcc, 0xdb, 0xa7, 0xa0, 0x08, 0x84}; 

  if (memcmp(mac_addr, macTransmissor, 6) != 0) return;

  unsigned long tempoAtual = millis();
  if (tempoAtual - ultimoPrint >= intervaloPrint) {
    memcpy(&MIDImessage, incomingData, sizeof(MIDImessage));

    Serial.print("ID: "); Serial.print(MIDImessage.id);
    Serial.print(" | Roll: "); Serial.print(MIDImessage.roll);
    Serial.print(" | Accel: "); Serial.print(MIDImessage.accel);
    Serial.print(" | Touch: "); Serial.print(MIDImessage.touch);
    Serial.print(" | Gyro X: "); Serial.print(MIDImessage.gyroX);
    Serial.print(" | Gyro Y: "); Serial.print(MIDImessage.gyroY);
    Serial.print(" | Gyro Z: "); Serial.println(MIDImessage.gyroZ);

    ultimoPrint = tempoAtual;
  }
}

void setup() {
  Serial.begin(115200);

  // Configura Wi-Fi como estação
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(82);

  // Fixa o canal no mesmo do transmissor
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(5, WIFI_SECOND_CHAN_NONE); 
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
