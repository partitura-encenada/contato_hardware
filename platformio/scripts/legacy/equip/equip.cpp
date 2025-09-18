#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"


#define FREQ_PRINT 20           // Comente para imprimir tudo
#define PROTOCOLO_WIFI WIFI_PROTOCOL_LR  // LR, 11B, 11G, 11N


typedef struct struct_message {
    int id; 
    int gyro;
    int accel;
    int touch;
} struct_message;

struct_message MIDImessage;

#ifdef FREQ_PRINT
const unsigned long intervaloPrint = 1000 / FREQ_PRINT;
unsigned long ultimoPrint = 0;
#endif

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
    uint8_t macTransmissor[] = {0xcc, 0xdb, 0xa7, 0xa0, 0x08, 0x84};

    if (memcmp(mac_addr, macTransmissor, 6) != 0) return;

    memcpy(&MIDImessage, incomingData, sizeof(MIDImessage));

    #ifdef FREQ_PRINT
    unsigned long tempoAtual = millis();
    if (tempoAtual - ultimoPrint >= intervaloPrint) {
        ultimoPrint = tempoAtual;
        Serial.println(String(MIDImessage.id) + "/" +
                       String(MIDImessage.gyro) + "/" +
                       String(MIDImessage.accel) + "/" +
                       String(MIDImessage.touch));
    }
    #else
    Serial.println(String(MIDImessage.id) + "/" +
                   String(MIDImessage.gyro) + "/" +
                   String(MIDImessage.accel) + "/" +
                   String(MIDImessage.touch));
    #endif
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(82);

    // Ativa protocolo definido
    esp_wifi_set_protocol(WIFI_IF_STA, PROTOCOLO_WIFI);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(5, WIFI_SECOND_CHAN_NONE);
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
}

void loop() {
    // Nada aqui
}
