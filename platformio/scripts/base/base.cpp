// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h" 


const int CANAL_ESPECIFICO = 2;     
uint8_t macTransmissor[] = {0x68, 0x25, 0xDD, 0x32, 0x88, 0xB4};

// Struct compacta — deve ser idêntica à do equip
typedef struct {
    uint8_t  id;
    int16_t  gyro;
    int32_t  accel;
    uint8_t  touch;
} struct_message;

static struct_message MIDImessage;
static struct_message bufferMessage;
volatile bool newData = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // mutex contra race condition

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (memcmp(mac_addr, macTransmissor, 6) != 0) return;
    if (len != sizeof(struct_message)) return; // descarta pacote com tamanho errado

    portENTER_CRITICAL_ISR(&mux);
    memcpy(&MIDImessage, incomingData, sizeof(MIDImessage));
    newData = true;
    portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_NONE);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CANAL_ESPECIFICO, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    // Preâmbulo longo: deve ser igual ao do equip
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    if (newData) {
        portENTER_CRITICAL(&mux);
        memcpy(&bufferMessage, &MIDImessage, sizeof(MIDImessage));
        newData = false;
        portEXIT_CRITICAL(&mux);

        char buf[64];
        snprintf(buf, sizeof(buf), "%d/%d/%d/%d",
                 bufferMessage.id,
                 bufferMessage.gyro,
                 bufferMessage.accel,
                 bufferMessage.touch);
        Serial.println(buf);
    }
}