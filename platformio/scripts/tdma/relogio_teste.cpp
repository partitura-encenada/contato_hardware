// ═════════ BASE MESTRE TDMA ═════════
// Envia beacon de sincronização para todos os equips
// Canal 8, ciclo de 6 slots × 1400µs = 8.4ms

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"

const int      CANAL      = 1;
const int      NUM_EQUIPS = 6;
const uint32_t SLOT_US    = 1500; // ← altere aqui para testar diferentes slots

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
    uint8_t  slot_atual;
    uint32_t timestamp;
} beacon_t;
beacon_t beacon;

esp_now_peer_info_t peerInfo;

esp_err_t setChannel(int channel) {
    esp_wifi_set_promiscuous(true);
    esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    return result;
}

void setup() {
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_NONE);

    WiFi.mode(WIFI_STA);
    setChannel(CANAL);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Erro ao adicionar peer");
        return;
    }

    Serial.println("Base mestre TDMA iniciada!");
    Serial.print("Canal: ");      Serial.println(CANAL);
    Serial.print("Equips: ");     Serial.println(NUM_EQUIPS);
    Serial.print("Slot: ");       Serial.print(SLOT_US);    Serial.println(" µs");
    Serial.print("Ciclo total: "); Serial.print(SLOT_US * NUM_EQUIPS); Serial.println(" µs");
}

void loop() {
    for (int slot = 0; slot < NUM_EQUIPS; slot++) {
        beacon.slot_atual = slot;
        beacon.timestamp  = micros();
        esp_now_send(broadcastAddress, (uint8_t *)&beacon, sizeof(beacon));
        delayMicroseconds(SLOT_US);
    }
}