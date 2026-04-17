// ═════════ BASE MESTRE TDMA ═════════
// Envia beacon de sincronização para todos os equips
// Cada equip recebe o beacon e calcula seu slot de transmissão
// Canal 8, ciclo de 12ms (6 slots × 2ms cada)

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"

const int CANAL = 8;
const int NUM_EQUIPS = 6;
const int SLOT_MS = 1;                        // 1ms por slot
const int CICLO_MS = NUM_EQUIPS * SLOT_MS;    // 6ms por ciclo completo

// Endereço de broadcast para todos os equips
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Estrutura do beacon
typedef struct {
    uint8_t slot_atual;   // slot que está sendo aberto (0 a 5)
    uint32_t timestamp;   // timestamp do ciclo
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

    // Adiciona peer broadcast
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Erro ao adicionar peer");
        return;
    }

    Serial.println("Base mestre TDMA iniciada!");
    Serial.print("Canal: "); Serial.println(CANAL);
    Serial.print("Ciclo: "); Serial.print(CICLO_MS); Serial.println("ms");
    Serial.print("Slot: "); Serial.print(SLOT_MS); Serial.println("ms");
}

void loop() {
    for (int slot = 0; slot < NUM_EQUIPS; slot++) {
        beacon.slot_atual = slot;
        beacon.timestamp = millis();
        esp_now_send(broadcastAddress, (uint8_t *)&beacon, sizeof(beacon));
        delay(SLOT_MS);
    }
}
