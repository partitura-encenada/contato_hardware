// ═════════ BENCHMARK ACK — base ═════════
// Recebe pacotes do benchmark_slot (equip_3) e devolve ACK
// Usar no lugar da base_3_tdma durante o benchmark
// Canal 8, macTransmissor = MAC do equip_3

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"

const int CANAL = 8;

// MAC do equip_3 — só aceita pacotes dele
uint8_t macTransmissor[] = {0x68, 0x25, 0xDD, 0x32, 0x88, 0xB4};

// ─── struct do pacote equip → base ───
typedef struct {
    uint8_t  id;
    int16_t  gyro;
    int32_t  accel;
    uint8_t  touch;
    uint16_t seq;
} message_t;

// ─── struct do ACK base → equip ───
typedef struct {
    uint16_t seq;
} ack_t;

esp_now_peer_info_t peerInfo;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (memcmp(mac_addr, macTransmissor, 6) != 0) return;
    if (len != sizeof(message_t)) return;

    message_t msg;
    memcpy(&msg, incomingData, sizeof(message_t));

    // devolve ACK com o mesmo número de sequência
    ack_t ack;
    ack.seq = msg.seq;
    esp_now_send(macTransmissor, (uint8_t *)&ack, sizeof(ack_t));
}

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

    esp_now_register_recv_cb(OnDataRecv);

    // registra equip_3 como peer para enviar ACK
    peerInfo = {};
    memcpy(peerInfo.peer_addr, macTransmissor, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Erro ao adicionar peer");
        return;
    }

    Serial.println("benchmark_ack pronto — aguardando pacotes...");
}

void loop() {}
