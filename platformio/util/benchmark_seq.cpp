// ═════════ BENCHMARK SEQ — base ═════════
// Conta pacotes recebidos vs esperados e mede taxa de perda real
// Usar com benchmark_beacon e relogio ligados
// Canal 8, imprime estatísticas a cada 10 segundos

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"

const int CANAL = 8;

// MAC do equip_3
uint8_t macTransmissor[] = {0x68, 0x25, 0xDD, 0x32, 0x88, 0xB4};

// ═════════ Struct mensagem ═════════
typedef struct {
    uint8_t  id;
    int16_t  gyro;
    int32_t  accel;
    uint8_t  touch;
    uint32_t seq;
} message_t;

// ═════════ Variáveis de contagem ═════════
volatile uint32_t ultimo_seq        = 0;
volatile uint32_t pacotes_recebidos = 0;
volatile uint32_t pacotes_perdidos  = 0;
volatile uint32_t gap_maximo        = 0;
volatile bool     primeiro_pacote   = true;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (memcmp(mac_addr, macTransmissor, 6) != 0) return;
    if (len != sizeof(message_t)) return;

    message_t msg;
    memcpy(&msg, incomingData, sizeof(message_t));

    portENTER_CRITICAL_ISR(&mux);

    if (primeiro_pacote) {
        // primeiro pacote — inicializa sem contar perda
        ultimo_seq      = msg.seq;
        primeiro_pacote = false;
        pacotes_recebidos++;

    } else if (msg.seq == 0 || msg.seq < ultimo_seq) {
        // equip reiniciou — reseta contadores para não distorcer
        pacotes_recebidos = 1;
        pacotes_perdidos  = 0;
        gap_maximo        = 0;
        ultimo_seq        = msg.seq;

    } else {
        uint32_t gap = msg.seq - ultimo_seq - 1;
        if (gap > 0) {
            pacotes_perdidos += gap;
            if (gap > gap_maximo) gap_maximo = gap;
        }
        ultimo_seq = msg.seq;
        pacotes_recebidos++;
    }

    portEXIT_CRITICAL_ISR(&mux);
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
    Serial.println("benchmark_seq pronto — aguardando pacotes...");
    Serial.println("Estatísticas impressas a cada 10 segundos");
    Serial.println("══════════════════════════════════════════");
}

void loop() {
    delay(10000);

    portENTER_CRITICAL(&mux);
    uint32_t recebidos = pacotes_recebidos;
    uint32_t perdidos  = pacotes_perdidos;
    uint32_t gap_max   = gap_maximo;
    uint32_t total     = recebidos + perdidos;
    portEXIT_CRITICAL(&mux);

    float taxa_perda = total > 0 ? (float)perdidos / total * 100.0 : 0.0;

    Serial.println("══════════ ESTATÍSTICAS ══════════");
    Serial.print("Recebidos  : "); Serial.println(recebidos);
    Serial.print("Perdidos   : "); Serial.println(perdidos);
    Serial.print("Total esp. : "); Serial.println(total);
    Serial.print("Taxa perda : "); Serial.print(taxa_perda, 2); Serial.println(" %");
    Serial.print("Maior gap  : "); Serial.print(gap_max); Serial.println(" pacotes consecutivos");
    Serial.println("══════════════════════════════════");
}