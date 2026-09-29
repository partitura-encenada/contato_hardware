// Documentação: README.md, seção "Equip e Base 6DOF".

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "ota_receptor.h"

// ALTERAR conforme o equip físico (valores atuais: base 6)
const int CHANNEL = 11;
uint8_t equipAddress[] = {0x84, 0x1F, 0xE8, 0x1B, 0xBD, 0x40};  // MAC do equip_6DOF
const uint8_t BASE_ID = 6;

// Mesma struct do equip_6DOF (15 bytes)
typedef struct __attribute__((packed)) {
    uint8_t id;
    uint8_t mode;       // 'B' = brutos, 'T' = tratados
    int16_t rot[3];
    int16_t acc[3];
    uint8_t touch;
} message_6dof_t;

typedef struct {
    uint8_t active;
} control_t;

static message_6dof_t receivedMessage;
static message_6dof_t bufferMessage;
volatile bool newData = false;
bool serialActive = false;
uint32_t lastResend = 0;

esp_now_peer_info_t peerEquip;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void sendControl(uint8_t active) {
    control_t ctrl;
    ctrl.active = active;
    esp_now_send(equipAddress, (uint8_t *)&ctrl, sizeof(ctrl));
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (otaProcessarPacote(mac_addr, incomingData, len)) return;

    if (memcmp(mac_addr, equipAddress, 6) != 0) return;
    if (len != sizeof(message_6dof_t)) return;

    portENTER_CRITICAL_ISR(&mux);
    memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));
    newData = true;
    portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(1);

    esp_log_level_set("*", ESP_LOG_NONE);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    memset(&peerEquip, 0, sizeof(peerEquip));
    memcpy(peerEquip.peer_addr, equipAddress, 6);
    peerEquip.channel = 0;
    peerEquip.encrypt = false;
    esp_now_add_peer(&peerEquip);
}

void loop() {
    otaProcessarPendencias();

    if (Serial.available() > 0) {
        char cmd[16] = {0};

        Serial.readBytesUntil('\n', cmd, sizeof(cmd) - 1);

        if (strcmp(cmd, "START") == 0) {
            serialActive = true;
            sendControl(1);
            lastResend = millis();
        }
        else if (strcmp(cmd, "STOP") == 0) {
            serialActive = false;
            sendControl(0);
        }
        else if (strcmp(cmd, "ID?") == 0) {
            Serial.print("ID/");
            Serial.println(BASE_ID);
        }
    }

    // reenvia o controle caso o equip tenha reiniciado
    if (serialActive && (millis() - lastResend >= 2000)) {
        lastResend = millis();
        sendControl(1);
    }

    if (newData) {
        portENTER_CRITICAL(&mux);
        memcpy(&bufferMessage, &receivedMessage, sizeof(receivedMessage));
        newData = false;
        portEXIT_CRITICAL(&mux);

        char buf[96];
        snprintf(buf, sizeof(buf), "%d/%c/%d/%d/%d/%d/%d/%d/%d",
                 bufferMessage.id,
                 bufferMessage.mode,
                 bufferMessage.rot[0], bufferMessage.rot[1], bufferMessage.rot[2],
                 bufferMessage.acc[0], bufferMessage.acc[1], bufferMessage.acc[2],
                 bufferMessage.touch);

        // só escreve se couber no buffer, para não travar o loop
        int len = strlen(buf) + 2;
        if (serialActive && Serial.availableForWrite() >= len) {
            Serial.println(buf);
        }
    }
}
