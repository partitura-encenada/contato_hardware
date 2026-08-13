//═════════ Bibliotecas ═════════
#include <esp_now.h>                    
#include <WiFi.h>                       
#include "esp_wifi.h"    
#include <esp_now.h>                        
#include <Preferences.h>

//═════════ ALTERAR POR CONJUNTO ═════════   
const int CANAL_ESPECIFICO = 1;     
uint8_t macTransmissores[][6] = {
    {0x3C, 0x8A, 0x1F, 0x80, 0x76, 0xA4}, // opção 1
    {0xF8, 0xB3, 0xB7, 0x50, 0xCC, 0xEC}  // opção 2
};

const int NUM_MACS = sizeof(macTransmissores) / sizeof(macTransmissores[0]);

uint8_t macAtivo[6];
bool temMacAtivo = false;
const uint8_t BASE_ID = 7;

//═════════ Struct da mensagem ESP-NOW ═════════
typedef struct {
    uint8_t  id;
    int16_t  gyro;
    int32_t  accel;
    uint8_t  touch;
} struct_message;

static struct_message MIDImessage;
static struct_message bufferMessage;
volatile bool newData = false;
bool serialAtivo = false; // só imprime depois que contato_cli mandar START
Preferences prefs;
bool modoDiscover = false;
uint32_t ultimoReenvio = 0;

typedef struct {
    uint8_t ativo;
} controle_t;

esp_now_peer_info_t peerEquip;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // mutex contra race condition

bool macEstaNaLista(const uint8_t *mac) {
    for (int i = 0; i < NUM_MACS; i++) {
        if (memcmp(mac, macTransmissores[i], 6) == 0) {
            return true;
        }
    }
    return false;
}

void enviarControleParaMac(const uint8_t *mac, uint8_t ativo) {
    controle_t ctrl;
    ctrl.ativo = ativo;
    esp_now_send(mac, (uint8_t *)&ctrl, sizeof(ctrl));
}

void enviarControle(uint8_t ativo) {
    if (temMacAtivo) {
        enviarControleParaMac(macAtivo, ativo);
    }
}

void enviarControleParaTodos(uint8_t ativo) {
    for (int i = 0; i < NUM_MACS; i++) {
        enviarControleParaMac(macTransmissores[i], ativo);
    }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (!macEstaNaLista(mac_addr)) return;
    if (len != sizeof(struct_message)) return; // descarta pacote com tamanho errado

    if (!temMacAtivo) {
        memcpy(macAtivo, mac_addr, 6);
        temMacAtivo = true;

        if (modoDiscover) {
            prefs.putBytes("mac", macAtivo, 6);
            Serial.println("MAC_SAVED");
            modoDiscover = false;
        }
    }

    portENTER_CRITICAL_ISR(&mux);
    memcpy(&MIDImessage, incomingData, sizeof(MIDImessage));
    newData = true;
    portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
    Serial.begin(115200);
    prefs.begin("contato", false);

    if (prefs.getBytesLength("mac") == 6) {
        prefs.getBytes("mac", macAtivo, 6);
        temMacAtivo = true;
    }
    Serial.setTimeout(1);

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

    for (int i = 0; i < NUM_MACS; i++) {
        memset(&peerEquip, 0, sizeof(peerEquip));
        memcpy(peerEquip.peer_addr, macTransmissores[i], 6);
        peerEquip.channel = 0;
        peerEquip.encrypt = false;
        esp_now_add_peer(&peerEquip);
    }
}

void loop() {
// Comando não bloqueante vindo do contato_cli: START / STOP
    if (Serial.available() > 0) {
        char cmd[16] = {0};

        Serial.readBytesUntil('\n', cmd, sizeof(cmd) - 1);

        if (strcmp(cmd, "START") == 0) {
            serialAtivo = true;

            if (temMacAtivo) {
                enviarControle(1);
            }

            ultimoReenvio = millis();
        } 
        else if (strcmp(cmd, "STOP") == 0) {
            serialAtivo = false;
            if (temMacAtivo) {
                enviarControle(0);
            } else {
                enviarControleParaTodos(0);
            }
        }
        else if (strcmp(cmd, "ID?") == 0) {
            Serial.print("ID/");
            Serial.println(BASE_ID);
        }
        else if (strcmp(cmd, "DISCOVER") == 0) {
            temMacAtivo = false;
            modoDiscover = true;
            enviarControleParaTodos(1);
        }
    }
    if (serialAtivo && temMacAtivo && (millis() - ultimoReenvio >= 2000)) {
        ultimoReenvio = millis();
        enviarControle(1);
    }
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

        int len = strlen(buf) + 2;
        if (serialAtivo && Serial.availableForWrite() >= len) {
            Serial.println(buf);
        }
    }
}