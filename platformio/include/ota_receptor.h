#pragma once
#include <esp_now.h>
#include <Update.h>
#include <string.h>

#define OTA_TIPO_INICIO 0xAA
#define OTA_TIPO_DADO    0xBB
#define OTA_TIPO_FIM     0xCC
#define OTA_MAX_DADOS    230

typedef struct {
    uint8_t  tipo;
    uint32_t indice;
    uint32_t tamanho_total;
    uint16_t tamanho_dado;
    uint8_t  dados[OTA_MAX_DADOS];
} ota_pacote_t;

typedef struct {
    uint8_t  status;
    uint32_t bytes_recebidos;
    char     mensagem[64];
} ota_status_t;

static bool     otaEmAndamento     = false;
static bool     otaTeveErro        = false; 
static uint32_t otaBytesRecebidos  = 0;
static uint32_t otaTamanhoTotal    = 0;
static uint32_t otaIndiceEsperado  = 0;
static uint8_t  otaMacPonte[6];

#define OTA_FILA_TAMANHO 4
static ota_pacote_t   otaFila[OTA_FILA_TAMANHO];
static uint8_t         otaFilaMac[OTA_FILA_TAMANHO][6];
static volatile int    otaFilaEntrada = 0;
static volatile int    otaFilaSaida   = 0;
static volatile int    otaFilaCount   = 0;
static volatile uint32_t otaFilaDescartados = 0; 
static portMUX_TYPE    otaMux = portMUX_INITIALIZER_UNLOCKED;

inline void otaEnviarStatus(uint8_t status, const char *mensagem) {

    if (status != 0) {
        if (otaTeveErro) return;
        otaTeveErro = true;
    }

    ota_status_t resposta;
    resposta.status = status;
    resposta.bytes_recebidos = otaBytesRecebidos;
    strncpy(resposta.mensagem, mensagem, sizeof(resposta.mensagem) - 1);
    resposta.mensagem[sizeof(resposta.mensagem) - 1] = '\0';

    if (!esp_now_is_peer_exist(otaMacPonte)) {
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, otaMacPonte, 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    for (int i = 0; i < 5; i++) {
        esp_now_send(otaMacPonte, (uint8_t *)&resposta, sizeof(resposta));
        delay(50);
    }
}


inline bool otaProcessarPacote(const uint8_t *mac_addr, const uint8_t *dadosRecebidos, int len) {
    if (len != sizeof(ota_pacote_t)) return false;

    portENTER_CRITICAL_ISR(&otaMux);
    if (otaFilaCount < OTA_FILA_TAMANHO) {
        memcpy(&otaFila[otaFilaEntrada], dadosRecebidos, sizeof(ota_pacote_t));
        memcpy(otaFilaMac[otaFilaEntrada], mac_addr, 6);
        otaFilaEntrada = (otaFilaEntrada + 1) % OTA_FILA_TAMANHO;
        otaFilaCount++;
    } else {
        otaFilaDescartados++;
    }
    portEXIT_CRITICAL_ISR(&otaMux);

    return true;
}

inline void otaProcessarUm(const ota_pacote_t &pacote) {
    switch (pacote.tipo) {
        case OTA_TIPO_INICIO: {
            otaTamanhoTotal = pacote.tamanho_total;
            otaBytesRecebidos = 0;
            otaIndiceEsperado = 0;
            otaTeveErro = false;
            otaFilaDescartados = 0; 
            Serial.print("[OTA] INICIO recebido, tamanho total: ");
            Serial.println(otaTamanhoTotal);

            Update.abort();

            if (!Update.begin(otaTamanhoTotal)) {
                otaEmAndamento = false;
                char msg[64];
                snprintf(msg, sizeof(msg), "Update.begin falhou: %s", Update.errorString());
                Serial.println(msg);
                otaEnviarStatus(1, msg);
                return;
            }

            Serial.println("[OTA] Update.begin OK");
            otaEmAndamento = true;
            return;
        }

        case OTA_TIPO_DADO: {
            if (!otaEmAndamento) {
                char msg[64];
                snprintf(msg, sizeof(msg), "sem INICIO idx=%u desc=%u",
                         (unsigned)pacote.indice, (unsigned)otaFilaDescartados);
                otaEnviarStatus(1, msg);
                return;
            }

            if (pacote.indice < otaIndiceEsperado) {
                return;
            }

            if (pacote.indice > otaIndiceEsperado) {
                otaEmAndamento = false;
                Update.abort();
                char msg[64];
                snprintf(msg, sizeof(msg), "fora ordem esp=%u veio=%u desc=%u",
                         (unsigned)otaIndiceEsperado, (unsigned)pacote.indice, (unsigned)otaFilaDescartados);
                otaEnviarStatus(1, msg);
                return;
            }

            if (Update.write((uint8_t *)pacote.dados, pacote.tamanho_dado) != pacote.tamanho_dado) {
                otaEmAndamento = false;
                Update.abort();
                otaEnviarStatus(1, "falha ao gravar na flash");
                return;
            }

            otaBytesRecebidos += pacote.tamanho_dado;
            otaIndiceEsperado++;
            return;
        }

        case OTA_TIPO_FIM: {
            if (!otaEmAndamento) {
                otaEnviarStatus(1, "FIM recebido sem sessao ativa");
                return;
            }

            if (pacote.indice != otaIndiceEsperado) {
                otaEmAndamento = false;
                Update.abort();
                char msg[64];
                snprintf(msg, sizeof(msg), "contagem nao bate: ponte=%u recebi=%u",
                         (unsigned)pacote.indice, (unsigned)otaIndiceEsperado);
                otaEnviarStatus(1, msg);
                return;
            }

            if (otaBytesRecebidos != otaTamanhoTotal) {
                otaEmAndamento = false;
                Update.abort();
                char msg[64];
                snprintf(msg, sizeof(msg), "tamanho incompleto: %u de %u",
                         (unsigned)otaBytesRecebidos, (unsigned)otaTamanhoTotal);
                otaEnviarStatus(1, msg);
                return;
            }

            if (!Update.end(true)) {
                otaEmAndamento = false;
                char msg[64];
                snprintf(msg, sizeof(msg), "Update.end falhou: %s", Update.errorString());
                otaEnviarStatus(1, msg);
                return;
            }

            otaEnviarStatus(0, "gravado com sucesso, reiniciando");
            delay(200);
            ESP.restart();
        }
    }
}


inline void otaProcessarPendencias() {
    while (true) {
        bool temPacote = false;
        ota_pacote_t pacote;
        uint8_t mac[6];

        portENTER_CRITICAL(&otaMux);
        if (otaFilaCount > 0) {
            pacote = otaFila[otaFilaSaida];
            memcpy(mac, otaFilaMac[otaFilaSaida], 6);
            otaFilaSaida = (otaFilaSaida + 1) % OTA_FILA_TAMANHO;
            otaFilaCount--;
            temPacote = true;
        }
        portEXIT_CRITICAL(&otaMux);

        if (!temPacote) break;

        memcpy(otaMacPonte, mac, 6);
        otaProcessarUm(pacote);
    }
}