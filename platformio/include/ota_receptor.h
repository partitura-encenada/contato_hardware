// ═════════ ota_receptor.h ═════════
// Modulo reutilizavel de recepcao de firmware por ESP-NOW (OTA).
//
// COMO USAR em qualquer equip_X.cpp / base_X.cpp:
//   1. #include "ota_receptor.h"   (no topo, com os outros includes)
//   2. Na SUA funcao OnDataRecv existente, como PRIMEIRA linha do corpo:
//          if (otaProcessarPacote(mac_addr, incomingData, len)) return;
//
// Isso e seguro de adicionar em qualquer arquivo existente: o tamanho
// de ota_pacote_t e bem maior que beacon_t/controle_t/struct_message,
// entao otaProcessarPacote() descarta na hora (retorna false) qualquer
// pacote que nao seja realmente OTA, sem interferir na logica normal.

#pragma once
#include <esp_now.h>
#include <Update.h>
#include <string.h>

#define OTA_TIPO_INICIO 0xAA
#define OTA_TIPO_DADO    0xBB
#define OTA_TIPO_FIM     0xCC
#define OTA_MAX_DADOS    200

// Deve ser EXATAMENTE igual ao ota_pacote_t da ponte.cpp - e o formato
// do pacote que ela envia.
typedef struct {
    uint8_t  tipo;
    uint32_t indice;
    uint32_t tamanho_total;
    uint16_t tamanho_dado;
    uint8_t  dados[OTA_MAX_DADOS];
} ota_pacote_t;

// Deve ser EXATAMENTE igual ao ota_status_t da ponte.cpp - e o formato
// da resposta que este modulo manda de volta.
typedef struct {
    uint8_t  status; // 0 = sucesso, 1 = erro
    uint32_t bytes_recebidos;
    char     mensagem[64];
} ota_status_t;

static bool     otaEmAndamento     = false;
static uint32_t otaBytesRecebidos  = 0;
static uint32_t otaTamanhoTotal    = 0;
static uint32_t otaIndiceEsperado  = 0; // proximo indice de DADO valido
static uint8_t  otaMacPonte[6];

inline void otaEnviarStatus(uint8_t status, const char *mensagem) {
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

    esp_now_send(otaMacPonte, (uint8_t *)&resposta, sizeof(resposta));
}

// Retorna true se o pacote recebido era um pacote OTA (e ja foi
// tratado); false se nao era (o chamador deve seguir com sua logica
// normal de OnDataRecv nesse caso).
inline bool otaProcessarPacote(const uint8_t *mac_addr, const uint8_t *dadosRecebidos, int len) {
    if (len != sizeof(ota_pacote_t)) return false;

    ota_pacote_t pacote;
    memcpy(&pacote, dadosRecebidos, sizeof(pacote));
    memcpy(otaMacPonte, mac_addr, 6);

    switch (pacote.tipo) {
        case OTA_TIPO_INICIO: {
            otaTamanhoTotal = pacote.tamanho_total;
            otaBytesRecebidos = 0;
            otaIndiceEsperado = 0;

            if (!Update.begin(otaTamanhoTotal)) {
                otaEmAndamento = false;
                otaEnviarStatus(1, "Update.begin falhou");
                return true;
            }

            otaEmAndamento = true;
            return true;
        }

        case OTA_TIPO_DADO: {
            if (!otaEmAndamento) {
                otaEnviarStatus(1, "dado recebido sem INICIO");
                return true;
            }

            if (pacote.indice < otaIndiceEsperado) {
                // Retransmissao de um pacote que ja foi gravado (a ponte
                // reenviou porque nao recebeu a confirmacao de radio a
                // tempo, mas o pacote original chegou). Ignora sem
                // gravar de novo - gravar duas vezes corromperia a imagem.
                return true;
            }

            if (pacote.indice > otaIndiceEsperado) {
                // Faltou um pacote no meio: a imagem ja esta incompleta
                // e nao da pra confiar no que vier depois. Aborta.
                otaEmAndamento = false;
                Update.abort();
                char msg[64];
                snprintf(msg, sizeof(msg), "pacote fora de ordem: esperava %u, veio %u",
                         (unsigned)otaIndiceEsperado, (unsigned)pacote.indice);
                otaEnviarStatus(1, msg);
                return true;
            }

            // pacote.indice == otaIndiceEsperado: caso normal.
            if (Update.write((uint8_t *)pacote.dados, pacote.tamanho_dado) != pacote.tamanho_dado) {
                otaEmAndamento = false;
                Update.abort();
                otaEnviarStatus(1, "falha ao gravar na flash");
                return true;
            }

            otaBytesRecebidos += pacote.tamanho_dado;
            otaIndiceEsperado++;
            return true;
        }

        case OTA_TIPO_FIM: {
            if (!otaEmAndamento) {
                otaEnviarStatus(1, "FIM recebido sem sessao ativa");
                return true;
            }

            if (pacote.indice != otaIndiceEsperado) {
                otaEmAndamento = false;
                Update.abort();
                char msg[64];
                snprintf(msg, sizeof(msg), "contagem de pacotes nao bate: ponte contou %u, recebi %u",
                         (unsigned)pacote.indice, (unsigned)otaIndiceEsperado);
                otaEnviarStatus(1, msg);
                return true;
            }

            if (otaBytesRecebidos != otaTamanhoTotal) {
                otaEmAndamento = false;
                Update.abort();
                char msg[64];
                snprintf(msg, sizeof(msg), "tamanho incompleto: %u de %u",
                         (unsigned)otaBytesRecebidos, (unsigned)otaTamanhoTotal);
                otaEnviarStatus(1, msg);
                return true;
            }

            if (!Update.end(true)) {
                otaEmAndamento = false;
                char msg[64];
                snprintf(msg, sizeof(msg), "Update.end falhou: %s", Update.errorString());
                otaEnviarStatus(1, msg);
                return true;
            }

            otaEnviarStatus(0, "gravado com sucesso, reiniciando");
            delay(200); // da tempo do pacote de status sair antes do reboot
            ESP.restart();
            return true; // nunca chega aqui de fato
        }

        default:
            return false;
    }
}
