// ═════════ ESP32 PONTE — Serial -> ESP-NOW (OTA) ═════════
// Recebe um binario de firmware pela Serial (protocolo definido em
// ota.py, no contato_cli) e retransmite por ESP-NOW, fragmentado, para
// o MAC do equip/base alvo. Nao faz Wi-Fi de internet - so ESP-NOW, no
// mesmo canal que o resto do sistema contato usa.
//
// Protocolo (ver tambem o cabecalho de ota.py):
//   comp -> ponte : "OTA_MAC <mac hex sem dois-pontos>\n"
//   ponte -> comp : "OK_MAC\n"
//   comp -> ponte : "OTA_SIZE <bytes>\n"
//   ponte -> comp : "OK_SIZE\n"
//   comp -> ponte : <ate 200 bytes binarios> (repetido)
//   ponte -> comp : "OK_CHUNK\n"              (apos cada pedaco)
//   comp -> ponte : "OTA_END\n"
//   ponte -> comp : "RESULTADO: ...\n"

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"

// ═════════ ALTERAR POR CONJUNTO ═════════
const int CANAL = 1; // TEM que ser o mesmo canal usado por bases/equips/relogio

// ═════════ Struct do pacote OTA (fragmento por ESP-NOW) ═════════
#define OTA_TIPO_INICIO 0xAA
#define OTA_TIPO_DADO   0xBB
#define OTA_TIPO_FIM    0xCC
#define OTA_MAX_DADOS   200

typedef struct {
    uint8_t  tipo;
    uint32_t indice;
    uint32_t tamanho_total;
    uint16_t tamanho_dado;
    uint8_t  dados[OTA_MAX_DADOS];
} ota_pacote_t;

// Deve ser EXATAMENTE igual ao ota_status_t do ota_receptor.h - e a
// resposta que o alvo manda de volta apos gravar (ou falhar).
typedef struct {
    uint8_t  status; // 0 = sucesso, 1 = erro
    uint32_t bytes_recebidos;
    char     mensagem[64];
} ota_status_t;

// Deve ser EXATAMENTE igual ao calibracao_resultado_t do
// calibracao_espnow.cpp - e o resultado da calibracao MPU6050.
typedef struct {
    int16_t offset_accel_x;
    int16_t offset_accel_y;
    int16_t offset_accel_z;
    int16_t offset_gyro_x;
    int16_t offset_gyro_y;
    int16_t offset_gyro_z;
} calibracao_resultado_t;

volatile bool statusRecebido = false;
ota_status_t statusRecebidoDados;

enum EstadoPonte { AGUARDANDO_COMANDO, RECEBENDO_DADOS };
EstadoPonte estado = AGUARDANDO_COMANDO;

uint8_t macAlvo[6];
bool temMacAlvo = false;
esp_now_peer_info_t peerAlvo;

uint32_t tamanhoTotalEsperado = 0;
uint32_t bytesRecebidos = 0;
uint32_t indicePacote = 0;

volatile bool envioConfirmado = false;
volatile bool envioComSucesso = false;

// ═════════ Callback de confirmacao de entrega ESP-NOW ═════════
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    envioConfirmado = true;
    envioComSucesso = (status == ESP_NOW_SEND_SUCCESS);
}

// A ponte recebe por ESP-NOW: a resposta de status apos um OTA_END
// (ota_status_t), ou um resultado de calibracao MPU6050
// (calibracao_resultado_t) - diferencia pelo tamanho do pacote.
void OnStatusRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (len == sizeof(ota_status_t)) {
        memcpy(&statusRecebidoDados, incomingData, sizeof(statusRecebidoDados));
        statusRecebido = true;
        return;
    }

    if (len == sizeof(calibracao_resultado_t)) {
        calibracao_resultado_t resultado;
        memcpy(&resultado, incomingData, sizeof(resultado));

        Serial.print("CALIBRACAO: ax=");
        Serial.print(resultado.offset_accel_x);
        Serial.print(" ay=");
        Serial.print(resultado.offset_accel_y);
        Serial.print(" az=");
        Serial.print(resultado.offset_accel_z);
        Serial.print(" gx=");
        Serial.print(resultado.offset_gyro_x);
        Serial.print(" gy=");
        Serial.print(resultado.offset_gyro_y);
        Serial.print(" gz=");
        Serial.println(resultado.offset_gyro_z);
        return;
    }
}

bool enviarPacoteOta(ota_pacote_t &pacote) {
    envioConfirmado = false;
    envioComSucesso = false;

    esp_now_send(macAlvo, (uint8_t *)&pacote, sizeof(pacote));

    uint32_t inicio = millis();
    while (!envioConfirmado && millis() - inicio < 1000) {
        delay(1);
    }

    return envioConfirmado && envioComSucesso;
}

bool enviarPacoteOtaComRetentativa(ota_pacote_t &pacote, int tentativas = 3) {
    for (int i = 0; i < tentativas; i++) {
        if (enviarPacoteOta(pacote)) return true;
        delay(20);
    }
    return false;
}

// ═════════ Parse do MAC em hex (12 chars, sem dois-pontos) ═════════
bool parseMacHex(const char *hex, uint8_t *mac) {
    if (strlen(hex) != 12) return false;
    for (int i = 0; i < 6; i++) {
        char par[3] = { hex[i * 2], hex[i * 2 + 1], '\0' };
        mac[i] = (uint8_t)strtol(par, nullptr, 16);
    }
    return true;
}

void registrarPeerAlvo() {
    if (esp_now_is_peer_exist(macAlvo)) {
        esp_now_del_peer(macAlvo);
    }

    memset(&peerAlvo, 0, sizeof(peerAlvo));
    memcpy(peerAlvo.peer_addr, macAlvo, 6);
    peerAlvo.channel = 0;
    peerAlvo.encrypt = false;
    esp_now_add_peer(&peerAlvo);
}

// ═════════ Processa uma linha de comando recebida pela serial ═════════
void processarLinha(String linha) {
    linha.trim();
    if (linha.length() == 0) return;

    if (linha.startsWith("OTA_MAC ")) {
        String hex = linha.substring(8);
        hex.trim();

        if (!parseMacHex(hex.c_str(), macAlvo)) {
            Serial.println("ERRO_MAC");
            return;
        }

        temMacAlvo = true;
        registrarPeerAlvo();
        Serial.println("OK_MAC");
        return;
    }

    if (linha.startsWith("OTA_SIZE ")) {
        if (!temMacAlvo) {
            Serial.println("ERRO_SEM_MAC");
            return;
        }

        tamanhoTotalEsperado = (uint32_t) linha.substring(9).toInt();
        bytesRecebidos = 0;
        indicePacote = 0;

        ota_pacote_t pacote;
        pacote.tipo = OTA_TIPO_INICIO;
        pacote.indice = 0;
        pacote.tamanho_total = tamanhoTotalEsperado;
        pacote.tamanho_dado = 0;

        if (!enviarPacoteOtaComRetentativa(pacote)) {
            Serial.println("ERRO_INICIO_NAO_CONFIRMADO");
            return;
        }

        Serial.println("OK_SIZE");
        estado = RECEBENDO_DADOS;
        return;
    }

    if (linha == "OTA_END") {
        ota_pacote_t pacote;
        pacote.tipo = OTA_TIPO_FIM;
        pacote.indice = indicePacote;
        pacote.tamanho_total = tamanhoTotalEsperado;
        pacote.tamanho_dado = 0;

        bool ok = enviarPacoteOtaComRetentativa(pacote);

        if (bytesRecebidos != tamanhoTotalEsperado) {
            Serial.print("RESULTADO: FALHA - recebi ");
            Serial.print(bytesRecebidos);
            Serial.print(" de ");
            Serial.print(tamanhoTotalEsperado);
            Serial.println(" bytes esperados");
        } else if (!ok) {
            Serial.println("RESULTADO: FALHA - pacote de fim nao confirmado pelo radio");
        } else {
            // Espera a resposta de aplicacao do alvo (gravou/nao gravou).
            statusRecebido = false;
            uint32_t inicioEspera = millis();
            while (!statusRecebido && millis() - inicioEspera < 8000) {
                delay(10);
            }

            if (statusRecebido) {
                if (statusRecebidoDados.status == 0) {
                    Serial.print("RESULTADO: SUCESSO - ");
                } else {
                    Serial.print("RESULTADO: FALHA - ");
                }
                Serial.println(statusRecebidoDados.mensagem);
            } else {
                Serial.println("RESULTADO: SEM CONFIRMACAO DO ALVO (radio entregou, mas ninguem respondeu em 8s - o alvo pode nao ter o modulo ota_receptor.h instalado ainda)");
            }
        }

        temMacAlvo = false;
        estado = AGUARDANDO_COMANDO;
        return;
    }
}

// ═════════ setup ═════════
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(3000);
    esp_log_level_set("*", ESP_LOG_NONE);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    if (esp_now_init() != ESP_OK) {
        Serial.println("RESULTADO: FALHA init ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnStatusRecv);

    Serial.println("Ponte pronta.");
}

// ═════════ loop ═════════
void loop() {
    if (estado == AGUARDANDO_COMANDO) {
        if (Serial.available() > 0) {
            String linha = Serial.readStringUntil('\n');
            processarLinha(linha);
        }
        return;
    }

    // estado == RECEBENDO_DADOS
    uint32_t restante = tamanhoTotalEsperado - bytesRecebidos;
    uint16_t tamanhoChunk = (restante < OTA_MAX_DADOS) ? restante : OTA_MAX_DADOS;

    if (Serial.available() >= tamanhoChunk) {
        ota_pacote_t pacote;
        pacote.tipo = OTA_TIPO_DADO;
        pacote.indice = indicePacote;
        pacote.tamanho_total = tamanhoTotalEsperado;
        pacote.tamanho_dado = tamanhoChunk;

        Serial.readBytes(pacote.dados, tamanhoChunk);

        if (enviarPacoteOtaComRetentativa(pacote)) {
            bytesRecebidos += tamanhoChunk;
            indicePacote++;
            Serial.println("OK_CHUNK");

            if (bytesRecebidos >= tamanhoTotalEsperado) {
                estado = AGUARDANDO_COMANDO; // proximo esperado: OTA_END
            }
        } else {
            Serial.println("ERRO_CHUNK_NAO_CONFIRMADO");
        }
    }
}
