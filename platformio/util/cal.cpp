// ═════════ Calibracao MPU6050 por ESP-NOW ═════════
// Roda CalibrateAccel/CalibrateGyro e manda o resultado pra ponte por
// ESP-NOW. Mantem o receptor OTA ativo depois de terminar, pra voce
// conseguir subir o firmware normal do equip em seguida sem precisar
// de USB.
//
// Fluxo: contato calibrar --id X --porta COM... compila e envia este
// arquivo via ponte (mesmo protocolo do `contato ota`). Depois que a
// calibracao termina e o resultado e enviado, o equip fica parado
// (loop() vazio) esperando o proximo OTA - que vai ser o firmware de
// verdade do equip, com os offsets ja preenchidos a mao.

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "ota_receptor.h"

// ═════════ ALTERAR POR CONJUNTO ═════════
const int CANAL = 1; // TEM que ser o mesmo canal do resto do sistema
uint8_t PONTE_MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // ALTERAR: MAC do ESP32 ponte

MPU6050 mpu;
const int callibration_time = 6;

typedef struct {
    int16_t offset_accel_x;
    int16_t offset_accel_y;
    int16_t offset_accel_z;
    int16_t offset_gyro_x;
    int16_t offset_gyro_y;
    int16_t offset_gyro_z;
} calibracao_resultado_t;

esp_now_peer_info_t peerPonte;

// ═════════ Callback de recepcao - so existe pra atender OTA ═════════
// Depois da calibracao, o equip so aceita novos pacotes OTA (pra
// receber o firmware normal em seguida). Qualquer outra coisa e
// ignorada por otaProcessarPacote (retorna false).
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    otaProcessarPacote(mac_addr, incomingData, len);
}

void enviarResultado(calibracao_resultado_t &resultado) {
    if (!esp_now_is_peer_exist(PONTE_MAC)) {
        memset(&peerPonte, 0, sizeof(peerPonte));
        memcpy(peerPonte.peer_addr, PONTE_MAC, 6);
        peerPonte.channel = 0;
        peerPonte.encrypt = false;
        esp_now_add_peer(&peerPonte);
    }

    // Sem confirmacao de entrega aqui (diferente do fluxo de OTA) -
    // manda varias vezes de proposito pra aumentar a chance de chegar.
    for (int i = 0; i < 5; i++) {
        esp_now_send(PONTE_MAC, (uint8_t *)&resultado, sizeof(resultado));
        delay(100);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    esp_log_level_set("*", ESP_LOG_NONE);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("ERRO MPU6050");
        return;
    }

    uint8_t dev_status = mpu.dmpInitialize();

    if (dev_status != 0) {
        Serial.print("ERRO DMP: ");
        Serial.println(dev_status);
        return;
    }

    Serial.println("Calibrando... nao mexa no sensor.");

    mpu.CalibrateAccel(callibration_time);
    mpu.CalibrateGyro(callibration_time);

    calibracao_resultado_t resultado;
    resultado.offset_accel_x = mpu.getXAccelOffset();
    resultado.offset_accel_y = mpu.getYAccelOffset();
    resultado.offset_accel_z = mpu.getZAccelOffset();
    resultado.offset_gyro_x  = mpu.getXGyroOffset();
    resultado.offset_gyro_y  = mpu.getYGyroOffset();
    resultado.offset_gyro_z  = mpu.getZGyroOffset();

    enviarResultado(resultado);

    Serial.println("Resultado enviado para a ponte. Aguardando proximo OTA.");
}

void loop() {
    otaProcessarPendencias();

    // Nada aqui - so espera o proximo OTA (firmware normal do equip).
}