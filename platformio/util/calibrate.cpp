#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "ota_receptor.h"
#include <Preferences.h>


const int CANAL = 1;
uint8_t PONTE_MAC[] = {0x14, 0x33, 0x5C, 0x2D, 0xF3, 0x68}; 

MPU6050 mpu;
const int callibration_time = 6;
const int touch_sensitivity = 20;

const char *PREF_NAMESPACE = "contato";
const char *PREF_KEY_OFFS  = "mpu_offs";

typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} MPUOffsets;

Preferences prefs;

typedef struct {
    int16_t offset_accel_x;
    int16_t offset_accel_y;
    int16_t offset_accel_z;
    int16_t offset_gyro_x;
    int16_t offset_gyro_y;
    int16_t offset_gyro_z;
} calibracao_resultado_t;

esp_now_peer_info_t peerPonte;

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

    Serial.println("Pronto. Encoste no toque para iniciar a calibracao.");

    while (touchRead(T3) >= touch_sensitivity) {
        delay(50);
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

    MPUOffsets offs;
    offs.accelX = resultado.offset_accel_x;
    offs.accelY = resultado.offset_accel_y;
    offs.accelZ = resultado.offset_accel_z;
    offs.gyroX  = resultado.offset_gyro_x;
    offs.gyroY  = resultado.offset_gyro_y;
    offs.gyroZ  = resultado.offset_gyro_z;

    prefs.begin(PREF_NAMESPACE, false);
    size_t wrote = prefs.putBytes(PREF_KEY_OFFS, &offs, sizeof(MPUOffsets));
    prefs.end();

    if (wrote == sizeof(MPUOffsets)) {
        Serial.println("Offsets salvos na NVS.");
    } else {
        Serial.printf("ERRO salvando offsets na NVS: escreveu %u de %u bytes\n",
                      (unsigned)wrote, (unsigned)sizeof(MPUOffsets));
    }

    enviarResultado(resultado);

    Serial.println("Resultado enviado para a ponte. Aguardando proximo OTA.");
}

void loop() {
    otaProcessarPendencias();

}