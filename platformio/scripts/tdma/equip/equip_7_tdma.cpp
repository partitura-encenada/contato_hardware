// ═════════ Bibliotecas ═════════
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h"
#include "esp_log.h"

// ═════════ Defines ═════════
// #define DEBUG
// #define USE_DELAY
// #define AUTO_CALLIBRATION

// ═════════ ALTERAR POR CONJUNTO ═════════
const uint8_t ID = 7;
const uint8_t MEU_SLOT = 4;           // slot 0 = equip 3
const int CANAL = 8;
uint8_t broadcastAddress[] = {0x14, 0x33, 0x5C, 0x2E, 0x12, 0xC8}; // MAC da base_3
const int touch_sensitivity = 20;
const int callibration_time = 6;

// ═════════ Struct beacon da base mestre ═════════
typedef struct {
    uint8_t slot_atual;
    uint32_t timestamp;
} beacon_t;

// ═════════ Struct mensagem para base individual ═════════
typedef struct {
    uint8_t  id;
    int16_t  gyro;
    int32_t  accel;
    uint8_t  touch;
} message_t;

// ═════════ Variáveis MPU6050 ═════════
MPU6050 mpu;
uint8_t     dev_status;
uint16_t    packet_size;
uint8_t     fifo_buffer[64];
Quaternion  q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
bool        dmp_ready = false;
float       ypr[3];

message_t message;
esp_now_peer_info_t peerInfo;
volatile bool meu_slot_aberto = false;

// ═════════ Callback beacon ═════════
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (len != sizeof(beacon_t)) return;
    beacon_t beacon;
    memcpy(&beacon, incomingData, sizeof(beacon_t));
    if (beacon.slot_atual == MEU_SLOT) {
        meu_slot_aberto = true;
    }
}

// ═════════ setChannel ═════════
esp_err_t setChannel(int channel) {
    esp_wifi_set_promiscuous(true);
    esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    return result;
}

// ═════════ setup ═════════
void setup() {
    setCpuFrequencyMhz(80);
    Wire.begin();
    Wire.setClock(400000);

    #ifdef DEBUG
        Serial.begin(115200);
        esp_log_level_set("*", ESP_LOG_NONE);
    #endif

    mpu.initialize();
    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);

    #ifndef AUTO_CALLIBRATION
        mpu.setZAccelOffset(1098);
        mpu.setXGyroOffset(177);
        mpu.setYGyroOffset(78);
        mpu.setZGyroOffset(1);
    #endif

    if (dev_status == 0) {
        #ifdef AUTO_CALLIBRATION
            mpu.CalibrateAccel(callibration_time);
            mpu.CalibrateGyro(callibration_time);
        #endif
        dmp_ready = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    }

    WiFi.mode(WIFI_STA);
    setChannel(CANAL);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    if (esp_now_init() != ESP_OK) return;

    // Recebe beacon da base mestre
    esp_now_register_recv_cb(OnDataRecv);

    // Peer da base individual
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

// ═════════ loop ═════════
void loop() {
    if (!dmp_ready) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifo_buffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        message.id    = ID;
        message.gyro  = (int16_t)(ypr[2] * 180 / M_PI);
        message.accel = (int32_t)aaReal.x;
        message.touch = (touchRead(T3) < touch_sensitivity) ? 1 : 0;
    } else {
        delay(1);
    }

    // Transmite apenas quando a base mestre abrir o slot
    if (meu_slot_aberto) {
        meu_slot_aberto = false;
        esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));
    }
}
