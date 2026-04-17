// ═════════ Bibliotecas ═════════
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h"
#include "esp_log.h"

// ═════════ Defines ═════════
// #define USE_DELAY
// #define AUTO_CALLIBRATION
// #define PRINT_MAC        // Imprime o MAC deste dispositivo no boot
// #define PRINT_CANAL      // Imprime o canal Wi-Fi configurado no boot
// #define PRINT_SENSOR     // Imprime os valores do sensor em tempo real

// ═════════ ALTERAR POR CONJUNTO ═════════
const uint8_t ID = 6;
const uint8_t MEU_SLOT = 3;           // slot 3 = equip 6
const int CANAL = 8;
uint8_t broadcastAddress[] = {0x14, 0x33, 0x5C, 0x30, 0x2B, 0x58}; // MAC da base_6
const int delay_time = 10;
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
uint16_t    fifo_count;
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

    #ifdef PRINT_CANAL
        uint8_t primaryChan;
        wifi_second_chan_t secondChan;
        esp_wifi_get_channel(&primaryChan, &secondChan);
        Serial.print("Canal real configurado: ");
        Serial.println(primaryChan);
    #endif
    return result;
}

// ═════════ setup ═════════
void setup() {
    setCpuFrequencyMhz(80);
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_NONE);

    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);

    // Offsets antes do resetFIFO — ordem correta
    #ifndef AUTO_CALLIBRATION
        mpu.setZAccelOffset(1440);
        mpu.setXGyroOffset(59);
        mpu.setYGyroOffset(38);
        mpu.setZGyroOffset(20);
    #endif

    if (dev_status == 0) {
        #ifdef AUTO_CALLIBRATION
            mpu.CalibrateAccel(callibration_time);
            mpu.CalibrateGyro(callibration_time);
            mpu.PrintActiveOffsets();
        #endif
        dmp_ready = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(dev_status);
    }

    // Limpa FIFO acumulado após calibração
    delay(100);
    mpu.resetFIFO();

    WiFi.mode(WIFI_STA);
    setChannel(CANAL);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    #ifdef PRINT_MAC
        Serial.print("MAC deste dispositivo: ");
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        for (int i = 0; i < 6; i++) {
            Serial.print("0x");
            if (mac[i] < 0x10) Serial.print("0");
            Serial.print(mac[i], HEX);
            if (i < 5) Serial.print(", ");
        }
        Serial.println();
    #endif

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Recebe beacon da base mestre
    esp_now_register_recv_cb(OnDataRecv);

    // Peer da base individual
    peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
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

        #ifdef PRINT_SENSOR
            char buf[64];
            snprintf(buf, sizeof(buf), "id:%d gyro:%d accel:%d touch:%d",
                     message.id, message.gyro, message.accel, message.touch);
            Serial.println(buf);
        #endif
    } else {
        delay(1);
    }

    // Transmite apenas quando a base mestre abrir o slot
    if (meu_slot_aberto) {
        meu_slot_aberto = false;
        esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));
    }
}