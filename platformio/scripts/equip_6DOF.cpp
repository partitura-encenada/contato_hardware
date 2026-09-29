// Documentação: README.md, seção "Equip e Base 6DOF".

#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "ota_receptor.h"

// #define PRINT_MAC
// #define PRINT_CHANNEL
// #define PRINT_SENSOR

// ═════════ Configuração ═════════
const bool RAW_DATA = true;         // true = brutos; false = tratados pelo DMP (igual ao equip_N)

// ALTERAR conforme o equip físico (valores atuais: equip 6)
const int LED_BLUE = 2;
const uint8_t ID = 6;
const uint8_t MY_SLOT = 5;          // posição no TDMA; a mesma do equip_N substituído
const int CHANNEL = 11;
uint8_t baseAddress[] = {0x14, 0x08, 0x08, 0xA4, 0x59, 0xE8};  // MAC da base_6DOF
const int touch_sensitivity = 20;
const uint32_t RAW_READ_INTERVAL_US = 2000;  // leitura do sensor no modo bruto (500 Hz)

// ═════════ Mensagens ═════════
// Beacon do mestre TDMA (mesmo formato do TDMA.cpp)
typedef struct {
    uint8_t current_slot;
    uint32_t timestamp;
} beacon_t;

// Controle da base: ativo=1 transmite, ativo=0 para
typedef struct {
    uint8_t active;
} control_t;

// Mesma struct da base_6DOF (15 bytes)
typedef struct __attribute__((packed)) {
    uint8_t id;
    uint8_t mode;       // 'B' = brutos, 'T' = tratados
    int16_t rot[3];
    int16_t acc[3];
    uint8_t touch;
} message_6dof_t;

// ═════════ Estado ═════════
MPU6050 mpu;
uint8_t     dev_status;
uint8_t     fifo_buffer[64];
Quaternion  q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorFloat gravity;
bool        sensor_ready = false;
float       ypr[3];
uint32_t    last_raw_read_us = 0;

message_6dof_t message;
esp_now_peer_info_t peerInfo;
volatile bool my_slot_open = false;
volatile bool transmission_active = false;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (otaProcessarPacote(mac_addr, incomingData, len)) return;

    if (len == sizeof(beacon_t)) {
        beacon_t beacon;
        memcpy(&beacon, incomingData, sizeof(beacon_t));
        if (beacon.current_slot == MY_SLOT) {
            my_slot_open = true;
        }
        return;
    }

    if (len == sizeof(control_t)) {
        control_t control;
        memcpy(&control, incomingData, sizeof(control_t));
        transmission_active = (control.active == 1);
        return;
    }
}

esp_err_t setChannel(int channel) {
    esp_wifi_set_promiscuous(true);
    esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    #ifdef PRINT_CHANNEL
        uint8_t primaryChan;
        wifi_second_chan_t secondChan;
        esp_wifi_get_channel(&primaryChan, &secondChan);
        Serial.print("Canal real configurado: ");
        Serial.println(primaryChan);
    #endif
    return result;
}

void applyOffsets() {
    // ALTERAR: offsets calibrados do equip físico (contato calibrate --id N --port COMx)
    mpu.setXAccelOffset(-2228);
    mpu.setYAccelOffset(-1953);
    mpu.setZAccelOffset(3546);
    mpu.setXGyroOffset(49);
    mpu.setYGyroOffset(-30);
    mpu.setZGyroOffset(-6);
}

void setupSensor() {
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    if (RAW_DATA) {
        // mesmas faixas e filtro do DMP
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setDLPFMode(MPU6050_DLPF_BW_42);
        applyOffsets();
        sensor_ready = true;
        return;
    }

    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    applyOffsets();

    if (dev_status == 0) {
        sensor_ready = true;
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(dev_status);
    }

    delay(100);
    mpu.resetFIFO();
}

// Retorna false se não havia leitura nova
bool readSensor() {
    if (RAW_DATA) {
        uint32_t now = micros();
        if (now - last_raw_read_us < RAW_READ_INTERVAL_US) return false;
        last_raw_read_us = now;

        // variáveis locais: ponteiro para campo packed pode travar o ESP32
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        message.rot[0] = gx;
        message.rot[1] = gy;
        message.rot[2] = gz;
        message.acc[0] = ax;
        message.acc[1] = ay;
        message.acc[2] = az;
        message.mode = 'B';
        return true;
    }

    if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) return false;

    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifo_buffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    for (int i = 0; i < 3; i++) {
        message.rot[i] = (int16_t)(ypr[i] * 180 / M_PI);
    }
    message.acc[0] = aaReal.x;
    message.acc[1] = aaReal.y;
    message.acc[2] = aaReal.z;
    message.mode = 'T';
    return true;
}

void setup() {
    setCpuFrequencyMhz(80);
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    pinMode(LED_BLUE, OUTPUT);
    esp_log_level_set("*", ESP_LOG_NONE);

    // id e modo certos mesmo antes da primeira leitura
    message.id   = ID;
    message.mode = RAW_DATA ? 'B' : 'T';

    setupSensor();

    WiFi.mode(WIFI_STA);
    setChannel(CHANNEL);
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

    esp_now_register_recv_cb(OnDataRecv);

    peerInfo = {};
    memcpy(peerInfo.peer_addr, baseAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    otaProcessarPendencias();

    if (!sensor_ready) return;

    if (readSensor()) {
        message.id    = ID;
        message.touch = (touchRead(T3) < touch_sensitivity) ? 1 : 0;

        digitalWrite(LED_BLUE, transmission_active && message.touch);

        #ifdef PRINT_SENSOR
            char buf[96];
            snprintf(buf, sizeof(buf), "id:%d modo:%c rot:%d,%d,%d acc:%d,%d,%d touch:%d",
                     message.id, message.mode,
                     message.rot[0], message.rot[1], message.rot[2],
                     message.acc[0], message.acc[1], message.acc[2],
                     message.touch);
            Serial.println(buf);
        #endif
    } else {
        delay(1);
    }

    // só transmite na posição aberta pelo TDMA
    if (my_slot_open) {
        my_slot_open = false;
        if (transmission_active) {
            esp_now_send(baseAddress, (uint8_t *)&message, sizeof(message));
        }
    }
}
