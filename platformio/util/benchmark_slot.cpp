// ═════════ BENCHMARK SLOT ESP-NOW — equip_3 ═════════
// Mede tempo de envio e taxa de perda de pacotes com slot de duração fixa
// Requer benchmark_ack.cpp rodando na base_3
// Canal 8, MAC da base_3, struct real do equip (8 bytes)

#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h"
#include "esp_log.h"

const int    CANAL        = 8;
const int    NUM_AMOSTRAS = 500;
const uint32_t SLOT_US   = 1400; // ← duração fixa do slot a testar (altere aqui)

// MAC da base_3
uint8_t broadcastAddress[] = {0x14, 0x33, 0x5C, 0x2E, 0xE6, 0x88};

// ─── struct do pacote equip → base ───
typedef struct {
    uint8_t  id;
    int16_t  gyro;
    int32_t  accel;
    uint8_t  touch;
    uint16_t seq;
} message_t;         // Total: 10 bytes

// ─── struct do ACK base → equip ───
typedef struct {
    uint16_t seq;
} ack_t;

message_t message;
esp_now_peer_info_t peerInfo;

// ─── MPU6050 ───
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

// ─── variáveis de medição ───
volatile bool     envio_concluido = false;
volatile bool     ack_recebido    = false;
volatile uint16_t ack_seq         = 0;
volatile uint32_t tempo_envio_us  = 0;
uint32_t          t_inicio        = 0;

uint32_t amostras[NUM_AMOSTRAS];
int      amostra_idx         = 0;
int      pacotes_perdidos     = 0;
bool     benchmark_concluido = false;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    tempo_envio_us  = micros() - t_inicio;
    envio_concluido = true;
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (len != sizeof(ack_t)) return;
    ack_t ack;
    memcpy(&ack, incomingData, sizeof(ack_t));
    ack_seq      = ack.seq;
    ack_recebido = true;
}

esp_err_t setChannel(int channel) {
    esp_wifi_set_promiscuous(true);
    esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    return result;
}

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

    mpu.setZAccelOffset(1590);
    mpu.setXGyroOffset(166);
    mpu.setYGyroOffset(-44);
    mpu.setZGyroOffset(49);

    if (dev_status == 0) {
        dmp_ready   = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(dev_status);
    }

    delay(100);
    mpu.resetFIFO();

    WiFi.mode(WIFI_STA);
    setChannel(CANAL);
    esp_wifi_set_max_tx_power(82);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Erro ao adicionar peer");
        return;
    }

    Serial.println("Iniciando benchmark...");
    Serial.print("Pacote: "); Serial.print(sizeof(message_t)); Serial.println(" bytes");
    Serial.print("Canal: "); Serial.println(CANAL);
    Serial.print("Slot: "); Serial.print(SLOT_US); Serial.println(" µs");
    Serial.print("Amostras: "); Serial.println(NUM_AMOSTRAS);
    Serial.println("─────────────────────────────────");
}

void loop() {
    if (benchmark_concluido) return;
    if (!dmp_ready) return;

    // lê MPU para dados reais
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifo_buffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        message.id    = 3;
        message.gyro  = (int16_t)(ypr[2] * 180 / M_PI);
        message.accel = (int32_t)aaReal.x;
        message.touch = (touchRead(T3) < 20) ? 1 : 0;
    }

    if (amostra_idx < NUM_AMOSTRAS) {
        message.seq     = amostra_idx;
        envio_concluido = false;
        ack_recebido    = false;

        // marca início do slot
        t_inicio = micros();
        esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));

        // aguarda confirmação de envio
        while (!envio_concluido);
        amostras[amostra_idx] = tempo_envio_us;

        // aguarda ACK até o slot terminar
        while (!ack_recebido && (micros() - t_inicio) < SLOT_US);

        if (!ack_recebido || ack_seq != amostra_idx) {
            pacotes_perdidos++;
        }

        // aguarda o restante do slot
        uint32_t tempo_gasto = micros() - t_inicio;
        if (tempo_gasto < SLOT_US)
            delayMicroseconds(SLOT_US - tempo_gasto);

        amostra_idx++;

    } else {
        // calcula resultados
        uint32_t soma = 0, minimo = UINT32_MAX, maximo = 0;
        for (int i = 0; i < NUM_AMOSTRAS; i++) {
            soma += amostras[i];
            if (amostras[i] < minimo) minimo = amostras[i];
            if (amostras[i] > maximo) maximo = amostras[i];
        }
        float media      = (float)soma / NUM_AMOSTRAS;
        float taxa_perda = (float)pacotes_perdidos / NUM_AMOSTRAS * 100.0;

        Serial.println("═════════ RESULTADO ═════════");
        Serial.print("Slot testado : "); Serial.print(SLOT_US); Serial.println(" µs");
        Serial.println("─────────────────────────────");
        Serial.print("Mínimo : "); Serial.print(minimo);   Serial.println(" µs");
        Serial.print("Máximo : "); Serial.print(maximo);   Serial.println(" µs");
        Serial.print("Média  : "); Serial.print(media, 1); Serial.println(" µs");
        Serial.println("─────────────────────────────");
        Serial.print("Mínimo em ms: "); Serial.print(minimo / 1000.0, 3); Serial.println(" ms");
        Serial.print("Máximo em ms: "); Serial.print(maximo / 1000.0, 3); Serial.println(" ms");
        Serial.print("Média  em ms: "); Serial.print(media  / 1000.0, 3); Serial.println(" ms");
        Serial.println("─────────────────────────────");
        Serial.print("Pacotes perdidos: "); Serial.print(pacotes_perdidos);
        Serial.print(" / "); Serial.println(NUM_AMOSTRAS);
        Serial.print("Taxa de perda: "); Serial.print(taxa_perda, 2); Serial.println(" %");
        Serial.println("═════════════════════════════");

        benchmark_concluido = true;
    }
}