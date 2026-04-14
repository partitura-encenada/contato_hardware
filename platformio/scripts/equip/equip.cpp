//═════════ Bibliotecas ═════════
#include "MPU6050_6Axis_MotionApps20.h" // Biblioteca do MPU6050 com suporte ao DMP (Digital Motion Processor)
#include <esp_now.h>                    // Protocolo ESP-NOW para comunicação sem fio entre ESP32s
#include <WiFi.h>                       // Necessário para inicializar o Wi-Fi (base do ESP-NOW)
#include "Wire.h"                       // Comunicação I2C com o MPU6050
#include "esp_wifi.h"                   // Funções avançadas de Wi-Fi (canal, potência, taxa)

//═════════ Defines ═════════
// #define USE_DELAY          // Ativa delay fixo no loop. Comente para desativar
// #define AUTO_CALLIBRATION  // Ativa calibração automática do MPU no boot. Comente para usar offsets manuais
// #define PRINT_MAC          // Imprime o MAC deste dispositivo no boot
// #define PRINT_CANAL        // Imprime o canal Wi-Fi configurado no boot
// #define PRINT_SENSOR       // Imprime os valores do sensor em tempo real

//═════════ ALTERAR POR CONJUNTO ═════════                  
const uint8_t ID = 3;                // Identificador único deste conjunto (3–8)
const int CANAL_ESPECIFICO = 2;      // Canal Wi-Fi usado pelo ESP-NOW (deve ser igual ao da base)
uint8_t broadcastAddress[] = {0x14, 0x33, 0x5C, 0x2E, 0xE6, 0x88}; // Endereço MAC da base deste conjunto
const int delay_time = 10;           // Tempo de delay em ms (usado apenas se USE_DELAY ativo)
const int touch_sensitivity = 20;    // leituras abaixo desse valor = tocado
const int callibration_time = 6;     // Número de ciclos de calibração automática do MPU


//═════════ Variáveis do MPU6050 ═════════
MPU6050 mpu; // Objeto principal do sensor

uint8_t     dev_status;       // Resultado da inicialização do DMP (0 = sucesso)
uint16_t    packet_size;      // Tamanho esperado de cada pacote do DMP
uint16_t    fifo_count;       // Quantidade de bytes atualmente no FIFO do DMP
uint8_t     fifo_buffer[64];  // Buffer para armazenar um pacote lido do FIFO
Quaternion  q;                // Quaternion de orientação [w, x, y, z] calculado pelo DMP
VectorInt16 aa;               // Aceleração bruta [x, y, z] em unidades de 16384 LSB/g
VectorInt16 aaReal;           // Aceleração linear [x, y, z] com a gravidade removida
VectorFloat gravity;          // Vetor de gravidade estimado a partir do quaternion
bool        dmp_ready = false; // Flag: indica se o DMP foi inicializado com sucesso
float       ypr[3];           // Ângulos calculados pelo DMP: [yaw, pitch, roll] em radianos


//═════════ Struct da mensagem ESP-NOW ═════════
// Deve ser idêntica à struct da base — o ESP-NOW transmite os bytes crus,
// então qualquer diferença de tipo ou ordem corrompe os dados recebidos
typedef struct {
    uint8_t  id;    // 1 byte  — ID do conjunto (3–8)
    int16_t  gyro;  // 2 bytes — ângulo de roll em graus (-180 a +180)
    int32_t  accel; // 4 bytes — aceleração linear em X (int32 pois pode ultrapassar ±32767 em impactos)
    uint8_t  touch; // 1 byte  — estado do touch (0 = não tocado, 1 = tocado)
} message_t;        // Total: 8 bytes (vs 16 bytes com int para tudo)
message_t message;            // Instância da mensagem a ser enviada
esp_now_peer_info_t peerInfo; // Informações do peer (base) para o ESP-NOW


//═════════ setChannel ═════════
// Define o canal Wi-Fi do ESP32 via modo promíscuo
// O modo promíscuo é necessário porque o canal só pode ser alterado enquanto ele está ativo
esp_err_t setChannel(int channel) {
    esp_wifi_set_promiscuous(true);  // Ativa modo promíscuo para permitir troca de canal
    esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE); // Define canal primário, sem canal secundário
    esp_wifi_set_promiscuous(false); // Desativa modo promíscuo após a troca

    #ifdef PRINT_CANAL
        uint8_t primaryChan;
        wifi_second_chan_t secondChan;
        esp_wifi_get_channel(&primaryChan, &secondChan); // Lê o canal real configurado para confirmar
        Serial.print("Canal real configurado: ");
        Serial.println(primaryChan);
    #endif

    return result; // Retorna ESP_OK em caso de sucesso
}


//═════════ setup ═════════
void setup() {
    setCpuFrequencyMhz(80); // Reduz clock de 240MHz para 80MHz — economiza energia sem impactar o DMP
    Wire.begin();           // Inicializa barramento I2C nos pinos padrão (SDA=21, SCL=22)
    Wire.setClock(400000);  // Define clock I2C em 400kHz (modo fast) para leitura mais rápida do MPU

    Serial.begin(115200); // Inicializa Serial — necessário para estabilidade no boot e prints de erro

    mpu.initialize(); // Inicializa o MPU6050 via I2C (liga o sensor e configura registradores básicos)
    // Testa a conexão I2C com o MPU6050 e imprime resultado
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Inicializa o DMP (Digital Motion Processor) — processador interno do MPU que calcula quaternions
    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true); // Habilita o DMP após inicialização

    #ifndef AUTO_CALLIBRATION
        // Offsets manuais calculados previamente para este sensor específico
        // Compensam erros de fabricação e montagem do MPU6050
        mpu.setZAccelOffset(1590);
        mpu.setXGyroOffset(166);
        mpu.setYGyroOffset(-44);
        mpu.setZGyroOffset(49);
    #endif

    if (dev_status == 0) { // DMP inicializado com sucesso
        #ifdef AUTO_CALLIBRATION
            // Calibração automática: mantém o sensor em posição plana durante o processo
            mpu.CalibrateAccel(callibration_time);
            mpu.CalibrateGyro(callibration_time);
            mpu.PrintActiveOffsets(); // Imprime os offsets calculados
        #endif
        dmp_ready = true;   // Libera o loop para começar a ler dados
        packet_size = mpu.dmpGetFIFOPacketSize();  // Obtém o tamanho fixo de cada pacote do FIFO
    }
    else {
        // Erros possíveis: 1 = falha ao carregar firmware do DMP, 2 = falha na configuração
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(dev_status);
    }

    WiFi.mode(WIFI_STA);          // Configura Wi-Fi em modo station (necessário para o ESP-NOW funcionar)
    setChannel(CANAL_ESPECIFICO); // Fixa o canal Wi-Fi no canal definido para este conjunto
    esp_wifi_set_max_tx_power(82); // Potência máxima de transmissão (82 = 20.5 dBm, máximo do ESP32)
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_1M_L); // Taxa 1Mbps com preâmbulo longo — mais robusto a interferências

    #ifdef PRINT_MAC
        // Imprime o MAC deste dispositivo — necessário para configurar o broadcastAddress da base
        Serial.print("MAC deste dispositivo: ");
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        for (int i = 0; i < 6; i++) {
            Serial.print("0x");
            if (mac[i] < 0x10) Serial.print("0"); // Padding para manter formato 0xXX
            Serial.print(mac[i], HEX);
            if (i < 5) Serial.print(", ");
        }
        Serial.println();
    #endif

    if (esp_now_init() != ESP_OK) { // Inicializa o protocolo ESP-NOW
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    peerInfo = {};                                    // Zera a struct sem redeclará-la
    memcpy(peerInfo.peer_addr, broadcastAddress, 6); // Copia o MAC da base para o peer
    peerInfo.channel = 0;                            // 0 = usa o canal já configurado no Wi-Fi
    peerInfo.encrypt = false;                        // Sem criptografia (reduz latência)
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {     // Registra a base como destinatário fixo
        Serial.println("Failed to add peer");
        return;
    }
}


//═════════ loop ═════════
void loop() {
    if (!dmp_ready) return; // Aguarda DMP estar pronto antes de qualquer leitura

    // Tenta ler o pacote mais recente do FIFO do DMP
    // Retorna true apenas quando há um pacote completo disponível
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {

        mpu.dmpGetQuaternion(&q, fifo_buffer);         // Extrai quaternion de orientação do pacote
        mpu.dmpGetGravity(&gravity, &q);               // Calcula vetor de gravidade a partir do quaternion (uma única vez)
        mpu.dmpGetAccel(&aa, fifo_buffer);             // Extrai aceleração bruta do pacote
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);    // Calcula yaw/pitch/roll usando quaternion e gravidade
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // Remove componente gravitacional da aceleração bruta

        message.id    = ID;                                           // ID fixo deste conjunto
        message.gyro  = (int16_t)(ypr[2] * 180 / M_PI);             // Converte roll de radianos para graus inteiros
        message.accel = (int32_t)aaReal.x;                           // Aceleração linear no eixo X
        message.touch = (touchRead(T3) < touch_sensitivity) ? 1 : 0; // 1 se tocado, 0 se não tocado

        // Envia a struct como array de bytes para o MAC da base
        // sizeof(message) garante que apenas os bytes da struct são enviados
        esp_now_send(broadcastAddress, (uint8_t *)&message, sizeof(message));

        #ifdef PRINT_SENSOR
            char buf[64];
            snprintf(buf, sizeof(buf), "id:%d gyro:%d accel:%d touch:%d",
                     message.id,
                     message.gyro,
                     message.accel,
                     message.touch);
            Serial.println(buf);
        #endif

        #ifdef USE_DELAY
            delay(delay_time); // Delay opcional — aumenta latência mas pode ajudar em casos de instabilidade
        #endif
    } else {
        delay(1); // Feed do watchdog timer quando não há pacote disponível no FIFO
    }
}