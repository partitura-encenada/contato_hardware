// Bibliotecas
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "esp_wifi.h" 


#define USE_DELAY   // Comente esta linha para desativar o delay
#define DEBUG
// #define AUTO_CALLIBRATION

// Constantes e pseudo-constantes
const int   delay_time = 10;
const int   touch_sensitivity = 20;  
const int   callibration_time = 6;  
const int   CANAL_ESPECIFICO = 10;

MPU6050 mpu;

uint8_t     dev_status;      
uint16_t    packet_size;   
uint16_t    fifo_count;    
uint8_t     fifo_buffer[64];
Quaternion  q;              // [w, x, y, z]         Quaternion 
VectorInt16 aa;             // [x, y, z]            Accel
VectorInt16 aaReal;         // [x, y, z]            Accel sem gravidade
VectorFloat gravity;        // [x, y, z]            Gravidade
bool        dmp_ready = false;  
float       ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll
uint8_t     broadcastAddress[] = {0x78, 0xe3, 0x6d, 0xd8, 0x16, 0xd4};

typedef struct { // Struct da mensagem, deve ser igual ao da base 
    int id = 9;
    int roll;
    int accel;
    int touch;
} message_t;
message_t message;
esp_now_peer_info_t peerInfo;


// Função para definir o canal
esp_err_t setChannel(int channel) {
  esp_wifi_set_promiscuous(true);  
  esp_err_t result = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  #ifdef DEBUG
    uint8_t primaryChan;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);
    Serial.print("Canal configurado: ");
    Serial.println(primaryChan);
  #endif

  return result;
}

void setup() {
    setCpuFrequencyMhz(80);
    // Inicializar Wire, Serial (caso monitorando) e MPU
    Wire.begin();
    Wire.setClock(400000); // Clock I2C 400khz. Comente caso erro na compilação 
    #ifdef DEBUG
        Serial.begin(115200);
    #endif
    mpu.initialize();
    #ifdef DEBUG
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));  
    #endif  

    // DMP
    dev_status = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);       
    #ifndef AUTO_CALLIBRATION
        mpu.setZAccelOffset(1718);
        mpu.setXGyroOffset(-30);    
        mpu.setYGyroOffset(-20);    
        mpu.setZGyroOffset(19);    
    #endif
 
    if (dev_status == 0) { // Sucesso
        #ifdef AUTO_CALLIBRATION
            mpu.CalibrateAccel(callibration_time);
            mpu.CalibrateGyro(callibration_time);
            mpu.PrintActiveOffsets();
        #endif
        dmp_ready = true;
        packet_size = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code ")); // Erro
        Serial.print(dev_status); // 1 = "initial memory load failed"; 2 = "DMP configuration updates failed"
    }

    // CONFIGURA WI-FI NO CANAL ESPECÍFICO
    WiFi.mode(WIFI_STA);           
    setChannel(CANAL_ESPECIFICO);
    esp_wifi_set_max_tx_power(82);

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

    // Inicia o ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Configuração do peer
    peerInfo = {}; // inicializa a variável global, sem redeclara-la
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0; // usa o canal já configurado no Wi-Fi
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
    // Serial.println("Peer adicionado e ESP-NOW pronto!");
}

void loop() {
    if (!dmp_ready) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) { 
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifo_buffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // message.yaw =   ypr[0] * 180/M_PI;      // -180º >=     yaw     <= +180º
        // message.pitch = ypr[1] * 180/M_PI;      // -180º >=     pitch   <= +180º
        message.roll =  ypr[2] * 180/M_PI;      // -180º >=     roll    <= +180º
        message.accel = aaReal.x;
        message.touch = (touchRead(T3) < touch_sensitivity) ? 1 : 0; //mudança message.touch = 1 ? touchRead(T3) < 20 : 0;

        #ifdef DEBUG
            Serial.print("Touch raw value: ");
            Serial.println(touchRead(T3));
        #endif

        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message)); // Casta pointer para uint8_t e envia mensagem para peer 

    #ifdef USE_DELAY
        delay(delay_time);
    #endif
    }  
}
