# 1 "C:\\Users\\cbred\\AppData\\Local\\Temp\\tmp03l5nggr"
#include <Arduino.h>
# 1 "C:/Users/cbred/Projetos/contato-hardware/platformio/src/main.ino"

#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;
#define INTERRUPT_PIN 35
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void setup();
void loop();
int touchRead();
#line 27 "C:/Users/cbred/Projetos/contato-hardware/platformio/src/main.ino"
void dmpDataReady() {
    mpuInterrupt = true;
}


uint8_t broadcastAddress[] = {0x84, 0xcc, 0xa8, 0x5d, 0x63, 0x90};







typedef struct struct_message {
    int id;
    int gyro;
    int accel;
    int touch;
} struct_message;

struct_message MIDImessage;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {


}


float ypr_mod = 0;
int mediaAccel;
int buttonState;
int lastButtonState = 0;


void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  Serial.begin(115200);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));



  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {

          mpu.CalibrateAccel(6);
          mpu.CalibrateGyro(6);
          mpu.PrintActiveOffsets();

          Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);


          Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
          Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
          Serial.println(F(")..."));
          attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();


          Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;


          packetSize = mpu.dmpGetFIFOPacketSize();
      }
      else {




          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
      }



  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() {

    MIDImessage.id = 6;

    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        ypr_mod = ypr[2] * 180/M_PI;

        int pressed = touchRead();

        MIDImessage.gyro = ypr_mod;
        MIDImessage.accel = mediaAccel;
        MIDImessage.touch = pressed;
        esp_now_send(broadcastAddress, (uint8_t *) &MIDImessage, sizeof(MIDImessage));
        Serial.println("MIDI SENT");
   }

  delay(10);
}


int touchRead()
{
    int media = 0;
    mediaAccel = 0;
    for(int i=0; i< 100; i++)
    {
      media += touchRead(T3);
      mediaAccel += aaReal.x;

    }
    media = media/100;
    mediaAccel = mediaAccel/100;
    if (media < 20)
    {
      return 1;
    }
    else
    {
      return 0;
    }
}