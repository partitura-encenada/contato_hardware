//LIBRARIES
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
<<<<<<< HEAD
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//MPU Initialization
MPU6050 mpu;
#define INTERRUPT_PIN 35
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;   
uint16_t fifoCount;    
uint8_t fifoBuffer[64];
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;    
void dmpDataReady() {
    mpuInterrupt = true;
}

//ESPNOW Initialization
uint8_t broadcastAddress[] = {0x84, 0xcc, 0xa8, 0x5d, 0x63, 0x90};
// Base 3: 0xcc, 0xdb, 0xa7, 0x91, 0x47, 0xe8
// Base 4: 0xb0, 0xa7, 0x32, 0xd7, 0x58, 0x7c
// Base 5: 0x40, 0x22, 0xd8, 0x4f, 0x5f, 0xd8
// Base 6: 0x84, 0xcc, 0xa8, 0x5d, 0x63, 0x90
// Base 7: 0xcc, 0xdb, 0xa7, 0x91, 0x53, 0x00 

//Message Struct
=======
#include "Wire.h"
#include <EEPROM.h>

//MPU Initialization
MPU6050 mpu;
bool dmpReady = false;  
uint8_t dev_status;      
uint16_t fifo_count;    
uint8_t fifo_buffer[64];
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    
float ypr[3];

//ESP-NOW Initialization
uint8_t broadcastAddress[] = {0xf8, 0xb3, 0xb7, 0x2b, 0x09, 0x48};

>>>>>>> 0e86ae4718a74856566d12b21395ad2cbd86b72a
typedef struct struct_message {
    int id; // must be unique for each sender board
    int gyro;
    int accel;
    int touch;
} struct_message;
<<<<<<< HEAD

struct_message MIDImessage;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//Variables
float ypr_mod = 0;
int mediaAccel;
int buttonState;             // the current reading from the input pin
int lastButtonState = 0;


void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  Serial.begin(115200);
  //GYRO Initialization
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again 
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //Offsets
  mpu.setXGyroOffset(220);//220
  mpu.setYGyroOffset(76);//76
  mpu.setZGyroOffset(-85);//-85
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (devStatus == 0) {
          // Calibration Time: generate offsets and calibrate our MPU6050
          mpu.CalibrateAccel(6);
          mpu.CalibrateGyro(6);
          mpu.PrintActiveOffsets();
          // turn on the DMP, now that it's ready
          Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);
  
          // enable Arduino interrupt detection
          Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
          Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
          Serial.println(F(")..."));
          attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();
  
          // set our DMP Ready flag so the main loop() function knows it's okay to use it
          Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;
  
          // get expected DMP packet size for later comparison
          packetSize = mpu.dmpGetFIFOPacketSize();
      } 
      else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
      }

=======
struct_message message;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Pacote enviado" : "Falha no envio");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  dev_status = mpu.dmpInitialize();

  if (dev_status == 0) { // Sucesso
      //  TENTANDO SALVAR OFFSETS MEMORIA PERMANENTE DO ESP XD
      EEPROM.begin(128);
      if (EEPROM.readShort(0) == 0)
      {
          mpu.CalibrateAccel(6);
          mpu.CalibrateGyro(6);
          int16_t* offsets = mpu.GetActiveOffsets();
          for (int i = 0; i < 6; i++)
          {
              EEPROM.writeShort(i*16, offsets[i]);
          }
      }
      else 
      {   
          mpu.setZAccelOffset(EEPROM.readShort(2 * 16));
          mpu.setXGyroOffset(EEPROM.readShort(3 * 16));
          mpu.setYGyroOffset(EEPROM.readShort(4 * 16));
          mpu.setZGyroOffset(EEPROM.readShort(5 * 16));
      }
      // EEPROM.writeShort(0,0); // Descomente para resetar a calibração
      EEPROM.end();
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
      dmpReady = true;
    }
>>>>>>> 0e86ae4718a74856566d12b21395ad2cbd86b72a

  //ESPNOW Initialization
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
<<<<<<< HEAD
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
=======
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
>>>>>>> 0e86ae4718a74856566d12b21395ad2cbd86b72a
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() {
<<<<<<< HEAD
  
    MIDImessage.id = 6; 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      
        mpu.dmpGetQuaternion(&q, fifoBuffer);
=======
    message.id = 5; 
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) { 
      
        mpu.dmpGetQuaternion(&q, fifo_buffer);
>>>>>>> 0e86ae4718a74856566d12b21395ad2cbd86b72a
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetGravity(&gravity, &q);
<<<<<<< HEAD
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
    media =  media/100;
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
=======
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); 
    
        message.gyro = ypr[2] * 180/M_PI;
        message.accel = aaReal.x;
        message.touch = 1 ? touchRead(T3) < 30 : 0;
        esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
   }
  delay(10);
}

>>>>>>> 0e86ae4718a74856566d12b21395ad2cbd86b72a
