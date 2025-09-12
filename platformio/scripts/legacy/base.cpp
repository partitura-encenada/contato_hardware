/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    int gyro;
    int accel;
    int touch;
} struct_message;

// Create a struct_message called myData
struct_message MIDImessage;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  
  char macStr[18];
  memcpy(&MIDImessage, incomingData, sizeof(MIDImessage));
  Serial.println(String(MIDImessage.id)+'/'+String(MIDImessage.gyro)+'/'+String(MIDImessage.accel)+'/'+String(MIDImessage.touch));
 
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station and set fixed channel
  WiFi.mode(WIFI_STA);
  WiFi.channel(5);  // Para canal fixo 
  
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
}