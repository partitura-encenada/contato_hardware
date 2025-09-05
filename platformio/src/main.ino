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
#include "../util/util.h"

typedef struct struct_message {
    int id; // Deve ser único para cada remetente
    int yaw;
    int pitch;
    int roll;
    int accel_x;
    int accel_y;
    int accel_z;
    int touch;
} struct_message;

struct_message message;

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&message, incomingData, sizeof(message));
  Serial.println(String(message.id)+'/'+
                String(message.yaw)+'/'+
                String(message.pitch)+'/'+
                String(message.roll)+'/'+
                String(message.accel_x)+'/'+
                String(message.accel_y)+'/'+
                String(message.accel_z)+'/'+
                String(message.touch));
}
 
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro inicializando ESPNOW");
    return;
  }
  Util::PrintMACAddr();
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
}
