#include <esp_now.h>
#include <WiFi.h>
#include <random>

typedef struct { // Struct da mensagem, deve ser igual ao da base 
  int16_t yaw = 0;
  int16_t pitch = 0;
  int16_t roll = 0;
  int16_t accel = 0;
  int8_t touch = 0;
} message_t;
message_t message;
 
void setup() {
    Serial.begin(115200); // Inicializa conexão Serial
    message_t message;
}
 
void loop() { 
    if ((0 + (rand() % static_cast<int>(10 - 0 + 1))) == 1){
        message.roll++;
    }
    if ((0 + (rand() % static_cast<int>(10 - 0 + 1))) == 1){ 
        message.touch = 1;
    }
    else {
        message.touch = 1;
    }
    message.roll = message.roll % 180;
    Serial.println("99/" +
            String(message.roll)+'/'+
            String(message.accel)+'/'+
            String(message.touch));
    delay(10);
}


