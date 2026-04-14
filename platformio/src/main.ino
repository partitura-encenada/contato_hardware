#include <WiFi.h>                       
#include "Wire.h"                      

void setup() {
    Serial.begin(115200);
    Serial.println("BOOT");
}

void loop() {
    Serial.println("ok");
    delay(100);
}
