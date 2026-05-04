#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  // MAC eFuse
  uint64_t chipid = ESP.getEfuseMac();
  Serial.printf("MAC eFuse: %04X%08X\n", (uint16_t)(chipid >> 32), (uint32_t)chipid);

  // MAC WiFi STA
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("MAC STA: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
}

void loop() {
}