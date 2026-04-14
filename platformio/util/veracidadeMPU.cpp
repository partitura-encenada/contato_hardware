#include <Arduino.h>
#include <Wire.h>

void setup() {
  Wire.begin(21, 22);   // SDA, SCL do ESP32 DevKit
  Serial.begin(115200);
  delay(1000);

  Serial.println("I2C Scanner iniciado");
}

void loop() {
  byte error, address;
  int devicesFound = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado em 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    }
  }

  if (devicesFound == 0)
    Serial.println("Nenhum dispositivo encontrado");

  delay(3000);
}
