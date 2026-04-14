#include <WiFi.h>
#include <Wire.h>
#include "SPIFFS.h"

unsigned long startTime;
const unsigned long testDuration = 180000; // 3 minutos (em ms)

File logFile;

void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin(true)) {
    Serial.println("Erro ao montar SPIFFS");
    return;
  }

  // Cria/abre arquivo de log
  logFile = SPIFFS.open("/log.txt", FILE_WRITE);
  if (!logFile) {
    Serial.println("Erro ao abrir arquivo");
    return;
  }

  Serial.println("START");
  logFile.println("START");

  startTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - startTime <= testDuration) {
    String msg = "OK - tempo(ms): " + String(currentTime);

    Serial.println(msg);
    logFile.println(msg);

    delay(1000);
  } else {
    Serial.println("Teste finalizado!");
    logFile.println("Teste finalizado!");

    logFile.close(); // importante fechar o arquivo
    while (true); // para execução
  }
}