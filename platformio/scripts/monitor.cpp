#include <WiFi.h>
#include "esp_wifi.h"

const int CANAL = 1;

volatile uint32_t pacotes = 0;
volatile uint32_t pacotesForte = 0;

volatile int32_t somaRSSI = 0;
volatile int8_t maiorRSSI = -127;
volatile int8_t menorRSSI = 0;

void sniffer(void* buf, wifi_promiscuous_pkt_type_t type) {

    if (type != WIFI_PKT_MGMT &&
        type != WIFI_PKT_DATA &&
        type != WIFI_PKT_CTRL) {
        return;
    }

    wifi_promiscuous_pkt_t* pkt =
        (wifi_promiscuous_pkt_t*)buf;

    int8_t rssi = pkt->rx_ctrl.rssi;

    pacotes++;
    somaRSSI += rssi;

    if (rssi > maiorRSSI)
        maiorRSSI = rssi;

    if (menorRSSI == 0 || rssi < menorRSSI)
        menorRSSI = rssi;

    // Considera sinal forte acima de -60 dBm
    if (rssi > -60)
        pacotesForte++;
}

void setup() {

    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(false);

    esp_wifi_set_channel(
        CANAL,
        WIFI_SECOND_CHAN_NONE
    );

    esp_wifi_set_promiscuous_rx_cb(&sniffer);
    esp_wifi_set_promiscuous(true);

    Serial.println();
    Serial.println("============================");
    Serial.println(" MONITOR PASSIVO 2.4 GHz");
    Serial.println("============================");
    Serial.print("Canal monitorado: ");
    Serial.println(CANAL);
    Serial.println();
}

void loop() {

    static uint32_t ultimo = 0;

    if (millis() - ultimo >= 1000) {

        ultimo = millis();

        noInterrupts();

        uint32_t p = pacotes;
        uint32_t fortes = pacotesForte;
        int32_t soma = somaRSSI;
        int8_t maior = maiorRSSI;
        int8_t menor = menorRSSI;

        pacotes = 0;
        pacotesForte = 0;
        somaRSSI = 0;
        maiorRSSI = -127;
        menorRSSI = 0;

        interrupts();

        Serial.println("----------------------------");

        Serial.print("Pacotes/s: ");
        Serial.println(p);

        if (p > 0) {

            Serial.print("RSSI medio: ");
            Serial.print((float)soma / p);
            Serial.println(" dBm");

            Serial.print("Sinal mais forte: ");
            Serial.print(maior);
            Serial.println(" dBm");

            Serial.print("Sinal mais fraco: ");
            Serial.print(menor);
            Serial.println(" dBm");

            Serial.print("Pacotes > -60 dBm: ");
            Serial.println(fortes);
        }

        Serial.println();
    }
}