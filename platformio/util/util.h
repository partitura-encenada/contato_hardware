#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Função para printar endereço MAC
namespace Util{
  void PrintMACAddr(){
    if(Serial){
      uint8_t baseMac[6];
        esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
        if (ret == ESP_OK) {
          Serial.printf("\nMAC: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
                        baseMac[0], baseMac[1], baseMac[2],
                        baseMac[3], baseMac[4], baseMac[5]);
        } else {
          Serial.println("Failed to read MAC address");
        }
    }
  }
}  
