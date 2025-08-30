# Contato (hardware)
[![en](https://img.shields.io/badge/lang-en-red.svg)](https://github.com/partitura-encenada/contato_embarcado/blob/main/README.en.md)
Código embarcado para o dispositivo Contato sendo desenvolvido pelo curso de Dança da Universidade Federal do Rio de Janeiro em parceria com o Parque Tecnológico. Atualmente o dispositivo se baseia em um módulo ESP32, hoje sendo utilizado um modelo DEVKIT, em conjunto com o [IMU MPU6050 da Invensense] ( https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/ ) para captar dados de giroscópio e acelerômetro em 6 Graus de Liberdade.

# 📁 Organização 
contato_hardware/

├── arduino # projeto arduino

└── platformio # projeto platform.io +

    ├── scripts # rascunhos para upload
    
      ├── ble_equip.cpp # conexão BLE

      ├── esp_now_base.cpp # conexão ESP_NOW P2P (Peer comunicando com a máquina via serial)

      ├── esp_now_equip.cpp # conexão ESP_NOW P2P (Peer comunicando com a "base")
      
      └── teste.cpp # para testagem e debugging dos aparelhos
      
    └── util # utilidades
    
