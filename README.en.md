# Contato (hardware)
[![pt-br](https://img.shields.io/badge/lang-pt--br-green.svg)](https://github.com/partitura-encenada/contato_embarcado/blob/main/README.md)
Embedded code for the "Contato" device being developed by the Universidade Federal do Rio de Janeiro in partnership with the UFRJ Technological Park. The device currently is based in a ESP32 module (today being utilized the DEVKIT model), in addition with the [Invensense's MPU6050 IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) for capturing gyroscope and accelerometer data in 6 Degrees of Freedom.

# 📁 Organização 
contato_hardware/

├── arduino # arduino project

└── platformio # platform.io project +

    ├── scripts # upload sketches
    
      ├── ble_equip.cpp # BLE connection

      ├── com_port_base.cpp # ESP_NOW P2P based connection (Base comunicating with computer via Serial)

      ├── com_port_equip.cpp #  ESP_NOW P2P based connection (Device comunicating with "base")
      
      └── teste.cpp # for testing and debugging devices
      
    └── util # utilities 
    
