# Contato (hardware)
[![en](https://img.shields.io/badge/lang-en-red.svg)](README.en.md)

Código embarcado para o dispositivo **Contato**, desenvolvido pelo curso de Dança da Universidade Federal do Rio de Janeiro em parceria com o Parque Tecnológico UFRJ.

O dispositivo é baseado em um módulo **ESP32 DEVKIT V1** com um **IMU MPU6050** (giroscópio + acelerômetro em 6 graus de liberdade). Ele captura o movimento do usuário e transmite mensagens **MIDI via Bluetooth Low Energy (BLE)**, permitindo interação musical a partir do movimento corporal e do toque capacitivo.

## Como funciona

- O ângulo de rolagem do giroscópio (±90°) determina qual nota MIDI está selecionada dentre as seções configuráveis.
- O toque no sensor capacitivo dispara Note-On/Note-Off da nota selecionada. Manter o toque enquanto muda de seção desliza para a nova nota.
- Picos de aceleração no eixo X disparam uma nota de percussão curta (MIDI 36, canal 8).
- O ângulo de inclinação do antebraço (pronação/supinação) envia Pitch Bend, com zona morta de ±10°.
- Modo Legato opcional: a nota sustenta após soltar o toque, encerrando apenas quando um novo toque ou percussão for acionado.
- Toda a configuração (notas, sensibilidade, direção, pitch bend, legato, calibração) é persistida na memória não-volátil do ESP32 (NVS) e pode ser alterada pelo cliente BLE.

## Organização

```
contato_hardware/
├── arduino/          # implementações legadas (ESP-NOW P2P, sem manutenção ativa)
└── platformio/       # projeto ativo
    ├── platformio.ini
    ├── src/
    │   └── main.cpp  # firmware principal
    └── include/
        ├── config.h  # pinos, UUIDs BLE, constantes MIDI e de tempo
        └── types.h   # structs de dados (StatusPacket, MPUOffsets)
```

## Build e upload

Requer o [PlatformIO](https://platformio.org/) (CLI ou extensão VSCode).

```bash
cd platformio

# Compilar
platformio run -e esp32doit-devkit-v1

# Compilar e enviar para o dispositivo
platformio run --target upload -e esp32doit-devkit-v1

# Monitor serial (115200 baud)
platformio device monitor --speed 115200
```

## Hardware

| Componente | Descrição |
|---|---|
| ESP32 DEVKIT V1 | Microcontrolador principal |
| MPU6050 | IMU 6 eixos (I2C, 400 kHz) — SDA=21, SCL=22 |
| GPIO 2 | LED indicador (aceso = cliente conectado ou calibrando) |
| GPIO T3 | Sensor de toque capacitivo |

## Calibração

Na primeira inicialização sem offsets salvos, o dispositivo calibra automaticamente o MPU6050 e salva os offsets na NVS. Recalibrações posteriores podem ser disparadas pelo botão "Calibrar" na interface do cliente BLE (`contato_gui`).
