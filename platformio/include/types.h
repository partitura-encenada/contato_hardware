#pragma once

#include <Arduino.h>
#include "config.h"
#include <cstdint>

// Enviado via notify em STATUS_CHAR a cada STATUS_INTERVAL_MS.
struct __attribute__((packed)) StatusPacket {
    uint8_t state;    // 0 = normal, 1 = calibrando
    uint8_t touch;    // Sensor de toque: 1 = ativo, 0 = liberado
    int16_t gyro_x;   // Ângulo de rolagem em graus (limitado a ±GYRO_MAX_DEG)
    int16_t accel_x;  // Aceleração linear no eixo X (unidade bruta IMU ÷ 10)
    int16_t tilt;     // Inclinação do antebraço em graus (pitch, limitado a ±TILT_MAX_DEG)
};

// Registradores de offset do MPU6050 armazenados na NVS para calibração persistente.
struct MPUOffsets {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX,  gyroY,  gyroZ;
};

struct RuntimeState {
    bool dmpReady = false;
    bool touchPressed = false;
    bool notePlaying = false;
    bool accelPlaying = false;
    bool flipGyro = DEFAULT_DIR != 0;
    bool tiltEnabled = DEFAULT_TILT_ENABLED != 0;
    bool legatoEnabled = DEFAULT_LEGATO_ENABLED != 0;
    bool calibrationRequested = false;
    uint8_t status = STATUS_STATE_IDLE;
    int32_t accelThreshold = DEFAULT_ACCEL_THRESHOLD;
    unsigned long lastSent = 0;
    unsigned long lastAccel = 0;
    byte lastNote = DEFAULT_NOTE;
    int lastPitchBend = PITCH_BEND_CENTER;
};
