#pragma once

#include <cstdint>

// Enviado via notify em STATUS_CHAR a cada STATUS_INTERVAL_MS.
struct __attribute__((packed)) StatusPacket {
    int16_t gyro_x;   // Ângulo de rolagem em graus (limitado a ±GYRO_MAX_DEG)
    int16_t accel_x;  // Aceleração linear no eixo X (unidade bruta IMU ÷ 10)
    uint8_t touch;    // Sensor de toque: 1 = ativo, 0 = liberado
};

// Registradores de offset do MPU6050 armazenados na NVS para calibração persistente.
struct MPUOffsets {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX,  gyroY,  gyroZ;
};
