#pragma once

#include <cstdint>

// Notified via STATUS_CHAR every SENSOR_INTERVAL_MS.
struct __attribute__((packed)) StatusPacket {
    int16_t gyro_x;   // roll angle in degrees, clamped to ±GYRO_MAX_DEG
    int16_t accel_x;  // linear X-axis acceleration (raw IMU ÷ 10)
    uint8_t touch;    // 1 = touched, 0 = released
};

// MPU6050 offset registers, stored in NVS for persistent calibration.
struct MPUOffsets {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX,  gyroY,  gyroZ;
};
