#pragma once

#include <cstdint>

// Sent via BLE STATUS_CHAR notify every STATUS_INTERVAL_MS.
struct __attribute__((packed)) StatusPacket {
    int16_t gyro_x;   // Roll angle in degrees (clamped to ±GYRO_MAX_DEG)
    int16_t accel_x;  // Linear X acceleration (raw IMU ÷ 10)
    uint8_t touch;    // Touch sensor: 1 = active, 0 = released
};

// MPU6050 offset registers stored in NVS for persistent calibration.
struct MPUOffsets {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX,  gyroY,  gyroZ;
};
