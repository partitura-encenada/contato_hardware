#pragma once

#include <cstdint>

// Notified via STATUS_CHAR every SENSOR_INTERVAL_MS.
// Layout (6 bytes, no padding): state, touch, gyro_x, accel_x
struct StatusPacket {
    uint8_t  state;    // 0 = normal, 1 = calibrating
    uint8_t  touch;    // 1 = touched, 0 = released
    int16_t  gyro_x;   // roll angle in degrees, clamped to ±GYRO_MAX_DEG
    int16_t  accel_x;  // linear X-axis acceleration (raw IMU ÷ 10)
};

// MPU6050 offset registers, stored in NVS for persistent calibration.
struct MPUOffsets {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX,  gyroY,  gyroZ;
};
