#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

const int callibration_time = 6;

void setup() {
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200);
    delay(1000);

    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("ERRO MPU6050");
        return;
    }

    uint8_t dev_status = mpu.dmpInitialize();

    if (dev_status != 0) {
        Serial.print("ERRO DMP: ");
        Serial.println(dev_status);
        return;
    }

    mpu.CalibrateAccel(callibration_time);
    mpu.CalibrateGyro(callibration_time);

    Serial.println();

    Serial.print("mpu.setXAccelOffset(");
    Serial.print(mpu.getXAccelOffset());
    Serial.println(");");

    Serial.print("mpu.setYAccelOffset(");
    Serial.print(mpu.getYAccelOffset());
    Serial.println(");");

    Serial.print("mpu.setZAccelOffset(");
    Serial.print(mpu.getZAccelOffset());
    Serial.println(");");

    Serial.print("mpu.setXGyroOffset(");
    Serial.print(mpu.getXGyroOffset());
    Serial.println(");");

    Serial.print("mpu.setYGyroOffset(");
    Serial.print(mpu.getYGyroOffset());
    Serial.println(");");

    Serial.print("mpu.setZGyroOffset(");
    Serial.print(mpu.getZGyroOffset());
    Serial.println(");");
}

void loop() {
}