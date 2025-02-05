#pragma once
#include <MPU9250.h>
#include <Wire.h>

class SensorManager {
public:
    struct SensorData {
        struct { float x, y, z; } accel, gyro, mag;
        float yaw, pitch, roll;
    };

    SensorManager(MPU9250& sensor, MPU9250Setting& settings, TwoWire& wire);
    bool initialize();
    void calibrateAccelGyro();
    bool update();
    SensorData getData() const;

private:
    MPU9250& mpu;
    MPU9250Setting& settings;
    TwoWire& wire;
};