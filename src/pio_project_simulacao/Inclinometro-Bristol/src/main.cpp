#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "KalmanFilter.h"
#include "MpuUtils.h"

Adafruit_MPU6050 mpu; // Definição global do objeto
KalmanFilter kalmanX, kalmanY;
unsigned long prevTime = 0;

void setup() {
    Serial.begin(115200);
    if (!mpu.begin()) {
        Serial.println("Falha ao encontrar sensor");
        while (1);
    }
    setAndDisplayAccelRange(16);
    setAndDisplayGyroRange(500);
    setAndDisplayFilterBandwidth(21);
    prevTime = millis();
    kalmanX = KalmanFilter(0.005, 0.005, 0.01);
    kalmanY = KalmanFilter(0.005, 0.005, 0.01);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    float roll_acc, pitch_acc;
    calculateAccelAngles(pitch_acc, roll_acc); // Usando a função separada

    float gyroX_deg = g.gyro.x * (180 / PI);
    float gyroY_deg = g.gyro.y * (180 / PI);

    float roll = kalmanX.update(roll_acc, gyroX_deg, dt);
    float pitch = kalmanY.update(pitch_acc, gyroY_deg, dt);

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("° | Pitch: ");
    Serial.print(pitch);
    Serial.println("°");
}