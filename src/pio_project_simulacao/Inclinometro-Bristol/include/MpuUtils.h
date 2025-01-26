#ifndef MPU6050_UTILS_H
#define MPU6050_UTILS_H

#include <Adafruit_MPU6050.h>

extern Adafruit_MPU6050 mpu; // Declaração externa do objeto global

void setAndDisplayAccelRange(int range);
void setAndDisplayGyroRange(int range);
void setAndDisplayFilterBandwidth(int bandwidth);
void calculateAccelAngles(float &pitch_acc, float &roll_acc);

#endif