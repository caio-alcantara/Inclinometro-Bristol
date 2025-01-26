#include "MpuUtils.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

void setAndDisplayAccelRange(int range) {
  if (range != 2 && range != 4 && range != 8 && range != 16) {
    Serial.println("Erro: Range deve ser 2, 4, 8 ou 16 g");
    return;
  }

  mpu6050_accel_range_t rangeSetting; 
  
  switch(range) {
    case 2:  rangeSetting = MPU6050_RANGE_2_G;  break;
    case 4:  rangeSetting = MPU6050_RANGE_4_G;  break;
    case 8:  rangeSetting = MPU6050_RANGE_8_G;  break;
    case 16: rangeSetting = MPU6050_RANGE_16_G; break;
    default: return;
  }

  mpu.setAccelerometerRange(rangeSetting);
  
  // Converter o valor de retorno para g
  Serial.print("Intervalo do acelerômetro definido para: ");
  switch(mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:  Serial.print(2);  break;
    case MPU6050_RANGE_4_G:  Serial.print(4);  break;
    case MPU6050_RANGE_8_G:  Serial.print(8);  break;
    case MPU6050_RANGE_16_G: Serial.print(16); break;
  }
  Serial.println("g");
}

// Configuração do Range do Giroscópio
void setAndDisplayGyroRange(int range) {
  const int validRanges[] = {250, 500, 1000, 2000};
  bool valid = false;
  
  for(int i = 0; i < 4; i++) {
    if(range == validRanges[i]) {
      valid = true;
      break;
    }
  }
  
  if(!valid) {
    Serial.println("Erro: Aceita apenas 250, 500, 1000 ou 2000 °/s");
    return;
  }

  mpu6050_gyro_range_t rangeSetting;
  switch(range) {
    case 250:  rangeSetting = MPU6050_RANGE_250_DEG;  break;
    case 500:  rangeSetting = MPU6050_RANGE_500_DEG;  break;
    case 1000: rangeSetting = MPU6050_RANGE_1000_DEG; break;
    case 2000: rangeSetting = MPU6050_RANGE_2000_DEG; break;
  }
  mpu.setGyroRange(rangeSetting);

  Serial.print("Intervalo do giroscópio definido para: ±");
  Serial.print(range);
  Serial.println("°/s");
}

// Configuração do Filtro Passa-Baixa
void setAndDisplayFilterBandwidth(int bandwidth) {
  const int validBW[] = {5, 10, 21, 44, 94, 184, 260};
  bool valid = false;
  
  for(int i = 0; i < 7; i++) {
    if(bandwidth == validBW[i]) {
      valid = true;
      break;
    }
  }
  
  if(!valid) {
    Serial.println("Erro: BW deve ser 5,10,21,44,94,184 ou 260 Hz");
    return;
  }

  mpu6050_bandwidth_t bwSetting;
  switch(bandwidth) {
    case 5:   bwSetting = MPU6050_BAND_5_HZ;   break;
    case 10:  bwSetting = MPU6050_BAND_10_HZ;  break;
    case 21:  bwSetting = MPU6050_BAND_21_HZ;  break;
    case 44:  bwSetting = MPU6050_BAND_44_HZ;  break;
    case 94:  bwSetting = MPU6050_BAND_94_HZ;  break;
    case 184: bwSetting = MPU6050_BAND_184_HZ; break;
    case 260: bwSetting = MPU6050_BAND_260_HZ; break;
  }
  mpu.setFilterBandwidth(bwSetting);

  Serial.print("Frequência do filtro passa-baixa definido para: ");
  Serial.print(bandwidth);
  Serial.println("Hz");
}

void calculateAccelAngles(float &pitch_acc, float &roll_acc) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Calcula pitch (rotação no eixo Y) usando arco-tangente
  pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // Calcula roll (rotação no eixo X) usando arco-tangente
  roll_acc = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
}