#pragma once
#include <Arduino.h>

namespace Config {
    constexpr uint8_t MPU_I2C_ADDRESS = 0x68;
    constexpr uint8_t LED_PIN = 2;
    constexpr unsigned long SERIAL_BAUD_RATE = 115200;
    
    namespace Kalman {
        constexpr float PITCH_Q_ANGLE = 0.001f;
        constexpr float PITCH_Q_BIAS = 0.003f;
        constexpr float PITCH_R_MEASURE = 0.03f;
        
        constexpr float ROLL_Q_ANGLE = 0.001f;
        constexpr float ROLL_Q_BIAS = 0.003f;
        constexpr float ROLL_R_MEASURE = 0.03f;
    }
}