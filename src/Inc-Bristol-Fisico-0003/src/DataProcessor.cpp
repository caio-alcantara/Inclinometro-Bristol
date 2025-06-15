#include "DataProcessor.h"
#include <Arduino.h>
#include <math.h>
#include "Config.h"

// *********************** DataProcessor *********************** // 
// Responsável por processar os dados brutos do sensor e realizar cálculos úteis,
// como a conversão de aceleração em ângulos de inclinação e o cálculo da inclinação total

// Calcula o ângulo de inclinação frontal (pitch) com base nas acelerações medidas pelo acelerômetro
float DataProcessor::calculateAccelerometerPitch(float ax, float ay, float az) {
    return atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI; // Multiplica por 180.0f / PI para converter o resultado de radianos para graus
}

// Calcula o ângulo de inclinação lateral (roll) com base nas acelerações medidas pelo acelerômetro
float DataProcessor::calculateAccelerometerRoll(float ay, float az) {
    return atan2(ay, az) * 180.0f / PI; // Multiplica por 180.0f / PI para converter o resultado de radianos para graus
}

float DataProcessor::calculateTotalInclination(float pitch, float roll) {
    float inclination = sqrt(pitch * pitch + roll * roll);
    
    // Determinar o sinal baseado no maior componente absoluto
    float sign = 1.0f;
    if (abs(pitch) > abs(roll)) {
        sign = (pitch < 0) ? -1.0f : 1.0f;
    } else {
        sign = (roll < 0) ? -1.0f : 1.0f;
    }
    
    // Aplicar o sinal à inclinação
    inclination *= sign;
    
    // Aplicar offsets
    inclination += Config::positive_offset_angle;
    
    if (inclination <= 0 || abs(inclination) < 10) {
        inclination -= Config::negative_offset_angle;
    }
    
    // Limitar o valor entre -90 e +90
    if (inclination > 90) {
        inclination = 90;
    } else if (inclination < -90) {
        inclination = -90;
    }
    
    inclination = truncate(inclination, 2);
    return inclination;
}

// Arredondar valores a n casas decimais (número de casas decimais)
float DataProcessor::truncate(float value, int decimal_places) {
    const float factor = pow(10, decimal_places);
    return (value > 0.0f) ? floor(value * factor) / factor : ceil(value * factor) / factor;
}