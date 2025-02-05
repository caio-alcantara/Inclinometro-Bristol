#include "DataProcessor.h"
#include <Arduino.h>
#include <math.h>

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
    if ((inclination < 15 && inclination > 0) || inclination > 85) {
        inclination -= 5; // Aqui, o -5 foi utilizado como valor descoberto empiricamente
                          // Basta colocar o sensor em posição de 90 graus e ver o quão distante a medição está de 90
                          // Neste caso, a 90 graus o sensor media 95. Dessa forma, subtraímos 5
    }

    if (inclination < 0) { // Caso a inclinação seja negativa, ajusta para 0
        inclination = 0;   // Isso pode ocorrer devido a pequenos erros de medição
    }
    
    inclination = truncate(inclination, 2); // Truncate arredonda para 2 casas decimais
    return (inclination > 95.0f) ? inclination - 90.0f : inclination; 
    // Se a inclinação for maior que 95°, subtrai 90° para normalizar o valor.
    // Ou seja, o valor tende sempre a ser entre 0 e 90, independentemente da direção
}

// Arredondar valores a n casas decimais (número de casas decimais)
float DataProcessor::truncate(float value, int decimal_places) {
    const float factor = pow(10, decimal_places);
    return (value > 0.0f) ? floor(value * factor) / factor : ceil(value * factor) / factor;
}