#include "KalmanFilter.h"

// *********************** KalmanFilter ***********************
// O Filtro de Kalman é um algoritmo matemático usado para estimar o estado de um sistema a partir de medições ruidosas
// Aqui, o Filtro de Kalman é utilizado para realizar a fusão entre os dados do acelerômetro e do giroscópio a fim de obter o ângulo 
// total do dispositivo. O filtro nos ajuda a diminuir ruidos e erros provenientes de vibrações e movimentos bruscos.


// Construtor
KalmanFilter::KalmanFilter(float q_angle, float q_bias, float r_measure)
    : Q_angle(q_angle), Q_bias(q_bias), R_measure(r_measure) {}

// Inicializa o filtro de Kalman e seu estado inicial
void KalmanFilter::initialize(float initial_angle) {
    angle = initial_angle;
    bias = 0.0f;
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;
}

// Atualiza o filtro de Kalman com a medição do sensor,
// mantendo uma malha fechada com feedback vindos dos sensores e possibilitando um cálculo sempre atualizado
float KalmanFilter::update(float measurement, float rate, float delta_time) {
    /* As etapas seguem como:
        1. Predição: Atualiza o estado e a matriz de covariância com base no giroscópio
        2. Cálculo do Ganho de Kalman: Determina o peso da medição e da previsão
        3. Correção: Ajusta o estado com base na medição
        4. Atualização da Matriz de Covariância: Atualiza a incerteza da estimativa
    */ 
    predict(rate, delta_time);
    calculateKalmanGain();
    correct(measurement);
    updateCovarianceMatrix();
    return angle;
}

// Predição do filtro de Kalman
void KalmanFilter::predict(float rate, float delta_time) {
    // Atualiza o estado e a matriz de covariância com base no giroscópio
    angle += delta_time * (rate - bias); // Angulo integra a taxa angular do giroscópio, já corrigida pelo bias

    // Matriz de covariância atualiza a incerteza da estimativa
    P[0][0] += delta_time * (delta_time * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= delta_time * P[1][1];
    P[1][0] -= delta_time * P[1][1];
    P[1][1] += Q_bias * delta_time;
}

void KalmanFilter::calculateKalmanGain() {
    const float S = P[0][0] + R_measure;
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
}

void KalmanFilter::correct(float measurement) {
    const float innovation = measurement - angle;
    angle += K[0] * innovation;
    bias += K[1] * innovation;
}

void KalmanFilter::updateCovarianceMatrix() {
    const float P00_temp = P[0][0];
    const float P01_temp = P[0][1];
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}