#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float q_angle, float q_bias, float r_measure) {
    Q_angle = q_angle;
    Q_bias = q_bias;
    R_measure = r_measure;
    angle = 0;
    bias = 0;
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
}

float KalmanFilter::update(float new_angle, float new_rate, float dt) {
    // Predição 
    angle += dt * (new_rate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Atualização 
    float y = new_angle - angle;
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Correção
    angle += K[0] * y;
    bias += K[1] * y;

    // Atualização da covariância
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}