#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
private:
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];

public:
    KalmanFilter(float q_angle = 0.001, float q_bias = 0.003, float r_measure = 0.03);
    float update(float new_angle, float new_rate, float dt);
};

#endif