#pragma once

class KalmanFilter {
public:
    KalmanFilter(float q_angle, float q_bias, float r_measure);
    void initialize(float initial_angle);
    float update(float measurement, float rate, float delta_time);

private:
    void predict(float rate, float delta_time);
    void calculateKalmanGain();
    void correct(float measurement);
    void updateCovarianceMatrix();

    const float Q_angle, Q_bias, R_measure;
    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    float K[2] = {0.0f, 0.0f};
};