#include "kalman.h"

void Kalman_Init(KalmanFilter *kf) {
    // Initializing state covariance and process noise variables
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f;
    
    kf->angle = 0.0f;
    kf->bias = 0.0f;
    
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

float Kalman_GetAngle(KalmanFilter *kf, float newAngle, float newRate, float dt) {
    // Step 1: Predict current state (Prior)
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;
    
    // Step 2: Predict error covariance
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    // Step 3: Calculate Kalman Gain
    float S = kf->P[0][0] + kf->R_measure; // Estimate error
    float K[2]; // Kalman Gain array
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    // Step 4: Update state with measurement (Posterior)
    float y = newAngle - kf->angle; // Angle difference (Innovation)
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    
    // Step 5: Update error covariance
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}
