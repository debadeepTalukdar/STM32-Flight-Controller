#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance
    
    float angle;     // The angle calculated by the Kalman filter
    float bias;      // The gyro bias calculated by the Kalman filter
    float rate;      // Unbiased rate calculated from the rate and the calculated bias
    
    float P[2][2];   // Error covariance matrix
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf);
float Kalman_GetAngle(KalmanFilter *kf, float newAngle, float newRate, float dt);

#endif
