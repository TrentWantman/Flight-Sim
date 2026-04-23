#ifndef KALMANFILTER1D_H
#define KALMANFILTER1D_H
#include <cmath>

class KalmanFilter1D {
private:
    float x[2]; // state estimate x[0] = altitude, x[1] = vertical velocity
    float P[2][2]; // estimate error covariance
    float Q[2][2]; // process noise covariance
    float R; // measurement noise covariance

public:
    KalmanFilter1D(float processNoise = 0.1f, float measurementNoise = 1.0f) {
        x[0] = 0.0f;
        x[1] = 0.0f;
        P[0][0] = 500.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f;   P[1][1] = 500.0f;
        Q[0][0] = processNoise; Q[0][1] = 0.0f;
        Q[1][0] = 0.0f;         Q[1][1] = processNoise;
        R = measurementNoise;
    }

    float GetAltitude() const { return x[0]; }
    float GetVelocity() const { return x[1]; }

    void Predict(float accel, float dt){
        x[0] += (x[1]* dt);
        x[1] += (accel * dt);

        P[0][0] = P[0][0] + dt * P[1][0] + dt * P[0][1] + dt * dt * P[1][1] + Q[0][0];
        P[1][1] = P[1][1] + Q[1][1];
        P[0][1] = P[0][1] + dt * (P[1][1] - Q[1][1]) + Q[0][1];
        P[1][0] = P[1][0] + dt * (P[1][1] - Q[1][1]) + Q[1][0];
    }

    void Update(float gpsAltitude) {
        // Innovation
        float y = gpsAltitude - x[0];

        // Innovation covariance
        float S = P[0][0] + R;

        // Kalman gain
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Correct state estimate
        x[0] += (K[0] * y);
        x[1] += (K[1] * y);

        // Correct covariance
        float p00 = P[0][0];
        float p01 = P[0][1];
        
        P[0][0] = (1 - K[0]) * p00;
        P[0][1] = (1 - K[0]) * p01;
        P[1][0] = P[1][0] - K[1] * p00;
        P[1][1] = P[1][1] - K[1] * p01;
    }
        
};
#endif // KALMANFILTER1D_H