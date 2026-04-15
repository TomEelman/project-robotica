#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    float x;  // schatting
    float P;  // onzekerheid

    float Q;  // procesruis
    float R;  // meetsruis

    KalmanFilter(float q, float r, float initial = 0.0f) {
        Q = q;
        R = r;
        x = initial;
        P = 1.0f;
    }

    float Update(float meting) {
        // Predict
        P = P + Q;

        // Update
        float K = P / (P + R);
        x = x + K * (meting - x);
        P = (1 - K) * P;

        return x;
    }
};
#endif