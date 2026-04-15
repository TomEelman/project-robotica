#ifndef KALMANFILTER_H
#define KALMANFILTER_H

class KalmanFilter {
public:
    float x;
    float P;
    float Q;
    float R;

private:
    bool initialized = false;

public:
    KalmanFilter() : Q(50.0f), R(36.0f), x(0.0f), P(1.0f) {}
    
    KalmanFilter(float q, float r) 
        : Q(q), R(r), x(0.0f), P(1.0f) {}

    float Update(float meting) {
        if (!initialized) {
            x = meting;
            initialized = true;
            return x;
        }

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