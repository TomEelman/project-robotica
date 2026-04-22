#ifndef LOCALISATION_H
#define LOCALISATION_H

#include "SensorHub.h"

class Localisation {
public:
    Localisation(float wheelBase);

    void Predict(float vLeft, float vRight, float dt);
    void UpdateIMU(float imuYaw, float dt);

    float GetX() const;
    float GetY() const;
    float GetTheta() const;

private:
    // state
    float x;
    float y;
    float theta;

    // covariance (simplified 3x3)
    float P[3][3];

    // noise
    float Q[3][3];
    float R;

    float wheelBase;
};

#endif