#ifndef LOCALISATION_H
#define LOCALISATION_H

#include "SensorHub.h"

class Localisation {

private:

    // Pose
    float x;
    float y;
    float theta;

    // Velocity (optioneel handig)
    float v;
    float omega;

    // Robot parameters
    float wheelBase;   // afstand tussen wielen

public:

    Localisation(float wheelBase, float dt);

    void Update(float vLeft, float vRight, float imuYawRate, float dt);

    float GetX() const;
    float GetY() const;
    float GetTheta() const;
};

#endif