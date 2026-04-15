#include "Localisation.h"
#include <cmath>
#include <stdio.h>
Localisation::Localisation(float wheelBase, float dt)
    : x(0.0f), y(0.0f), theta(0.0f),
      v(0.0f), omega(0.0f),
      wheelBase(wheelBase*1000)
{
}

void Localisation::Update(float vLeft, float vRight, float imuYawRate, float dt)
{
    // 1. Bereken snelheid en rotatie uit wielen
    float v_wheels = (vLeft + vRight) * 0.5f;
    float omega_wheels = (vRight - vLeft) / wheelBase;

    // 2. Combineer met IMU (simpel: gyro gebruiken)
    omega = 0.7f * imuYawRate + 0.3f * omega_wheels;
    v = v_wheels;
    // 3. Integratie (dead reckoning)
    theta += omega * dt;
    printf("dt = %.8f\n", dt);
    float dx = v * cos(theta) * dt;
    float dy = v * sin(theta) * dt;

    x += dx;
    y += dy;
}

float Localisation::GetX() const { return x; }
float Localisation::GetY() const { return y; }
float Localisation::GetTheta() const { return theta; }