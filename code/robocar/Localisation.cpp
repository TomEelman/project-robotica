#include "Localisation.h"
#include <cmath>
#include <stdio.h>
#include "Localisation.h"

Localisation::Localisation(float wheelBase)
    : x(0.0f), y(0.0f), theta(0.0f), wheelBase(wheelBase)
{
    // init covariance
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = 0.0f;
            Q[i][j] = 0.0f;
        }
    }

    P[0][0] = 0.01f;
    P[1][1] = 0.01f;
    P[2][2] = 0.01f;

    // process noise (tune deze!)
    Q[0][0] = 0.02f;
    Q[1][1] = 0.02f;
    Q[2][2] = 0.01f;

    // IMU noise
    R = 0.01f;
}

void Localisation::Predict(float vLeft, float vRight, float dt)
{
    float v = 0.5f * (vLeft + vRight);
    float omega = (vRight - vLeft) / wheelBase;

    float c = cos(theta);
    float s = sin(theta);

    // state prediction
    x += v * c * dt;
    y += v * s * dt;
    theta += omega * dt;

    // normalize angle
    if (theta > M_PI) theta -= 2.0f * M_PI;
    if (theta < -M_PI) theta += 2.0f * M_PI;

    // Jacobian F
    float F[3][3] = {
        {1, 0, -v * s * dt},
        {0, 1,  v * c * dt},
        {0, 0, 1}
    };

    // P = FPF^T + Q (simplified expansion)
    float Pnew[3][3] = {0};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3; l++) {
                    Pnew[i][j] += F[i][k] * P[k][l] * F[j][l];
                }
            }
            Pnew[i][j] += Q[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = Pnew[i][j];
        }
    }
}

void Localisation::UpdateIMU(float imuYaw, float dt)
{
    (void)dt; // niet nodig hier maar kan je loggen

    float z = imuYaw;

    // innovation
    float y = z - theta;

    // normalize angle error
    if (y > M_PI) y -= 2.0f * M_PI;
    if (y < -M_PI) y += 2.0f * M_PI;

    // S = P + R
    float S = P[2][2] + R;

    // Kalman gain
    float Kx = P[0][2] / S;
    float Ky = P[1][2] / S;
    float Kt = P[2][2] / S;

    // update state
    x     += Kx * y;
    y     += Ky * y;
    theta += Kt * y;

    // update covariance (only theta coupling simplified)
    P[0][2] -= Kx * P[2][2];
    P[1][2] -= Ky * P[2][2];
    P[2][2] -= Kt * P[2][2];
}



float Localisation::GetX() const { return x; }
float Localisation::GetY() const { return y; }
float Localisation::GetTheta() const { return theta; }