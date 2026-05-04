#include "Localisation.h"
#include <cmath>

Localisation::Localisation(float wheelBase)
    : x(0.0f), y(0.0f), theta(0.0f), wheelBase(wheelBase)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = Q[i][j] = 0.0f;

    P[0][0] = 0.01f;  P[1][1] = 0.01f;  P[2][2] = 0.01f;
    Q[0][0] = 0.02f;  Q[1][1] = 0.02f;  Q[2][2] = 0.01f;
    R = 0.01f;
}

// ── EKF predict-stap (odometrie) ──────────────────────────────────
void Localisation::Predict(float vLeft, float vRight, float dt)
{
    float v     = 0.5f * (vLeft + vRight);
    float omega = (vRight - vLeft) / wheelBase;

    float c = std::cos(theta);
    float s = std::sin(theta);

    x     += v * c * dt;
    y     += v * s * dt;
    theta += omega * dt;

    // Normaliseer hoek naar (-π, π]
    while (theta >  M_PI) theta -= 2.0f * static_cast<float>(M_PI);
    while (theta < -M_PI) theta += 2.0f * static_cast<float>(M_PI);

    // Jacobiaan F (3×3)
    float F[3][3] = {
        {1.0f, 0.0f, -v * s * dt},
        {0.0f, 1.0f,  v * c * dt},
        {0.0f, 0.0f,  1.0f      }
    };

    // P ← F·P·Fᵀ + Q
    float Pnew[3][3] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++)
                for (int l = 0; l < 3; l++)
                    Pnew[i][j] += F[i][k] * P[k][l] * F[j][l];
            Pnew[i][j] += Q[i][j];
        }

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = Pnew[i][j];
}

// ── EKF update-stap (IMU yaw) ─────────────────────────────────────
// BUGFIX: origineel gebruikte 'y' als zowel lokale variabele als lidvariabele.
void Localisation::UpdateIMU(float imuYaw, float /*dt*/)
{
    // Innovatie (hoekfout)
    float innov = imuYaw - theta;

    // Normaliseer naar (-π, π]
    while (innov >  M_PI) innov -= 2.0f * static_cast<float>(M_PI);
    while (innov < -M_PI) innov += 2.0f * static_cast<float>(M_PI);

    // Innovatiecovariantie S = P[2][2] + R
    float S = P[2][2] + R;
    if (S < 1e-9f) return;  // degenerate case afvangen

    // Kalman-gain voor x, y, theta
    float Kx = P[0][2] / S;
    float Ky = P[1][2] / S;
    float Kt = P[2][2] / S;

    // Toestand bijwerken (let op: y is nu de lidvariabele, niet de innovatie)
    x     += Kx * innov;
    y     += Ky * innov;
    theta += Kt * innov;

    // Covariatie bijwerken (Joseph-vorm vereenvoudigd voor 1D meting)
    P[0][2] -= Kx * P[2][2];
    P[1][2] -= Ky * P[2][2];
    P[2][2] *= (1.0f - Kt);   // = P[2][2] - Kt*P[2][2]
}

float Localisation::GetX()     const { return x;     }
float Localisation::GetY()     const { return y;     }
float Localisation::GetTheta() const { return theta; }