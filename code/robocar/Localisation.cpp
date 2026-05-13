#include "Localisation.h"
#include <cmath>

static constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
static constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

// Normaliseer graden naar (-180, 180]
static float NormalizeDeg(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

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
    float omega = (vRight - vLeft) / wheelBase; // graden/s (als vLeft/vRight in mm/s zijn en wheelBase in mm)

    float thetaRad = theta * DEG2RAD;
    float c = std::cos(thetaRad);
    float s = std::sin(thetaRad);

    x     += v * c * dt;
    y     += v * s * dt;
    theta  = NormalizeDeg(theta + omega * RAD2DEG * dt);

    // Jacobiaan F (3×3) — hoekterm in graden/s dus omgerekend
    float vdt = v * dt;
    float F[3][3] = {
        {1.0f, 0.0f, -vdt * s * DEG2RAD},
        {0.0f, 1.0f,  vdt * c * DEG2RAD},
        {0.0f, 0.0f,  1.0f             }
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
void Localisation::UpdateIMU(float imuYawDeg, float /*dt*/)
{
    // Innovatie: verschil in graden, genormaliseerd
    float innov = NormalizeDeg(imuYawDeg - theta);

    // Innovatiecovariantie S = P[2][2] + R
    float S = P[2][2] + R;
    if (S < 1e-9f) return;

    // Kalman-gain voor x, y, theta
    float Kx = P[0][2] / S;
    float Ky = P[1][2] / S;
    float Kt = P[2][2] / S;

    x     += Kx * innov;
    y     += Ky * innov;
    theta  = NormalizeDeg(theta + Kt * innov);

    // Covariantie bijwerken
    P[0][2] -= Kx * P[2][2];
    P[1][2] -= Ky * P[2][2];
    P[2][2] *= (1.0f - Kt);
}

float Localisation::GetX()     const { return x;     }
float Localisation::GetY()     const { return y;     }
float Localisation::GetTheta() const { return theta; } // al in graden