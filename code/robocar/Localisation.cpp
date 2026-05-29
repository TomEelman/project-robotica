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

// ─────────────────────────────────────────────────────────────────
//  Constructor
//
//  wheelBaseMm : afstand tussen de wielen IN MILLIMETER (bijv. 235.0f).
//                Let op: MainPi5 moet Localisation(235.0f) aanroepen,
//                NIET Localisation(0.235f).
// ─────────────────────────────────────────────────────────────────
Localisation::Localisation(float wheelBaseMm)
    : x(0.0f), y(0.0f), theta(0.0f), wheelBase(wheelBaseMm)
    , x_anchor(0.0f), y_anchor(0.0f)
{
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            P[i][j] = Q[i][j] = 0.0f;

    P[0][0] = 1.0f;   P[1][1] = 1.0f;   P[2][2] = 1.0f;
    Q[0][0] = 0.5f;   Q[1][1] = 0.5f;   Q[2][2] = 0.1f;
    R = 0.5f;
}

// ─────────────────────────────────────────────────────────────────
//  Predict  —  odometrie EKF
//
//  vLeft, vRight : wielsnelheden in mm/s  (positief = vooruit)
//  dt            : tijdstap in seconden
//
//  Eenheden:
//    v      [mm/s]
//    omega  [rad/s]  = (vR - vL) [mm/s] / wheelBase [mm]  = rad/s  ✓
//    x, y   [mm]     += v [mm/s] * cos/sin * dt [s]
//    theta  [graden] += omega [rad/s] * RAD2DEG * dt [s]
// ─────────────────────────────────────────────────────────────────
void Localisation::Predict(float vLeft, float vRight, float dt)
{
    float v     = 0.5f * (vLeft + vRight);          // mm/s
    float omega = (vRight - vLeft) / wheelBase;      // rad/s

    float thetaRad = theta * DEG2RAD;
    float c = std::cos(thetaRad);
    float s = std::sin(thetaRad);

    x     += v * c * dt;                             // mm
    y     += v * s * dt;                             // mm
    theta  = NormalizeDeg(theta + omega * RAD2DEG * dt);  // graden

    // Jacobiaan F — ∂x/∂θ en ∂y/∂θ in mm/graad
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

// ─────────────────────────────────────────────────────────────────
//  UpdateIMU  —  EKF correctiestap op basis van IMU-yaw (graden)
// ─────────────────────────────────────────────────────────────────
void Localisation::UpdateIMU(float imuYawDeg, float /*dt*/)
{
    float innov = NormalizeDeg(imuYawDeg - theta);

    float S = P[2][2] + R;
    if (S < 1e-9f) return;

    float Kx = P[0][2] / S;
    float Ky = P[1][2] / S;
    float Kt = P[2][2] / S;

    x     += Kx * innov;
    y     += Ky * innov;
    theta  = NormalizeDeg(theta + Kt * innov);

    P[0][2] -= Kx * P[2][2];
    P[1][2] -= Ky * P[2][2];
    P[2][2] *= (1.0f - Kt);
}

float Localisation::GetX()     const { return x;     }   // mm
float Localisation::GetY()     const { return y;     }   // mm
float Localisation::GetTheta() const { return theta; }   // graden
// ──────────────────────────────────────────────────────────────────
//  ApplyIcpCorrection  —  pas ICP scan-matching correctie toe
//
//  De correctie wordt met een demping (ALPHA) gemengd zodat
//  ruisige ICP-resultaten de EKF niet destabiliseren.
// ──────────────────────────────────────────────────────────────────
void Localisation::ApplyIcpCorrection(float dx, float dy, float dtheta)
{
    // Gebruik het ankerpunt (positie bij vorige geslaagde scan) + ICP-meting
    // als nieuwe positie, in plaats van optellen bij de al-door-odometrie-
    // bijgewerkte positie. Odometrie en ICP meten beide de verplaatsing,
    // dus optellen zou 140% van de echte beweging geven (dubbeltelling).
    x      = x_anchor + dx;
    y      = y_anchor + dy;
    theta  = NormalizeDeg(theta + dtheta);

    // Bewaar als nieuw anker voor de volgende ICP-match
    x_anchor = x;
    y_anchor = y;

    // Verklein onzekerheid licht als ICP slaagt
    P[0][0] *= 0.8f;
    P[1][1] *= 0.8f;
    P[2][2] *= 0.8f;
}

void Localisation::SetIcpAnchor()
{
    // Synchroniseer het anker met de huidige odometriepositie.
    // Aanroepen wanneer ICP mislukt zodat het volgende geslaagde ICP
    // weet waar de robot volgens odometrie stond bij de vorige scan.
    x_anchor = x;
    y_anchor = y;
}