#include "Localisation.h"
#include <cmath>
#include <cstdio>

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
//  wheelBaseMm : hart-op-hart afstand tussen de wielen in mm (= 219.0f).
// ─────────────────────────────────────────────────────────────────
Localisation::Localisation(float wheelBaseMm)
    : x(0.0f), y(0.0f), theta(0.0f)
    , wheelBase(wheelBaseMm)
    , x_anchor(0.0f), y_anchor(0.0f), theta_anchor(0.0f)
    , totalEncDist(0.0f), totalLocDist(0.0f)
    , prevX(0.0f), prevY(0.0f)
    , debugTickCounter(0)
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
//    omega  [rad/s]  = (vR - vL) / wheelBase
//    x, y   [mm]     += v * cos/sin * dt
//    theta  [graden] += omega * RAD2DEG * dt
// ─────────────────────────────────────────────────────────────────
void Localisation::Predict(float vLeft, float vRight, float dt)
{
    float v     = 0.5f * (vLeft + vRight);       // mm/s
    float omega = (vRight - vLeft) / wheelBase;  // rad/s — BUG2 FIX: consistent 219mm

    float thetaRad = theta * DEG2RAD;
    float c = std::cos(thetaRad);
    float s = std::sin(thetaRad);

    // Bewaar positie voor de update om lokalisatie-afstand bij te houden
    prevX = x;
    prevY = y;

    x     += v * c * dt;
    y     += v * s * dt;
    theta  = NormalizeDeg(theta + omega * RAD2DEG * dt);

    // ── Encoder- vs lokalisatie-afstand accumuleren ────────────────
    // encoderDist: wat de wielen claimen (v * dt, 1D afstand)
    // locDist:     werkelijke 2D verplaatsing na EKF-update
    // Ratio ver van 1.0 → slip, verkeerde wheelbase, of encoderfout
    float encoderDist = std::fabs(v * dt);
    float locDist     = std::hypot(x - prevX, y - prevY);
    totalEncDist += encoderDist;
    totalLocDist += locDist;

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

    // ── Debug print elke 10 ticks (~1 seconde bij 100ms loop) ─────
    // Wat te letten op:
    //   ratio ~1.0  → encoder en lokalisatie komen overeen (goed)
    //   ratio < 0.9 → slip of de robot rijdt niet rechtdoor
    //   ratio > 1.1 → encoders overdrijven of wheelbase is te klein
    //   P groeit snel → model onzeker, IMU/ICP nodig
    ++debugTickCounter;
    if (debugTickCounter >= 10) {
        debugTickCounter = 0;
        printf("[LOC] pos=(%.1f,%.1f) theta=%.1f | "
               "vL=%.1f vR=%.1f | "
               "enc=%.0fmm loc=%.0fmm ratio=%.3f | "
               "P=(%.3f,%.3f,%.3f)\n",
               x, y, theta,
               vLeft, vRight,
               totalEncDist, totalLocDist,
               totalEncDist > 0.1f ? totalLocDist / totalEncDist : 1.0f,
               P[0][0], P[1][1], P[2][2]);
    }
}

// ─────────────────────────────────────────────────────────────────
//  UpdateIMU  —  EKF correctiestap op basis van IMU-yaw (graden)
// ─────────────────────────────────────────────────────────────────
void Localisation::UpdateIMU(float imuYawDeg, float /*dt*/)
{
    float thetaVoor = theta;
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

    // Debug: grote IMU-correcties wijzen op odometrie-drift of IMU-ruis
    float correctie = NormalizeDeg(theta - thetaVoor);
    if (std::fabs(correctie) > 2.0f) {
        printf("[IMU-CORR] grote correctie: imu=%.2f ekf_voor=%.2f correctie=%.2f innov=%.2f Kt=%.3f\n",
               imuYawDeg, thetaVoor, correctie, innov, Kt);
    }
}

float Localisation::GetX()     const { return x;     }
float Localisation::GetY()     const { return y;     }
float Localisation::GetTheta() const { return theta; }

// ─────────────────────────────────────────────────────────────────
//  ApplyIcpCorrection  —  pas ICP scan-matching correctie toe
//
//  BUG1 FIX: theta was cumulatief (theta += dtheta). Odometrie had
//  theta al bijgewerkt met dezelfde rotatie die ICP rapporteert →
//  dubbeltelling. Nu anchor-based voor alle drie variabelen.
//
//  BUG3 gevolg: MainPi5 moet NA deze aanroep loc.GetTheta() gebruiken
//  als heading voor Position, NIET huidigeImuYaw += icp.dtheta.
// ─────────────────────────────────────────────────────────────────
void Localisation::ApplyIcpCorrection(float dx, float dy, float dtheta)
{
    float xVoor     = x;
    float yVoor     = y;
    float thetaVoor = theta;

    // Anchor-based voor alle drie — geen dubbeltelling mogelijk
    x     = x_anchor     + dx;
    y     = y_anchor     + dy;
    theta = NormalizeDeg(theta_anchor + dtheta);

    // Verschil tussen wat odometrie dacht en wat ICP zegt
    float corrX = x - xVoor;
    float corrY = y - yVoor;
    float corrT = NormalizeDeg(theta - thetaVoor);

    // Grote correcties (>50mm of >5°) wijzen op slip of slechte ICP-match
    // die toch als valid doorging — let hier goed op in de logs
    printf("[ICP-CORR] icp=(%.1f,%.1f,%.2f) correctie=(%.1f,%.1f,%.2f) anker=(%.1f,%.1f,%.1f)%s\n",
           dx, dy, dtheta,
           corrX, corrY, corrT,
           x_anchor, y_anchor, theta_anchor,
           (std::fabs(corrX) > 50.0f || std::fabs(corrY) > 50.0f || std::fabs(corrT) > 5.0f)
               ? " *** GROTE CORRECTIE ***" : "");

    // Nieuw anker voor de volgende ICP-match
    x_anchor     = x;
    y_anchor     = y;
    theta_anchor = theta;

    // Verklein onzekerheid als ICP slaagt
    P[0][0] *= 0.8f;
    P[1][1] *= 0.8f;
    P[2][2] *= 0.8f;
}

// ─────────────────────────────────────────────────────────────────
//  SetIcpAnchor  —  synchroniseer anker na mislukte ICP
//
//  Aanroepen wanneer ICP mislukt. Zonder dit springt de positie
//  bij de volgende geslaagde ICP terug naar de positie van de
//  vorige geslaagde scan, ook al heeft odometrie sindsdien meters
//  verder gereden.
// ─────────────────────────────────────────────────────────────────
void Localisation::SetIcpAnchor()
{
    printf("[ICP-ANCHOR] mislukt — anker bijgewerkt naar (%.1f,%.1f,%.1f)\n",
           x, y, theta);

    x_anchor     = x;
    y_anchor     = y;
    theta_anchor = theta;
}

// ─────────────────────────────────────────────────────────────────
//  ResetDistanceCounters  —  nulstellen voor gerichte meting
// ─────────────────────────────────────────────────────────────────
void Localisation::ResetDistanceCounters()
{
    totalEncDist = 0.0f;
    totalLocDist = 0.0f;
    printf("[LOC] afstandstellers gereset\n");
}