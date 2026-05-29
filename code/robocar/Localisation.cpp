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
    , prevVLeft(-99999.0f), prevVRight(-99999.0f)
    , staleCount(0)
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
    // ── Stale-data detectie ────────────────────────────────────────
    // Identieke waarden als vorige tick = waarschijnlijk geen nieuw
    // Pico-pakket ontvangen. We integreren dan dezelfde snelheid
    // opnieuw, wat drift veroorzaakt.
    if (vLeft == prevVLeft && vRight == prevVRight) {
        ++staleCount;
    } else {
        if (staleCount > 1) {
            printf("[LOC-STALE] %d ticks lang dezelfde encoderwaarden "
                   "(vL=%.1f vR=%.1f) — mogelijk geen vers Pico-pakket!\n",
                   staleCount, prevVLeft, prevVRight);
        }
        staleCount = 1;
    }
    prevVLeft  = vLeft;
    prevVRight = vRight;

    // ── Kinematica ─────────────────────────────────────────────────
    float v        = 0.5f * (vLeft + vRight);       // mm/s
    float omega    = (vRight - vLeft) / wheelBase;  // rad/s — consistent 219mm
    float omegaDeg = omega * RAD2DEG;               // graden/s (voor debug)

    float thetaRad  = theta * DEG2RAD;
    float c         = std::cos(thetaRad);
    float s         = std::sin(thetaRad);

    prevX = x;
    prevY = y;
    float thetaVoor = theta;

    float dx_enc    = v * c * dt;
    float dy_enc    = v * s * dt;
    float dtheta_enc = omegaDeg * dt;

    x     += dx_enc;
    y     += dy_enc;
    theta  = NormalizeDeg(theta + dtheta_enc);

    // ── Afstandstellers ────────────────────────────────────────────
    float encoderDist = std::fabs(v * dt);
    float locDist     = std::hypot(x - prevX, y - prevY);
    totalEncDist += encoderDist;
    totalLocDist += locDist;

    // ── Per-tick debug ─────────────────────────────────────────────
    // Lees hieruit af:
    //   omega≠0 terwijl je rechtdoor rijdt → encoder-asymmetrie / wieldiameter-verschil
    //   dtheta groot maar IMU zegt 0 → EKF trekt theta verkeerde kant op
    //   [STALE] → geen vers Pico-pakket, dezelfde snelheid opnieuw geïntegreerd
    printf("[LOC-ENC] vL=%6.1f vR=%6.1f | v=%6.1f omega=%+5.2f°/s | "
           "dx=%+5.1f dy=%+5.1f dtheta=%+5.2f° | "
           "pos=(%.1f,%.1f,%.1f)%s\n",
           vLeft, vRight, v, omegaDeg,
           dx_enc, dy_enc, dtheta_enc,
           x, y, theta,
           (staleCount > 1) ? " [STALE]" : "");

    // ── Jacobiaan + covariantie P ──────────────────────────────────
    float vdt = v * dt;
    float F[3][3] = {
        {1.0f, 0.0f, -vdt * s * DEG2RAD},
        {0.0f, 1.0f,  vdt * c * DEG2RAD},
        {0.0f, 0.0f,  1.0f             }
    };

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

    // ── Samenvattende print elke 10 ticks (~1 seconde) ────────────
    ++debugTickCounter;
    if (debugTickCounter >= 10) {
        debugTickCounter = 0;
        float ratio = totalEncDist > 0.1f ? totalLocDist / totalEncDist : 1.0f;
        printf("[LOC-SUM] pos=(%.1f,%.1f) theta=%.1f | "
               "totEnc=%.0fmm totLoc=%.0fmm ratio=%.3f | "
               "P=(%.3f,%.3f,%.3f)%s\n",
               x, y, theta,
               totalEncDist, totalLocDist, ratio,
               P[0][0], P[1][1], P[2][2],
               (ratio < 0.9f || ratio > 1.1f) ? " *** RATIO AFWIJKEND ***" : "");
    }
}

// ─────────────────────────────────────────────────────────────────
//  UpdateIMU  —  EKF correctiestap op basis van IMU-yaw (graden)
// ─────────────────────────────────────────────────────────────────
void Localisation::UpdateIMU(float imuYawDeg, float /*dt*/)
{
    float thetaVoor = theta;
    float innov     = NormalizeDeg(imuYawDeg - theta);

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

    float correctie = NormalizeDeg(theta - thetaVoor);

    // Altijd printen zodat je per tick kunt zien:
    //   innov≈0 en Kt groot → IMU bevestigt odometrie (goed)
    //   innov groot en Kt groot → IMU corrigeert encoder-drift (gewenst)
    //   innov groot maar Kt klein → EKF vertrouwt IMU nauwelijks → heading loopt weg
    //   correctie en innov tegengesteld teken → oscillatie tussen IMU en encoders
    printf("[LOC-IMU]  imu=%+7.2f ekf_voor=%+7.2f innov=%+6.2f Kt=%.3f -> dtheta=%+5.2f theta=%+7.2f%s\n",
           imuYawDeg, thetaVoor, innov, Kt, correctie, theta,
           std::fabs(correctie) > 2.0f ? " *** GROTE CORRECTIE ***" : "");
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