#include "Localisation.h"
#include <cmath>
#include <cstdio>

static constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
static constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

// NIEUW — veilig
static float NormalizeDeg(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
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
    , prevImuYaw(0.0f), prevImuInitialized(false)
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
            //printf("[LOC-STALE] %d ticks lang dezelfde encoderwaarden "
              //     "(vL=%.1f vR=%.1f) — mogelijk geen vers Pico-pakket!\n",
                //   staleCount, prevVLeft, prevVRight);
        }
        staleCount = 1;
    }
    prevVLeft  = vLeft;
    prevVRight = vRight;

    // ── Kinematica ─────────────────────────────────────────────────
    float v        = 0.5f * (vLeft + vRight);       // mm/s
    
    //float omega    = (vRight - vLeft) / wheelBase;  // rad/s — consistent 219mm
    //float omegaDeg = omega * RAD2DEG;               // graden/s (voor debug)

    float thetaRad  = theta * DEG2RAD;
    float c         = std::cos(thetaRad);
    float s         = std::sin(thetaRad);

    prevX = x;
    prevY = y;

    float dx_enc    = v * c * dt;
    float dy_enc    = v * s * dt;

    // Positie alleen bijwerken bij lineaire beweging. Bij een pure turn
    // (linear ≈ 0, vLeft ≈ -vRight) is v ≈ 0 en is de werkelijke
    // verplaatsing verwaarloosbaar klein op de kaart — die ruis weghouden.
    constexpr float LIN_DEAD_MM_S = 5.0f;
    if (std::fabs(v) > LIN_DEAD_MM_S) {
        x += dx_enc;
        y += dy_enc;
    }
    // theta NIET bijwerken uit de encoders: heading komt volledig van de IMU
    // (zie UpdateIMU). Encoder-rotatie is onbetrouwbaar door
    // wielasymmetrie/slip.

    // ── Afstandstellers ────────────────────────────────────────────
    float encoderDist = std::fabs(v * dt);
    float locDist     = (std::fabs(v) > LIN_DEAD_MM_S)
                        ? std::hypot(x - prevX, y - prevY)
                        : 0.0f;
    totalEncDist += encoderDist;
    totalLocDist += locDist;

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
}

// ─────────────────────────────────────────────────────────────────
//  UpdateIMU  —  EKF correctiestap op basis van IMU-yaw (graden)
// ─────────────────────────────────────────────────────────────────
void Localisation::UpdateIMU(float imuYawDeg, float /*dt*/)
{
    // Gebruik delta-yaw i.p.v. absolute yaw om wraparound te voorkomen.
    // Als de IMU van 179° naar -179° springt (of 359°→0°), geeft de
    // absolute overschrijving een knik van 358° in de heading.
    // Door de delta te normaliseren blijft de stap altijd klein (-180..+180).
    //
    // prevImuYaw bijhouden: eerste tick initialiseren op huidige waarde.
    if (!prevImuInitialized) {
        prevImuYaw         = imuYawDeg;
        prevImuInitialized = true;
    }

    float delta = NormalizeDeg(imuYawDeg - prevImuYaw);
    prevImuYaw  = imuYawDeg;

    theta = NormalizeDeg(theta + delta);

    // Heading-onzekerheid klein houden: IMU is de heading-waarheid.
    P[2][2] = 0.01f;
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

    // Alleen translatie uit ICP toepassen. Heading komt uitsluitend van de
    // IMU, dus theta wordt hier NIET aangeraakt — de ICP-rotatie is in deze
    // omgeving slecht bepaald en liet de kaart eerder krombuigen.
    x = x_anchor + dx;
    y = y_anchor + dy;

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
    /*
    printf("[ICP-ANCHOR] mislukt — anker bijgewerkt naar (%.1f,%.1f,%.1f)\n",
           x, y, theta);
    */
           

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