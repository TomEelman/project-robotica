#include "Localisation.h"
#include <cmath>

static constexpr float DEG2RAD = 3.14159265358979f / 180.0f;

// Normaliseer graden naar (-180, 180]
static float NormalizeDeg(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

Localisation::Localisation(float wheelBaseMm)
    : x(0.0f), y(0.0f), theta(0.0f)
    , wheelBase(wheelBaseMm)
    , imuOffset(0.0f), imuInit(false), headingBias(0.0f)
{
}

// ─────────────────────────────────────────────────────────────────
//  Update — heading van IMU, x/y dood-gerekend langs die heading
// ─────────────────────────────────────────────────────────────────
void Localisation::Update(float vLeft, float vRight, float imuYawDeg, float dt)
{
    // Eerste meting bepaalt het nulpunt van de IMU.
    if (!imuInit) {
        imuOffset = imuYawDeg;
        imuInit   = true;
    }

    // Heading: rauwe IMU naar CCW kaart-frame + trage ICP-driftcorrectie.
    theta = NormalizeDeg(IMU_SIGN * (imuYawDeg - imuOffset) + headingBias);

    // Voorwaartse snelheid uit de wielen. Bij een draai op de plek is
    // vLeft ≈ -vRight, dus v ≈ 0 en schuift de positie niet op — precies wat
    // we willen: alleen de heading verandert, x/y blijven staan.
    float v = 0.5f * (vLeft + vRight);   // mm/s

    float thetaRad = theta * DEG2RAD;
    x += v * std::cos(thetaRad) * dt;    // mm
    y += v * std::sin(thetaRad) * dt;    // mm
}

// ─────────────────────────────────────────────────────────────────
//  ApplyIcp — scan-matching correctie in het kaart-frame
// ─────────────────────────────────────────────────────────────────
void Localisation::ApplyIcp(float dx, float dy, float dtheta)
{
    x += dx;
    y += dy;
    // dtheta corrigeert de gyro-drift. We schrijven het in de bias zodat de
    // volgende Update (die theta absoluut uit de IMU zet) de correctie behoudt.
    headingBias = NormalizeDeg(headingBias + dtheta);
    theta       = NormalizeDeg(theta + dtheta);
}

void Localisation::Reset(float x_, float y_)
{
    x = x_;
    y = y_;
}
