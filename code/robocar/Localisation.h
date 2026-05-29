#ifndef LOCALISATION_H
#define LOCALISATION_H

// ─────────────────────────────────────────────────────────────────
//  Localisation — één heading-bron, geen dubbeltelling
//
//  Ontwerpkeuzes (zie ook MainPi5):
//   • De IMU-yaw is de ENIGE heading-bron. Een gyro is veel
//     betrouwbaarder voor heading dan differentiële wiel-odometrie,
//     dus we integreren wielverschillen NIET meer in theta.
//   • De IMU is CW-positief (rechtsom = +). De kaart projecteert de
//     LIDAR met standaard cos/sin (CCW-conventie). Om beide in
//     hetzelfde frame te krijgen draaien we het IMU-teken om:
//         theta = NormDeg( IMU_SIGN * (imuYaw - imuOffset) + headingBias )
//     met IMU_SIGN = -1. Klopt de opgeslagen kaart gespiegeld/uitgesmeerd
//     bij bochten, dan is dit teken het enige dat je hoeft te flippen.
//   • x/y worden dood-gerekend langs DEZELFDE theta, zodat de richting
//     waarin de positie opschuift exact gelijk is aan de hoek waarmee
//     de LIDAR in de kaart wordt geschreven (geen smear meer).
//   • ICP corrigeert x/y direct en de trage gyro-drift via headingBias.
// ─────────────────────────────────────────────────────────────────
class Localisation {
public:
    explicit Localisation(float wheelBaseMm);

    // Eén control-tick. vLeft/vRight in mm/s (positief = vooruit),
    // imuYawDeg = rauwe IMU-yaw in graden, dt in seconden.
    // De eerste aanroep latcht de IMU-offset (nulpunt).
    void Update(float vLeft, float vRight, float imuYawDeg, float dt);

    // ICP-correctie in het kaart-frame (CCW): dx/dy in mm, dtheta in graden.
    // x/y worden direct verschoven; dtheta corrigeert de gyro-drift via de
    // headingBias (zodat de absolute IMU-heading de volgende tick niet de
    // correctie wegpoetst). Alleen aanroepen bij een geldige ICP-match.
    void ApplyIcp(float dx, float dy, float dtheta);

    // Herstart de schatting (positie naar opgegeven punt, bias/offset behouden).
    void Reset(float x = 0.0f, float y = 0.0f);

    float GetX()     const { return x;     }  // mm
    float GetY()     const { return y;     }  // mm
    float GetTheta() const { return theta; }  // graden, CCW, (-180,180]

private:
    // Flip dit teken als IMU-draairichting niet bij de kaart past.
    static constexpr float IMU_SIGN = -1.0f;

    float x;            // mm
    float y;            // mm
    float theta;        // graden, CCW map-frame

    float wheelBase;    // mm (informatief; heading komt van IMU)

    float imuOffset;    // graden, rauwe IMU-yaw bij start
    bool  imuInit;
    float headingBias;  // graden, trage ICP-driftcorrectie
};

#endif
