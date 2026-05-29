#ifndef LOCALISATION_H
#define LOCALISATION_H

class Localisation {
public:
    Localisation(float wheelBase);

    // vLeft / vRight in mm/s, dt in seconden
    void Predict(float vLeft, float vRight, float dt);

    // imuYawDeg: yaw van de IMU in graden
    void UpdateIMU(float imuYawDeg, float dt);

    // Pas een ICP scan-matching correctie toe op x, y en theta.
    // dx/dy in mm, dtheta in graden.
    // Alleen aanroepen als IcpResult.valid == true.
    void ApplyIcpCorrection(float dx, float dy, float dtheta);

    // Synchroniseer het ICP-ankerpunt met de huidige odometriepositie.
    // Aanroepen wanneer ICP mislukt (ScanMatcher geeft valid=false),
    // zodat het volgende geslaagde ICP weet waar de robot nu staat.
    void SetIcpAnchor();

    float GetX()     const; // mm
    float GetY()     const; // mm
    float GetTheta() const; // graden, genormaliseerd naar (-180, 180]

private:
    float x;
    float y;
    float theta; // intern opgeslagen in graden

    float P[3][3];
    float Q[3][3];
    float R;

    float wheelBase;

    // Ankerpunten: positie + heading op het moment van de vorige geslaagde
    // ICP-scan. ApplyIcpCorrection stelt x/y/theta in als anchor + delta,
    // zodat odometrie-drift tussen twee scans volledig vervangen wordt door
    // de ICP-meting (geen dubbeltelling).
    float x_anchor;
    float y_anchor;
    float theta_anchor; // graden
};

#endif