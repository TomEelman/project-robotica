#ifndef LOCALISATION_H
#define LOCALISATION_H

class Localisation {
public:
    // wheelBaseMm: hart-op-hart wielafstand in mm (219.0f voor deze robot).
    explicit Localisation(float wheelBaseMm);

    // ── EKF predict + correct ─────────────────────────────────────
    // vLeft / vRight in mm/s, dt in seconden
    void Predict(float vLeft, float vRight, float dt);

    // imuYawDeg: gecorrigeerde yaw van de IMU in graden (na offset-aftrek)
    void UpdateIMU(float imuYawDeg, float dt);

    // ── State accessors ───────────────────────────────────────────
    float GetX()     const; // mm
    float GetY()     const; // mm
    float GetTheta() const; // graden, genormaliseerd naar (-180, 180]

    // ── Debug hulp ────────────────────────────────────────────────
    // Reset de encoder- en lokalisatie-afstandstellers voor gerichte
    // validatiemetingen (bijv. rij 1m rechtdoor en controleer de ratio).
    void ResetDistanceCounters();

    float GetTotalEncDist() const { return totalEncDist; }
    float GetTotalLocDist() const { return totalLocDist; }

private:
    // ── EKF toestand ──────────────────────────────────────────────
    float x;        // mm
    float y;        // mm
    float theta;    // graden, [-180, 180]

    float P[3][3];  // covariantiematrix
    float Q[3][3];  // procesruis
    float R;        // meetsruis IMU-yaw

    float wheelBase; // mm — consistent 219mm, NERGENS 235mm gebruiken

    // ── Debug-tellers ─────────────────────────────────────────────
    float totalEncDist;    // gecumuleerde encoder-afstand (mm)
    float totalLocDist;    // gecumuleerde 2D-verplaatsing na EKF (mm)
    float prevX, prevY;    // positie vorige Predict-aanroep
    int   debugTickCounter;

    // Stale-data detectie: als vLinks en vRechts identiek zijn aan de
    // vorige aanroep, is er waarschijnlijk geen nieuw Pico-pakket binnengekomen.
    float prevVLeft;
    float prevVRight;
    int   staleCount;      // opeenvolgende ticks met identieke snelheidswaarden

    // IMU wraparound fix: delta-yaw bijhouden i.p.v. absolute yaw.
    // Voorkomt heading-sprong bij 359°→0° overgang.
    float prevImuYaw;
    bool  prevImuInitialized;
};

#endif