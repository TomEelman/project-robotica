#pragma once
#include <vector>
#include <cmath>

// ─────────────────────────────────────────────────────────────────
//  ScanMatcher  —  lichtgewicht 2D ICP scan-to-scan matcher
//
//  Vergelijkt de huidige scan met de vorige en geeft een (dx, dy, dtheta)
//  correctie terug die de lokalisatie-drift vermindert.
//
//  Alleen gebruikt bij LIDAR-scans — geen extra hardware nodig.
//  De correctie is beperkt (MAX_TRANS_MM, MAX_ROT_DEG) zodat grote
//  fouten door slip niet als correctie worden doorgegeven.
// ─────────────────────────────────────────────────────────────────

struct ScanPoint {
    float x, y;  // in mm, robotframe
};

struct IcpResult {
    float dx;       // mm
    float dy;       // mm
    float dtheta;   // graden
    float fitness;  // gemiddelde afstand na matching (lager = beter)
    bool  valid;    // false als matching mislukt of onbetrouwbaar
};

class ScanMatcher {
public:
    // max_trans_mm: maximale toegestane translatie-correctie
    // max_rot_deg:  maximale toegestane rotatie-correctie
    explicit ScanMatcher(float max_trans_mm = 80.0f,
                         float max_rot_deg  = 8.0f);

    // Voer ICP uit tussen de opgeslagen vorige scan en de nieuwe scan.
    // ranges[360]: afstanden in mm (0 = ongeldig)
    // robotTheta:  huidige robothoek in graden (voor frame-transformatie)
    // initDx/initDy: encoder-gebaseerde initiële verplaatsing (mm, wereldframe).
    //   De huidige scan wordt hier EERST mee verschoven zodat ICP al dicht bij
    //   de oplossing begint en minder kans heeft op een verkeerd lokaal minimum.
    // Geeft de correctie terug die op de positie opgeteld moet worden.
    IcpResult Match(const float ranges[360], float robotTheta,
                    float initDx = 0.0f, float initDy = 0.0f);

    // Reset: gooi de vorige scan weg (bijv. na grote sprong of reset)
    void Reset();

    bool HasPrevScan() const { return hasPrev; }

private:
    static constexpr int    MAX_ITER      = 20;
    static constexpr float  CONV_THRESH   = 0.5f;   // mm — convergentie
    // MAX_DIST_MM: maximale afstand voor een punt-paar in FindCorrespondences.
    // Bij 278mm/s en 100ms scan-interval beweegt de robot ~28mm per scan.
    // 200mm was te groot → veel valse koppelingen → ICP convergeerde naar
    // verkeerd lokaal minimum → grote fictieve trans/rot → afgewezen → drift.
    // 80mm = 3× verwachte beweging: voldoende marge, elimineert verre foute paren.
    static constexpr float  MAX_DIST_MM   = 80.0f;
    static constexpr int    MIN_POINTS    = 30;      // min bruikbare punten

    float maxTransMm;
    float maxRotDeg;

    bool                 hasPrev;
    std::vector<ScanPoint> prevPoints;

    // Zet ranges om naar 2D punten in wereldframe
    static std::vector<ScanPoint> RangesToPoints(
        const float ranges[360], float theta);

    // Bereken zwaartepunt
    static void Centroid(const std::vector<ScanPoint>& pts,
                         float& cx, float& cy);

    // Vind dichtstbijzijnde buurpunten (brute force, max MAX_DIST_MM)
    static void FindCorrespondences(
        const std::vector<ScanPoint>& src,
        const std::vector<ScanPoint>& dst,
        std::vector<int>& indices,
        std::vector<float>& dists);

    // SVD-loze 2D rotatie/translatie via cross-covariantie
    static bool ComputeTransform(
        const std::vector<ScanPoint>& src,
        const std::vector<ScanPoint>& dst,
        const std::vector<int>& idx,
        float& outDx, float& outDy, float& outDtheta);

    // Transformeer punten
    static void TransformPoints(std::vector<ScanPoint>& pts,
                                 float dx, float dy, float dtheta);
};