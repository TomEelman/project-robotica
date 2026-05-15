#include "Navigator.h"
#include <cmath>

// ─────────────────────────────────────────────────────────────────
//  NormDeg
// ─────────────────────────────────────────────────────────────────

float NormDeg(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}


// ─────────────────────────────────────────────────────────────────
//  AnalyseerScan
// ─────────────────────────────────────────────────────────────────

static float g_vorigeUitwijk = 0.0f;

ScanAnalyse AnalyseerScan(const float ranges[360]) {
    static constexpr float KRITIEK_MM  = 350.0f;
    static constexpr float REMMEN_MM   = 500.0f;
    static constexpr float VEILIG_MM   = 700.0f;
    static constexpr float CHASSIS_MM  = 280.0f;
    static constexpr float MAX_RANGE   = 8000.0f;
    static constexpr float HYSTERESIS  = 200.0f;

    float sector[36];
    for (int s = 0; s < 36; ++s) sector[s] = MAX_RANGE;
    for (int a = 0; a < 360; ++a) {
        float r = ranges[a];
        if (r <= 0.0f || r > MAX_RANGE) continue;
        int s = (a / 10) % 36;
        if (r < sector[s]) sector[s] = r;
    }

    auto sectorMin = [&](int van, int tot) -> float {
        float m = MAX_RANGE;
        for (int s = van % 36; s != (tot % 36); s = (s + 1) % 36)
            if (sector[s] < m) m = sector[s];
        return m;
    };

    float minVoorSmal = MAX_RANGE;
    { float m1 = sectorMin(0,4), m2 = sectorMin(33,36); minVoorSmal = (m1<m2)?m1:m2; }
    float minVoor = MAX_RANGE;
    { float m1 = sectorMin(0,5), m2 = sectorMin(31,36); minVoor = (m1<m2)?m1:m2; }

    float minRechtsZijde = sectorMin(5,  13);
    float minLinksZijde  = sectorMin(23, 31);
    float minRechts      = sectorMin(5,  18);
    float minLinks       = sectorMin(19, 35);
    float ruimteRechts   = sectorMin(1,  18);
    float ruimteLinks    = sectorMin(19, 36);
    float minAchter      = sectorMin(15, 21);  // 150°–210°

    bool inGang = (minVoor >= REMMEN_MM) &&
                  (minRechtsZijde < VEILIG_MM) && (minLinksZijde < VEILIG_MM);

    ScanAnalyse res{};
    res.ruimteLinks  = ruimteLinks;
    res.ruimteRechts = ruimteRechts;
    res.minVoor      = minVoor;
    res.minLinks     = minLinks;
    res.minRechts    = minRechts;
    res.minAchter    = minAchter;
    res.uitwijkHoek  = g_vorigeUitwijk;

    // Volledig vrij veld: geen obstakellogica nodig
    if (minVoor >= VEILIG_MM) {
        res.staat = 0;
        g_vorigeUitwijk = 0.0f;
        return res;
    }

    // In een gang: staat=0 zodat de hoofdlus gangcentrering kan doen
    // via ruimteLinks/ruimteRechts, maar geen uitwijkhoek forceren
    if (inGang) {
        res.staat = 0;
        g_vorigeUitwijk = 0.0f;
        return res;
    }

    bool rechtsVeilig = (minRechtsZijde > CHASSIS_MM);
    bool linksVeilig  = (minLinksZijde  > CHASSIS_MM);
    float scoreRechts = rechtsVeilig ? ruimteRechts : 0.0f;
    float scoreLinks  = linksVeilig  ? ruimteLinks  : 0.0f;
    if (g_vorigeUitwijk > 0.0f) scoreRechts += HYSTERESIS;
    if (g_vorigeUitwijk < 0.0f) scoreLinks  += HYSTERESIS;

    float nieuweUitwijk = (!rechtsVeilig && !linksVeilig)
        ? ((ruimteRechts >= ruimteLinks) ? +1.0f : -1.0f)
        : ((scoreRechts  >= scoreLinks)  ? +1.0f : -1.0f);

    res.uitwijkHoek = nieuweUitwijk;
    g_vorigeUitwijk = nieuweUitwijk;

    if      (minVoorSmal < KRITIEK_MM) res.staat = 3;
    else if (minVoor     < REMMEN_MM)  res.staat = 2;
    else                                res.staat = 1;
    return res;
}


// ─────────────────────────────────────────────────────────────────
//  Navigator
// ─────────────────────────────────────────────────────────────────

Navigator::Navigator()
    : path()
    , currentTarget(0.0f, 0.0f, 0.0f)
    , isUpdated(false)
    , hasPath(false)
{}

void Navigator::SetPath(const Path& newPath) {
    path = newPath;
    path.Reset();
    hasPath      = !path.IsEmpty();
    gefilterdAng = 0.0f;
    if (hasPath)
        currentTarget = path.GetCurrentWaypoint();
    isUpdated = true;
}

void Navigator::Update(Position current) {
    if (!hasPath || path.IsEmpty()) return;

    while (ReachedPoint(current) && !path.IsEmpty()) {
        path.Advance();
        if (!path.IsEmpty())
            currentTarget = path.GetCurrentWaypoint();
        else
            hasPath = false;
    }
    isUpdated = true;
}

DriveCommand Navigator::GetNextCommand(Position current) {
    if (!hasPath || path.IsEmpty())
        return DriveCommand(0.0f, 0.0f);

    float dist     = CalculateDistance(current, currentTarget);
    float angleErr = CalculateAngleError(current, currentTarget);

    if (dist < REACHED_THRESHOLD_MM)
        return DriveCommand(LINEAR_SPEED_MM_S, 0.0f);

    float absErr   = std::fabs(angleErr);
    float linSpeed = (absErr > SLOW_TURN_THRESHOLD)
                     ? LINEAR_SPEED_MM_S * SLOW_TURN_FACTOR
                     : LINEAR_SPEED_MM_S;

    float gewenstAng = 0.0f;
    if (absErr > ANGLE_DEADBAND_DEG) {
        gewenstAng = ANGULAR_GAIN * absErr;
        if (gewenstAng > MAX_ANGULAR_DEG_S) gewenstAng = MAX_ANGULAR_DEG_S;
        if (angleErr > 0.0f) gewenstAng = -gewenstAng;
    }

    gefilterdAng = ANG_FILTER_ALFA * gewenstAng + (1.0f - ANG_FILTER_ALFA) * gefilterdAng;
    if (std::fabs(gefilterdAng) < 0.5f) gefilterdAng = 0.0f;

    return DriveCommand(linSpeed, gefilterdAng);
}

bool Navigator::ReachedPoint(Position current) const {
    return !path.IsEmpty() &&
           CalculateDistance(current, currentTarget) < REACHED_THRESHOLD_MM;
}

bool Navigator::IsFinished() const { return !hasPath || path.IsEmpty(); }
bool Navigator::IsUpdated()  const { return isUpdated; }

Position Navigator::GetCurrentTarget() const { return currentTarget; }
Path     Navigator::GetPath()          const { return path; }

float Navigator::CalculateDistance(Position a, Position b) const {
    float dx = b.GetX() - a.GetX(), dy = b.GetY() - a.GetY();
    return std::sqrt(dx*dx + dy*dy);
}

float Navigator::CalculateAngleError(Position current, Position target) const {
    float dx = target.GetX() - current.GetX();
    float dy = target.GetY() - current.GetY();
    float desired = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));
    return NormalizeDeg(desired - current.GetTheta());
}

float Navigator::NormalizeDeg(float deg) const {
    return NormDeg(deg);
}