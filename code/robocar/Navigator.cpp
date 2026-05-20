#include "Navigator.h"
#include <cmath>
#include <cstdio>

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
    float minAchter      = sectorMin(15, 21);

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

    if (minVoor >= VEILIG_MM) {
        res.staat = 0;
        g_vorigeUitwijk = 0.0f;
        return res;
    }
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
//  Navigator — constructor
// ─────────────────────────────────────────────────────────────────

Navigator::Navigator()
    : path()
    , currentTarget(0.0f, 0.0f, 0.0f)
    , isUpdated(false)
    , hasPath(false)
    , gefilterdAng(0.0f)
    , wallStaat(WallStaat::OPEN_RUIMTE)
    , wfGefilterdFout(0.0f)
{}


// ─────────────────────────────────────────────────────────────────
//  Waypoint-volger
// ─────────────────────────────────────────────────────────────────

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

    float absErr = std::fabs(angleErr);

    // Eerst draaien, dan rijden — bij grote hoekfout sta je stil
    // zodat je niet met volle snelheid de verkeerde kant op rijdt.
    if (absErr > 30.0f) {
        float draai = ANGULAR_GAIN * absErr;
        if (draai > MAX_ANGULAR_DEG_S) draai = MAX_ANGULAR_DEG_S;
        if (angleErr > 0.0f) draai = -draai;
        gefilterdAng = ANG_FILTER_ALFA * draai + (1.0f - ANG_FILTER_ALFA) * gefilterdAng;
        return DriveCommand(0.0f, gefilterdAng);  // stilstaan + draaien
    }

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

bool Navigator::IsFinished() const { return !hasPath || path.IsEmpty(); }
bool Navigator::IsUpdated()  const { return isUpdated; }
Position Navigator::GetCurrentTarget() const { return currentTarget; }
Path     Navigator::GetPath()          const { return path; }

bool Navigator::ReachedPoint(Position current) const {
    return !path.IsEmpty() &&
           CalculateDistance(current, currentTarget) < REACHED_THRESHOLD_MM;
}

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

float Navigator::NormalizeDeg(float deg) const { return NormDeg(deg); }


// ─────────────────────────────────────────────────────────────────
//  Muurvolger — hulpfuncties
// ─────────────────────────────────────────────────────────────────

float Navigator::WfSectorMin(const float ranges[360], int van, int tot) const {
    static constexpr float MAX_R = 8000.0f;
    float m = MAX_R;
    for (int a = van; a < tot; ++a) {
        int   idx = ((a % 360) + 360) % 360;
        float r   = ranges[idx];
        if (r > 0.0f && r < m) m = r;
    }
    return m;
}

float Navigator::WfSectorGem(const float ranges[360], int van, int tot) const {
    static constexpr float MAX_R = 8000.0f;
    float sum = 0.0f;
    int   cnt = 0;
    for (int a = van; a < tot; ++a) {
        int   idx = ((a % 360) + 360) % 360;
        float r   = ranges[idx];
        if (r > 0.0f && r < MAX_R) { sum += r; ++cnt; }
    }
    return cnt > 0 ? sum / static_cast<float>(cnt) : MAX_R;
}

void Navigator::ResetMuurvolger() {
    wallStaat       = WallStaat::OPEN_RUIMTE;
    wfGefilterdFout = 0.0f;
}


// ─────────────────────────────────────────────────────────────────
//  BerekenMuurCommando
// ─────────────────────────────────────────────────────────────────

WallResult Navigator::BerekenMuurCommando(const float ranges[360]) {

    // ── Sectoren lezen ────────────────────────────────────────────
    float minVoorSmal   = std::min(WfSectorMin(ranges, 350, 360),
                                    WfSectorMin(ranges,   0,  10));
    float minVoor       = std::min(WfSectorMin(ranges, 330, 360),
                                    WfSectorMin(ranges,   0,  30));
    float muurRechts    = WfSectorGem(ranges,  75, 105);
    float muurRechtsMin = WfSectorMin(ranges,  75, 105);
    float rechtsvoor    = WfSectorMin(ranges,  30,  75);

    (void)muurRechtsMin;

    bool muurVoor           = (minVoorSmal < WF_VOOR_KRITIEK_MM);
    bool muurVoorWaarsch    = (minVoor     < WF_VOOR_REMMEN_MM);
    bool rechterMuurAanwezig = (muurRechts  < WF_MUUR_AANWEZIG_MM);
    bool rechtsVoorVrij     = (rechtsvoor  > WF_MUUR_AANWEZIG_MM * 1.3f);

    // FIX: Initialize WallResult using a constructor-style list since DriveCommand lacks a default constructor.
    WallResult res{DriveCommand(0.0f, 0.0f), WallStaat::OPEN_RUIMTE, 0.0f};

    // ── GEVAL 1: Muur voor → binnenhoek, draai links ──────────────
    if (muurVoor) {
        wallStaat       = WallStaat::BINNENHOEK;
        wfGefilterdFout = 0.0f;

        res.cmd    = DriveCommand(0.0f, -WF_BINNENHOEK_DRAAI);
        res.staat  = WallStaat::BINNENHOEK;
        res.fout_mm = 0.0f;

        printf("[WALL] BINNENHOEK voor=%.0fmm → linksaf %.1f deg/s\n",
               minVoorSmal, -WF_BINNENHOEK_DRAAI);
        return res;
    }

    // ── GEVAL 2: Rechter muur weg + rechtsvoor open → buitenhoek ──
    if (!rechterMuurAanwezig && rechtsVoorVrij &&
        wallStaat != WallStaat::BUITENHOEK)
    {
        wallStaat       = WallStaat::BUITENHOEK;
        wfGefilterdFout = 0.0f;
        printf("[WALL] BUITENHOEK muurR=%.0fmm rechtsV=%.0fmm → rechtsaf\n",
               muurRechts, rechtsvoor);
    }

    if (wallStaat == WallStaat::BUITENHOEK) {
        if (rechterMuurAanwezig && muurRechts < WF_TARGET_DIST_MM * 1.5f) {
            wallStaat = WallStaat::RECHTS_VOLGEN;
            printf("[WALL] Muur teruggevonden op %.0fmm → RECHTS_VOLGEN\n", muurRechts);
        } else {
            res.cmd    = DriveCommand(WF_LIN_LANGZAAM, WF_BUITENHOEK_DRAAI);
            res.staat  = WallStaat::BUITENHOEK;
            res.fout_mm = 0.0f;
            return res;
        }
    }

    // ── GEVAL 3: Geen muur rechts, geen buitenhoek → open ruimte ──
    if (!rechterMuurAanwezig) {
        wallStaat = WallStaat::OPEN_RUIMTE;
        float lin = muurVoorWaarsch ? WF_LIN_LANGZAAM : WF_LIN_NORMAAL;
        printf("[WALL] OPEN_RUIMTE voor=%.0fmm rechts=%.0fmm\n", minVoor, muurRechts);
        res.cmd    = DriveCommand(lin, 0.0f);
        res.staat  = WallStaat::OPEN_RUIMTE;
        res.fout_mm = 0.0f;
        return res;
    }

    // ── GEVAL 4: Normale rechter muurvolging ─────────────────────
    wallStaat = WallStaat::RECHTS_VOLGEN;

    float ruweFout = muurRechts - WF_TARGET_DIST_MM;
    wfGefilterdFout = WF_EMA * ruweFout + (1.0f - WF_EMA) * wfGefilterdFout;

    float corrDegS = WF_KP * wfGefilterdFout;
    if (corrDegS >  WF_MAX_CORR) corrDegS =  WF_MAX_CORR;
    if (corrDegS < -WF_MAX_CORR) corrDegS = -WF_MAX_CORR;

    float lin = muurVoorWaarsch ? WF_LIN_LANGZAAM : WF_LIN_NORMAAL;

    printf("[WALL] RECHTS_VOLGEN muurR=%.0fmm fout=%.0fmm corr=%.1fdeg/s lin=%.0f\n",
           muurRechts, wfGefilterdFout, corrDegS, lin);

    res.cmd    = DriveCommand(lin, corrDegS);
    res.staat  = WallStaat::RECHTS_VOLGEN;
    res.fout_mm = wfGefilterdFout;
    return res;
}
