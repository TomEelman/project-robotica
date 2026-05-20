#pragma once

#include "Path.h"
#include "Position.h"
#include "GridMap.h"
#include "DriveCommand.h"

// ─────────────────────────────────────────────────────────────────
//  ScanAnalyse — resultaat van een 360° LIDAR-scan
//
//  staat:       0=vrij  1=veilig  2=remmen  3=kritiek
//  uitwijkHoek: +1=rechts uitwijken  -1=links uitwijken
//  minAchter:   minimale ruimte achter de robot (sector 150°–210°)
// ─────────────────────────────────────────────────────────────────
struct ScanAnalyse {
    int   staat;
    float uitwijkHoek;
    float ruimteLinks;
    float ruimteRechts;
    float minVoor;
    float minLinks;
    float minRechts;
    float minAchter;
};

ScanAnalyse AnalyseerScan(const float ranges[360]);
float NormDeg(float deg);


// ─────────────────────────────────────────────────────────────────
//  WallStaat — toestand van de muurvolger
//
//  RECHTS_VOLGEN : rijdt langs de rechter muur op streefafstand
//  BUITENHOEK    : rechter muur verdween → boog naar rechts
//  BINNENHOEK    : muur voor → stop + draai links
//  OPEN_RUIMTE   : geen muren zichtbaar → hoofdlus moet frontier kiezen
// ─────────────────────────────────────────────────────────────────
enum class WallStaat {
    RECHTS_VOLGEN,
    BUITENHOEK,
    BINNENHOEK,
    OPEN_RUIMTE,
};

struct WallResult {
    DriveCommand cmd;
    WallStaat    staat;
    float        fout_mm;  // afwijking van streefafstand (debug)
};


// ─────────────────────────────────────────────────────────────────
//  Navigator — waypoint-volger + rechterhandmuurvolger
// ─────────────────────────────────────────────────────────────────
class Navigator {
public:
    Navigator();

    // ── Waypoint-volger (ongewijzigd) ─────────────────────────────
    void         SetPath(const Path& newPath);
    void         Update(Position current);
    DriveCommand GetNextCommand(Position current);
    bool         IsFinished() const;
    bool         IsUpdated()  const;
    Position     GetCurrentTarget() const;
    Path         GetPath()          const;

    // ── Muurvolger ────────────────────────────────────────────────
    // Geeft één rij-commando op basis van de huidige LIDAR-scan.
    // Roep elke lus-iteratie aan in plaats van GetNextCommand wanneer
    // je in muurvolgmodus zit. Geen pad nodig.
    //
    // ranges[]:  robot-relatief, 0=voor, 90=rechts, 270=links
    WallResult BerekenMuurCommando(const float ranges[360]);

    // Reset de interne muurvolger-state (bijv. bij overschakelen van modus)
    void ResetMuurvolger();

    WallStaat GetWallStaat() const { return wallStaat; }

private:
        // ── Waypoint-volger constanten ────────────────────────────────
    static constexpr float REACHED_THRESHOLD_MM = 150.0f;
    static constexpr float LINEAR_SPEED_MM_S    = 278.0f;
    static constexpr float ANGULAR_GAIN         = 1.5f;   // was 2.5 → minder agressief
    static constexpr float MAX_ANGULAR_DEG_S    = 25.0f;  // was 45 → zachter bijsturen
    static constexpr float MIN_ANGULAR_DEG_S    = 0.0f;
    static constexpr float ANGLE_DEADBAND_DEG   = 5.0f;
    static constexpr float SLOW_TURN_THRESHOLD  = 15.0f;  // was 25 → eerder afremmen
    static constexpr float SLOW_TURN_FACTOR     = 0.5f;   // was 0.6 → iets langzamer in bocht
    static constexpr float ANG_FILTER_ALFA      = 0.3f;   // was 0.4 → trager filter, minder zwaaien
        // ── Muurvolger constanten ─────────────────────────────────────
    static constexpr float WF_TARGET_DIST_MM    = 300.0f;  // streefafstand rechter muur
    static constexpr float WF_MUUR_AANWEZIG_MM  = 600.0f;  // max afstand om muur "aanwezig" te noemen
    static constexpr float WF_VOOR_KRITIEK_MM   = 350.0f;  // stop + draai links
    static constexpr float WF_VOOR_REMMEN_MM    = 500.0f;  // langzamer
    static constexpr float WF_LIN_NORMAAL       = 250.0f;  // rijsnelheid normaal (mm/s)
    static constexpr float WF_LIN_LANGZAAM      = 150.0f;  // rijsnelheid bij bochten
    static constexpr float WF_KP                = 0.08f;   // P-gain muurfout → hoeksnelheid
    static constexpr float WF_MAX_CORR          = 30.0f;   // max bijsturing (deg/s)
    static constexpr float WF_EMA               = 0.25f;   // EMA-filter op muurfout
    static constexpr float WF_BUITENHOEK_DRAAI  = 25.0f;   // hoeksnelheid buitenhoek (deg/s)
    static constexpr float WF_BINNENHOEK_DRAAI  = 35.0f;   // hoeksnelheid binnenhoek (deg/s)

    // ── Waypoint-volger state ─────────────────────────────────────
    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;
    float    gefilterdAng = 0.0f;

    // ── Muurvolger state ──────────────────────────────────────────
    WallStaat wallStaat      = WallStaat::OPEN_RUIMTE;
    float     wfGefilterdFout = 0.0f;

    // ── Gedeelde hulpfuncties ─────────────────────────────────────
    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;

    // ── Muurvolger hulpfuncties ───────────────────────────────────
    float WfSectorMin(const float ranges[360], int van, int tot) const;
    float WfSectorGem(const float ranges[360], int van, int tot) const;
};
