#pragma once

#include "Path.h"
#include "Position.h"
#include "GridMap.h"
#include "DriveCommand.h"

// ─────────────────────────────────────────────────────────────────
//  ScanAnalyse — resultaat van een 360° LIDAR-scan
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
//  WallStaat
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
    float        fout_mm;
};


// ─────────────────────────────────────────────────────────────────
//  Navigator
// ─────────────────────────────────────────────────────────────────
class Navigator {
public:
    Navigator();

    // ── Waypoint-volger ───────────────────────────────────────────
    void         SetPath(const Path& newPath);
    void         Update(Position current);
    DriveCommand GetNextCommand(Position current, float minVoor = 8000.0f);
    bool         IsFinished() const;
    bool         IsUpdated()  const;
    Position     GetCurrentTarget() const;
    Path         GetPath()          const;

    // ── Muurvolger ────────────────────────────────────────────────
    WallResult BerekenMuurCommando(const float ranges[360]);
    void       ResetMuurvolger();
    WallStaat  GetWallStaat() const { return wallStaat; }

private:
    // ── Waypoint-volger constanten ────────────────────────────────
    static constexpr float REACHED_THRESHOLD_MM  = 150.0f;
    static constexpr float LINEAR_SPEED_MM_S     = 278.0f;
    static constexpr float ANGULAR_GAIN          = 2.5f;
    static constexpr float MAX_ANGULAR_DEG_S     = 45.0f;
    static constexpr float ANGLE_DEADBAND_DEG    = 12.0f;
    static constexpr float SLOW_TURN_THRESHOLD   = 25.0f;
    static constexpr float SLOW_TURN_FACTOR      = 0.6f;

    // Stabiel commando: herbereken pas na N ticks of bij obstakel
    static constexpr int   CMD_STABIEL_TICKS     = 20;     // ~2 sec bij 10Hz
    static constexpr float OBSTAKEL_INTERRUPT_MM = 400.0f; // interrupt-drempel

    // ── Muurvolger constanten ─────────────────────────────────────
    static constexpr float WF_TARGET_DIST_MM     = 300.0f;
    static constexpr float WF_MUUR_AANWEZIG_MM   = 600.0f;
    static constexpr float WF_VOOR_KRITIEK_MM    = 350.0f;
    static constexpr float WF_VOOR_REMMEN_MM     = 500.0f;
    static constexpr float WF_LIN_NORMAAL        = 250.0f;
    static constexpr float WF_LIN_LANGZAAM       = 150.0f;
    static constexpr float WF_KP                 = 0.08f;
    static constexpr float WF_MAX_CORR           = 30.0f;
    static constexpr float WF_MIN_CORR           = 3.0f;
    static constexpr float WF_EMA                = 0.25f;
    static constexpr float WF_BUITENHOEK_DRAAI   = 25.0f;
    static constexpr float WF_BINNENHOEK_DRAAI   = 35.0f;
    static constexpr int   WF_BUITENHOEK_MAX_TICKS = 20;

    // ── Waypoint-volger state ─────────────────────────────────────
    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    // Stabiel commando state
    float stabielLin      = 0.0f;
    float stabielAng      = 0.0f;
    bool  heeftStabielCmd = false;
    int   cmdTicks        = 0;

    // ── Muurvolger state ──────────────────────────────────────────
    WallStaat wallStaat       = WallStaat::OPEN_RUIMTE;
    float     wfGefilterdFout = 0.0f;
    int       buitenhoekTicks = 0;

    // ── Hulpfuncties ──────────────────────────────────────────────
    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;
    float WfSectorMin(const float ranges[360], int van, int tot) const;
    float WfSectorGem(const float ranges[360], int van, int tot) const;
};