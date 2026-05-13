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

// Analyseer een 360° LIDAR-scan. Indices zijn robot-relatief:
// 0 = recht voor, 90 = rechts, 270 = links.
ScanAnalyse AnalyseerScan(const float ranges[360]);

// Normaliseer graden naar (−180, 180].
float NormDeg(float deg);


// ─────────────────────────────────────────────────────────────────
//  Navigator — volgt een pad van waypoints
// ─────────────────────────────────────────────────────────────────
class Navigator {
public:
    Navigator();

    void SetPath(const Path& newPath);
    void Update(Position current);
    DriveCommand GetNextCommand(Position current);

    bool IsFinished() const;
    bool IsUpdated()  const;

    Position GetCurrentTarget() const;
    Path     GetPath()          const;

private:
    static constexpr float REACHED_THRESHOLD_MM = 200.0f;
    static constexpr float LINEAR_SPEED_MM_S    = 278.0f;
    static constexpr float ANGULAR_GAIN         = 0.10f;
    static constexpr float MAX_ANGULAR_DEG_S    = 15.0f;
    static constexpr float MIN_ANGULAR_DEG_S    = 0.0f;
    static constexpr float ANGLE_DEADBAND_DEG   = 8.0f;
    static constexpr float SLOW_TURN_THRESHOLD  = 30.0f;
    static constexpr float SLOW_TURN_FACTOR     = 0.55f;
    static constexpr float ANG_FILTER_ALFA      = 0.15f;

    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;
    float    gefilterdAng = 0.0f;

    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;
};