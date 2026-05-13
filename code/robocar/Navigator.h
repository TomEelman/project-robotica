#pragma once

#include "Path.h"
#include "Position.h"
#include "GridMap.h"
#include "DriveCommand.h"

class Navigator {
public:
    Navigator();

    void SetPath(const Path& newPath);
    void Update(Position current);
    DriveCommand GetNextCommand(Position current);

    bool IsFinished()  const;
    bool IsUpdated()   const;

    Position GetCurrentTarget() const;
    Path     GetPath()          const;

private:
    // Drempelafstand om een waypoint als bereikt te beschouwen.
    static constexpr float REACHED_THRESHOLD_MM = 200.0f;

    // Rijsnelheid — altijd vooruit, nooit stoppen voor draaien
    static constexpr float LINEAR_SPEED_MM_S    = 278.0f;

    // Proportionele gain voor zachte bijsturing (deg/s per graad fout)
    // Bij 45° fout → 45 × 0.18 ≈ 8 deg/s bijsturing
    static constexpr float ANGULAR_GAIN         = 0.18f;

    // Maximum bijsturing tijdens het rijden (graden/s)
    static constexpr float MAX_ANGULAR_DEG_S    = 25.0f;

    // Minimum bijsturing boven motor-stall drempel (graden/s)
    static constexpr float MIN_ANGULAR_DEG_S    = 5.0f;

    // Dode zone: onder dit geen bijsturing (graden)
    static constexpr float ANGLE_DEADBAND_DEG   = 3.0f;

    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;
};