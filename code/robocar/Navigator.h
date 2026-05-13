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
    static constexpr float REACHED_THRESHOLD_MM = 150.0f;

    // Rijsnelheid
    static constexpr float LINEAR_SPEED_MM_S    = 278.0f;

    // Maximum draaisnelheid (graden/s)
    static constexpr float MAX_ANGULAR_DEG_S    = 90.0f;

    // Minimale draaisnelheid in de remzone (graden/s)
    static constexpr float MIN_ANGULAR_DEG_S    = 15.0f;

    // Remzone: begin met afschalen binnen deze hoekfout (graden)
    static constexpr float BRAKE_ZONE_DEG       = 20.0f;

    // Stop vooruitrijden als hoekfout groter is dan dit (graden)
    // 0.25 rad ≈ 14.3°
    static constexpr float ANGLE_THRESHOLD_DEG  = 14.3f;

    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;
};