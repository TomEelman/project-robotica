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
    static constexpr float REACHED_THRESHOLD_MM = 100.0f;
    static constexpr float LINEAR_SPEED_MM_S    = 200.0f;
    static constexpr float ANGULAR_GAIN         = 90.0f;
    static constexpr float MAX_ANGULAR_DEG_S    = 120.0f;
    static constexpr float ANGLE_THRESHOLD_RAD  = 0.3f;

    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    bool  ReachedPoint(Position current) const;
    float CalculateDistance (Position a, Position b);
    float CalculateAngleError(Position current, Position target);
    float NormalizeRad(float a);
};