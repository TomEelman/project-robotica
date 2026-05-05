#include "Navigator.h"

#include <cmath>
#include <iostream>

Navigator::Navigator()
    : path()
    , currentTarget(0.0f, 0.0f, 0.0f)
    , isUpdated(false)
    , hasPath(false)
{
}

void Navigator::SetPath(const Path& newPath) {
    path = newPath;
    path.Reset();
    hasPath = !path.IsEmpty();
    if (hasPath) {
        currentTarget = path.GetCurrentWaypoint();
        std::cout << "Navigator: nieuw pad geladen, eerste waypoint ("
                  << currentTarget.GetX() << ", "
                  << currentTarget.GetY() << ")\n";
    }
    isUpdated = true;
}

void Navigator::Update(Position current) {
    if (!hasPath || path.IsEmpty()) return;

    while (ReachedPoint(current) && !path.IsEmpty()) {
        path.Advance();
        if (!path.IsEmpty()) {
            currentTarget = path.GetCurrentWaypoint();
            std::cout << "Navigator: volgend waypoint ("
                      << currentTarget.GetX() << ", "
                      << currentTarget.GetY() << ")\n";
        } else {
            std::cout << "Navigator: doel bereikt!\n";
            hasPath = false;
        }
    }

    isUpdated = true;
}

DriveCommand Navigator::GetNextCommand(Position current) {
    if (!hasPath || path.IsEmpty()) {
        return DriveCommand(0.0f, 0.0f);
    }

    float dist     = CalculateDistance(current, currentTarget);
    float angleErr = CalculateAngleError(current, currentTarget);  // radians

    if (dist < REACHED_THRESHOLD_MM) {
        return DriveCommand(0.0f, 0.0f);
    }

    // graden/s = gain * radians (gain heeft eenheid graden/s per radian)
    float angularDegS = ANGULAR_GAIN * angleErr;
    if (angularDegS >  MAX_ANGULAR_DEG_S) angularDegS =  MAX_ANGULAR_DEG_S;
    if (angularDegS < -MAX_ANGULAR_DEG_S) angularDegS = -MAX_ANGULAR_DEG_S;

    float linearMmS = LINEAR_SPEED_MM_S;
    // Bij grote hoekfout: eerst draaien, niet rijden
    if (std::fabs(angleErr) > ANGLE_THRESHOLD_RAD) {
        linearMmS = 0.0f;
    }

    return DriveCommand(linearMmS, angularDegS);
}

bool Navigator::ReachedPoint(Position current) const {
    if (path.IsEmpty()) return false;
    return CalculateDistance(current, currentTarget) < REACHED_THRESHOLD_MM;
}

bool Navigator::IsFinished() const { return !hasPath || path.IsEmpty(); }
bool Navigator::IsUpdated()  const { return isUpdated; }

Position Navigator::GetCurrentTarget() const { return currentTarget; }
Path     Navigator::GetPath()          const { return path; }

float Navigator::CalculateDistance(Position a, Position b) {
    float dx = b.GetX() - a.GetX();
    float dy = b.GetY() - a.GetY();
    return std::sqrt(dx * dx + dy * dy);
}

float Navigator::CalculateAngleError(Position current, Position target) {
    float dx = target.GetX() - current.GetX();
    float dy = target.GetY() - current.GetY();

    float desired = std::atan2(dy, dx);
    float err     = desired - current.GetTheta();
    return NormalizeRad(err);
}

float Navigator::NormalizeRad(float a) {
    const float PI = static_cast<float>(M_PI);
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}