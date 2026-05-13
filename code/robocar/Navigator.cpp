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
    if (!hasPath || path.IsEmpty())
        return DriveCommand(0.0f, 0.0f);

    float dist     = CalculateDistance(current, currentTarget);
    float angleErr = CalculateAngleError(current, currentTarget); // graden [-180, 180]

    if (dist < REACHED_THRESHOLD_MM)
        return DriveCommand(LINEAR_SPEED_MM_S, 0.0f);  // doel bijna bereikt: rijd door

    float absErr = std::fabs(angleErr);

    // Dode zone: kleine afwijkingen negeren, rijdt dan puur rechtdoor
    // Tekenconventie Pico-hardware (geverifieerd handmatig):
    //   ang > 0 → rechts draaien,  ang < 0 → links draaien
    // atan2 geeft positieve hoekfout als doel LINKS ligt van huidige kijkrichting.
    // Dus: positieve hoekfout → negatieve angular (links draaien).
    float angularDegS = 0.0f;
    if (absErr > ANGLE_DEADBAND_DEG) {
        angularDegS = ANGULAR_GAIN * absErr;
        if (angularDegS < MIN_ANGULAR_DEG_S) angularDegS = MIN_ANGULAR_DEG_S;
        if (angularDegS > MAX_ANGULAR_DEG_S) angularDegS = MAX_ANGULAR_DEG_S;
        // Positieve hoekfout = doel is links → ang negatief (links)
        // Negatieve hoekfout = doel is rechts → ang positief (rechts)
        if (angleErr > 0.0f) angularDegS = -angularDegS;
    }

    // Altijd vooruit rijden — Drive::ExecuteLinear verwerkt angular als zachte bocht
    return DriveCommand(LINEAR_SPEED_MM_S, angularDegS);
}

bool Navigator::ReachedPoint(Position current) const {
    if (path.IsEmpty()) return false;
    return CalculateDistance(current, currentTarget) < REACHED_THRESHOLD_MM;
}

bool Navigator::IsFinished() const { return !hasPath || path.IsEmpty(); }
bool Navigator::IsUpdated()  const { return isUpdated; }

Position Navigator::GetCurrentTarget() const { return currentTarget; }
Path     Navigator::GetPath()          const { return path; }

float Navigator::CalculateDistance(Position a, Position b) const {
    float dx = b.GetX() - a.GetX();
    float dy = b.GetY() - a.GetY();
    return std::sqrt(dx * dx + dy * dy);
}

float Navigator::CalculateAngleError(Position current, Position target) const {
    float dx      = target.GetX() - current.GetX();
    float dy      = target.GetY() - current.GetY();
    float desired = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));
    float theta   = current.GetTheta(); // graden (geleverd door Localisation::GetTheta)
    return NormalizeDeg(desired - theta);
}

float Navigator::NormalizeDeg(float deg) const {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}