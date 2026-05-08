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
    // Groot genoeg zodat hij doorrijdt, klein genoeg voor goede bochten.
    static constexpr float REACHED_THRESHOLD_MM = 150.0f;

    // Rijsnelheid — zelfde als de handmatige modus
    static constexpr float LINEAR_SPEED_MM_S    = 278.0f;

    // Hoekversterkig: hoekfout (rad) × gain = deg/s draaisnelheid
    static constexpr float ANGULAR_GAIN         = 120.0f;

    // Maximum draaisnelheid
    static constexpr float MAX_ANGULAR_DEG_S    = 90.0f;

    // Hoekfout waarbij we beginnen af te remmen (in radialen).
    // 20 graden is een goede startwaarde — verhoog als je nog overshoot hebt,
    // verlaag als hij te vroeg afzwakt en traag naar de heading kruipt.
    static constexpr float PI             = 3.14159265358979f;
    static constexpr float BRAKE_ZONE_RAD = 20.0f * (PI / 180.0f);


    // Minimale angular velocity in de remzone — moet groot genoeg zijn om de
    // Pico-motors nog te laten bewegen (boven stall), maar klein genoeg om
    // de overshoot te beperken.
    static constexpr float MIN_ANGULAR_DEG_S = 15.0f;

    // Pas als de hoekfout groter is dan dit stopt hij met vooruitrijden
    // 0.25 rad ≈ 14° — snel genoeg voor soepele bochten
    static constexpr float ANGLE_THRESHOLD_RAD  = 0.25f;

    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    bool  ReachedPoint(Position current) const;
    float CalculateDistance (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target);
    float NormalizeRad(float a);
};