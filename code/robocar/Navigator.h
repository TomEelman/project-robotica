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

    // Rijsnelheid bij rechte koers
    static constexpr float LINEAR_SPEED_MM_S    = 278.0f;

    // Proportionele gain voor bijsturing (deg/s per graad fout).
    // Klein gehouden zodat kleine hoekfouten ook kleine correcties geven
    // en de robot niet oscilleert. Bij 20° fout → 20 × 0.10 = 2 deg/s.
    static constexpr float ANGULAR_GAIN         = 0.10f;

    // Maximum bijsturing tijdens het rijden (graden/s).
    // Lager dan voorheen zodat bochten geleidelijker gaan.
    static constexpr float MAX_ANGULAR_DEG_S    = 15.0f;

    // Geen minimum bijsturing — dat veroorzaakte overshoot bij kleine fouten.
    // De gain is nu puur proportioneel: kleine fout → kleine correctie.
    static constexpr float MIN_ANGULAR_DEG_S    = 0.0f;

    // Dode zone: groter dan voorheen zodat de robot kleine drift negeert
    // en niet constant heen-en-weer stuurt langs een rechte lijn.
    static constexpr float ANGLE_DEADBAND_DEG   = 8.0f;

    // Bij grote hoekfout (>30°) rijdt de robot langzamer zodat hij niet
    // in een bocht slipt. Snelheidsreductie tot 50% bij 45° fout.
    static constexpr float SLOW_TURN_THRESHOLD  = 30.0f;  // graden
    static constexpr float SLOW_TURN_FACTOR     = 0.55f;  // fractie van LINEAR_SPEED

    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;
};