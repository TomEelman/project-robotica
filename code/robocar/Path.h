#ifndef PATH_H
#define PATH_H

#include <vector>
#include "Position.h"

// Path bezit zijn eigen waypoints (geen raw pointer meer).
// Waypoints zijn in wereldcoordinaten (mm).
class Path {
private:
    std::vector<Position> wayPoints;
    int                   currentIndex;

public:
    Path();
    explicit Path(const std::vector<Position>& points);

    // Huidige (= eerstvolgende nog te bereiken) waypoint.
    Position GetCurrentWaypoint() const;

    // Het waypoint NA de huidige (voor look-ahead). Als er geen volgende is,
    // returnt hij het huidige waypoint.
    Position PeekNextWaypoint() const;

    int  GetCurrentIndex() const;
    int  GetSize()         const;
    bool HasNext()         const;
    bool IsEmpty()         const;
    void Advance();
    void Reset();
};

#endif