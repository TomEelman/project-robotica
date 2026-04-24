#ifndef PATH_H
#define PATH_H

#include "Position.h"

class Path {
private:
    Position* wayPoints;
    int       currentIndex;
    int       length;

public:
    Path(Position points[], int size);

    Position GetNextPoint() const;
    int      GetCurrentIndex() const;
    bool     HasNext() const;

    // Stap naar het volgende waypoint
    void Advance();

    // True als er geen waypoints meer zijn
    bool IsEmpty() const;
};

#endif