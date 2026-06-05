#ifndef PATH_H
#define PATH_H

#include <vector>
#include "Position.h"

class Path {
private:
    std::vector<Position> wayPoints;
    int                   currentIndex;

public:
    Path();
    explicit Path(const std::vector<Position>& points);

    Position GetCurrentWaypoint() const;

    Position PeekNextWaypoint() const;

    int  GetCurrentIndex() const;
    int  GetSize()         const;
    bool HasNext()         const;
    bool IsEmpty()         const;
    void Advance();
    void Reset();
};

#endif