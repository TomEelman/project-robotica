#ifndef PATH_H
#define PATH_H

#include "Position.h"

class Path {

private:
    Position* wayPoints;
    int currentIndex;
    int length;

public:
    Path(Position points[], int size);

    Position GetNextPoint();

    int GetCurrentIndex() const;

    bool HasNext() const;
};

#endif