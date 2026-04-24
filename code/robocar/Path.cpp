#include "Path.h"

Path::Path(Position points[], int size)
    : wayPoints(points)
    , currentIndex(0)
    , length(size)
{
}

Position Path::GetNextPoint() const {
    return wayPoints[currentIndex];
}

int Path::GetCurrentIndex() const {
    return currentIndex;
}

bool Path::HasNext() const {
    return currentIndex + 1 < length;
}

void Path::Advance() {
    if (!IsEmpty()) {
        currentIndex++;
    }
}

bool Path::IsEmpty() const {
    return currentIndex >= length;
}