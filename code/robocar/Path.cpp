#include "Path.h"

Path::Path()
    : wayPoints()
    , currentIndex(0)
{
}

Path::Path(const std::vector<Position>& points)
    : wayPoints(points)
    , currentIndex(0)
{
}

Position Path::GetCurrentWaypoint() const {
    if (IsEmpty()) return Position(0.0f, 0.0f, 0.0f);
    return wayPoints[currentIndex];
}

Position Path::PeekNextWaypoint() const {
    if (IsEmpty()) return Position(0.0f, 0.0f, 0.0f);
    int idx = currentIndex + 1;
    if (idx >= static_cast<int>(wayPoints.size())) idx = currentIndex;
    return wayPoints[idx];
}

int  Path::GetCurrentIndex() const { return currentIndex; }
int  Path::GetSize()         const { return static_cast<int>(wayPoints.size()); }
bool Path::HasNext()         const { return currentIndex + 1 < static_cast<int>(wayPoints.size()); }
bool Path::IsEmpty()         const { return currentIndex >= static_cast<int>(wayPoints.size()); }

void Path::Advance() {
    if (!IsEmpty()) currentIndex++;
}

void Path::Reset() {
    currentIndex = 0;
}