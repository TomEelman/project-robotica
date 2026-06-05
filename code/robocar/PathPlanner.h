#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "GridMap.h"
#include "Path.h"
#include "Position.h"

class PathPlanner {
private:
    GridMap gridMap;
    Path    currentPath;
    bool    allowUnknown;

public:
    explicit PathPlanner(const GridMap& map, bool allowUnknown = false);

    Path PlanPath(Position start, Position goal, const GridMap& map);

    Path GetCurrentPath() const;

    void UpdateMap(const GridMap& map);

    void SetAllowUnknown(bool v) { allowUnknown = v; }

private:
    static constexpr int INFLATION_CELLS = 2;
};

#endif