#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "GridMap.h"
#include "Path.h"
#include "Position.h"

class PathPlanner {

private:
    GridMap gridMap;
    Path currentPath;

public:
    PathPlanner(GridMap map);

    Path PlanPath(Position Start, Position Goal, const GridMap& map);

    Path GetCurrentPath() const;
};

#endif