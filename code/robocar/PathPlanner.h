#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "GridMap.h"
#include "Path.h"
#include "Position.h"

// PathPlanner doet A* over de GridMap. Cellen met state FREE zijn betreedbaar.
// UNKNOWN en OCCUPIED worden vermeden.
//
// Als je later WEL door UNKNOWN wilt plannen (handig voor exploration zodat
// de robot ergens heen kan binnen onontgonnen gebied), zet allowUnknown=true.
class PathPlanner {
private:
    GridMap gridMap;
    Path    currentPath;
    bool    allowUnknown;

public:
    explicit PathPlanner(const GridMap& map, bool allowUnknown = false);

    // Plant een pad van start naar goal. Beide in wereldcoordinaten (mm).
    // Returnt een lege Path als er geen pad is.
    Path PlanPath(Position start, Position goal, const GridMap& map);

    Path GetCurrentPath() const;

    // Update interne map-kopie (lidar heeft nieuwe info gegeven)
    void UpdateMap(const GridMap& map);
};

#endif