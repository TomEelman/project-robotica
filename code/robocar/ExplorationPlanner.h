#ifndef EXPLORATIONPLANNER_H
#define EXPLORATIONPLANNER_H

#include "Position.h"
#include "GridMap.h"

enum ExplorationState {
    SEARCHING,
    MOVING,
    IDLE
};

class ExplorationPlanner {

private:
    Position currentTarget;
    ExplorationState state;

public:
    ExplorationPlanner();

    Position ComputeNextTarget(GridMap map);

    Position GetCurrentTarget() const;

    ExplorationState GetState() const;
};

#endif