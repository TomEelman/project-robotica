#ifndef EXPLORATIONPLANNER_H
#define EXPLORATIONPLANNER_H

#include "Position.h"
#include "GridMap.h"
#include "Mapper.h"

Position KiesFrontierDoel(const Mapper& mapper, const Position& huidig,
                           const float lidarRanges[360]);

int TelFrontiers(const Mapper& mapper);

enum ExplorationState {
    SEARCHING,
    MOVING,
    IDLE
};

class ExplorationPlanner {
public:
    ExplorationPlanner();

    Position         ComputeNextTarget(const GridMap& map, Position currentPos);
    Position         GetCurrentTarget() const;
    ExplorationState GetState()         const;
    bool             HasTarget()        const;

private:
    Position         currentTarget;
    ExplorationState state;
};

#endif