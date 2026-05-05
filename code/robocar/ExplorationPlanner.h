#ifndef EXPLORATIONPLANNER_H
#define EXPLORATIONPLANNER_H

#include "Position.h"
#include "GridMap.h"

enum ExplorationState {
    SEARCHING,   // op zoek naar volgende frontier
    MOVING,      // target gekozen, robot rijdt erheen
    IDLE         // niets meer te exploreren
};

// Frontier exploration: zoekt FREE cellen die naast minstens een UNKNOWN cel
// liggen. Kiest de dichtstbijzijnde frontier (vanaf currentPos) als nieuw doel.
class ExplorationPlanner {
private:
    Position         currentTarget;
    ExplorationState state;

public:
    ExplorationPlanner();

    // Kies een nieuw target op basis van de map en huidige positie.
    // Returnt een Position. Check getState() == IDLE om te zien of er nog iets
    // te doen is (anders is currentTarget niet zinvol).
    Position ComputeNextTarget(const GridMap& map, Position currentPos);

    Position         GetCurrentTarget() const;
    ExplorationState GetState()         const;
    bool             HasTarget()        const;
};

#endif