#include "ExplorationPlanner.h"

#include <cmath>
#include <vector>
#include <iostream>

ExplorationPlanner::ExplorationPlanner()
    : currentTarget(0.0f, 0.0f, 0.0f)
    , state(SEARCHING)
{
}

Position ExplorationPlanner::ComputeNextTarget(const GridMap& map, Position currentPos) {
    state = SEARCHING;

    int W = map.GetWidth();
    int H = map.GetHeight();

    // Robot's huidige cel (voor afstand-vergelijk)
    int rx, ry;
    bool robotInGrid = map.WorldToGrid(currentPos.GetX(), currentPos.GetY(), rx, ry);

    int bestX = -1, bestY = -1;
    int bestDistSq = -1;

    // Frontier definitie: cel is FREE en heeft minstens een UNKNOWN buur
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = { 0, 0,-1, 1};

    for (int x = 0; x < W; ++x) {
        for (int y = 0; y < H; ++y) {
            if (!map.IsFree(x, y)) continue;

            bool isFrontier = false;
            for (int i = 0; i < 4; ++i) {
                int nx = x + dx[i];
                int ny = y + dy[i];
                if (map.IsUnknown(nx, ny)) { isFrontier = true; break; }
            }
            if (!isFrontier) continue;

            // Pak dichtstbijzijnde frontier
            int distSq;
            if (robotInGrid) {
                int ddx = x - rx;
                int ddy = y - ry;
                distSq = ddx * ddx + ddy * ddy;
            } else {
                distSq = x * x + y * y;
            }

            if (bestDistSq < 0 || distSq < bestDistSq) {
                bestDistSq = distSq;
                bestX = x;
                bestY = y;
            }
        }
    }

    if (bestX < 0) {
        // Geen frontiers meer - klaar
        state = IDLE;
        std::cout << "ExplorationPlanner: geen frontiers meer, IDLE\n";
        return currentTarget;
    }

    float wx, wy;
    map.GridToWorld(bestX, bestY, wx, wy);
    currentTarget = Position(wx, wy, 0.0f);
    state = MOVING;

    std::cout << "ExplorationPlanner: nieuw target cel (" << bestX << ", " << bestY
              << ") = wereld (" << wx << ", " << wy << ")\n";
    return currentTarget;
}

Position         ExplorationPlanner::GetCurrentTarget() const { return currentTarget; }
ExplorationState ExplorationPlanner::GetState()         const { return state; }
bool             ExplorationPlanner::HasTarget()        const { return state == MOVING; }