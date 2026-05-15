#ifndef EXPLORATIONPLANNER_H
#define EXPLORATIONPLANNER_H

#include "Position.h"
#include "GridMap.h"
#include "Mapper.h"
#include <vector>

// ─────────────────────────────────────────────────────────────────
//  KiesFrontierDoel
//
//  Kiest de beste frontier op basis van:
//    0.5 × bereikbaarheid (A*-pad lengte, korter = beter)
//    0.3 × LIDAR-ruimte in die richting
//    0.2 × afstand (dichterbij = iets beter — voorkomt verre onbereikbare doelen)
//
//  Frontiers in de blacklist worden overgeslagen.
//  Blacklist-items verlopen na BLACKLIST_TTL herplanmomenten.
// ─────────────────────────────────────────────────────────────────

struct BlacklistItem {
    float x_mm, y_mm;  // wereldcoordinaten in mm
    int   ttl;          // resterende levensduur (herplanmomenten)
};

Position KiesFrontierDoel(const Mapper& mapper,
                           const Position& huidig,
                           const float lidarRanges[360],
                           const std::vector<BlacklistItem>& blacklist = {});

// Voeg een gefaald doel toe aan de blacklist
void VoegToeAanBlacklist(std::vector<BlacklistItem>& blacklist,
                          float x_mm, float y_mm,
                          int ttl = 5);

// Verlaag TTL van alle items, verwijder verlopen items
void TickBlacklist(std::vector<BlacklistItem>& blacklist);

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