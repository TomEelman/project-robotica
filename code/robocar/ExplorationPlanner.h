#ifndef EXPLORATIONPLANNER_H
#define EXPLORATIONPLANNER_H

#include "Position.h"
#include "GridMap.h"
#include "Mapper.h"
#include <vector>

struct BlacklistItem {
    float x_mm; // worldcoordinates in mm
    float y_mm;  
    int   ttl; // time to live
};

void     AddToBlackList(std::vector<BlacklistItem>& blacklist, float x_mm, float y_mm, int ttl = 5);
void     TickBlacklist(std::vector<BlacklistItem>& blacklist);
Position ChooseFrontierGoal(const Mapper& mapper, const Position& huidig, const float lidarRanges[360],
                            const std::vector<BlacklistItem>& blacklist);

class ExplorationPlanner {
public:
    ExplorationPlanner();
    void AddToBlackList(std::vector<BlacklistItem>& blacklist, float x_mm, float y_mm, int ttl = 5);
    void TickBlacklist(std::vector<BlacklistItem>& blacklist);
    Position ChooseFrontierGoal(const Mapper& mapper, const Position& huidig, const float lidarRanges[360],
                        const std::vector<BlacklistItem>& blacklist = {});
private:
    Position currentTarget;
};

#endif