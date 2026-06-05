#ifndef MAPPER_H
#define MAPPER_H

#include "Position.h"
#include "GridMap.h"
#include "Path.h"
#include <string>

class Mapper {
public:
    Mapper(int width = 260, int height = 160, float resolution = 0.03f);

    void Update(const float ranges[], const float angles[], int count, const Position& position);

    void UpdateMotionCorrected(const float ranges[], const float angles[], int count, const Position& position,
        float omegaDegS, float scanDuurSec);

    void SetWaypoints(const Path& path);
                    
    void           GetRoomCoverage(float& outerWallPct, float& interiorPct, float& relCoveragePct) const;
    GridMap&       GetMap();
    const GridMap& GetMap()       const;
    const GridMap& GetGridMap() const;
    bool           SaveDebugMap(const std::string& filename) const;

private:
    GridMap map;
    bool    updated;
};

#endif