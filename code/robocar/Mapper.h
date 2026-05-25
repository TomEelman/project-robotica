#ifndef MAPPER_H
#define MAPPER_H

#include "Position.h"
#include "GridMap.h"
#include "Path.h"
#include <string>

class Mapper {
public:
    Mapper(int width = 300, int height = 300, float resolution = 0.05f);

    void Update(const float ranges[], const float angles[],
                int count, const Position& position);
    void Update(const float scan360[], const Position& position);

    void PrintMap(float robotX, float robotY,
                  int scanCount, int coverage) const;

    // In Mapper.h, public sectie:
    void UpdateMotionCorrected(const float ranges[], const float angles[],
                    int count, const Position& position,
                    float omegaDegS, float scanDuurSec);

    // Sla de geplande waypoints op voor visualisatie in de PPM-export.
    // Aanroepen elke keer dat een nieuw pad berekend wordt.
    void SetWaypoints(const Path& path);
                    
    int            GetCoverage()  const;
    GridMap&       GetMap();
    const GridMap& GetMap()       const;
    const GridMap& GetGridMap() const;
    bool           SaveDebugMap(const std::string& filename) const;

private:
    GridMap map;
    bool    updated;
};

#endif // MAPPER_H