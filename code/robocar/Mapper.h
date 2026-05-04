#ifndef MAPPER_H
#define MAPPER_H

#include "Position.h"
#include "GridMap.h"

// ═══════════════════════════════════════════════════════════════
//  Mapper  –  verbindt LIDAR-scans met de GridMap
//
//  Gebruik:
//    Mapper mapper(400, 400, 0.05f);          // 20×20 m bij 5 cm/cel
//    mapper.Update(ranges, angles, 360, pos); // na elke LIDAR-scan
//    int pct = mapper.GetCoverage();          // kaartdekking in %
//    GridMap& map = mapper.GetMap();          // voor A* planner
// ═══════════════════════════════════════════════════════════════

class Mapper {
public:
    // width/height in cellen; resolution in meter/cel (bv. 0.05)
    Mapper(int width = 400, int height = 400, float resolution = 0.05f);

    // Verwerk een volledige LIDAR-scan
    // ranges[]  in mm   (0 = geen meting)
    // angles[]  in graden (relatief aan robot)
    // count     = aantal elementen (bv. 360)
    // position  = huidige robotpositie (meter) + oriëntatie (rad)
    void Update(const float ranges[], const float angles[],
                int count, const Position& position);

    // Verwerk een LIDAR-scan waarbij de hoeken impliciet 0..359° zijn
    void Update(const float scan360[], const Position& position);

    // Percentage van het grid dat al verkend is (vrij+bezet / totaal)
    int GetCoverage() const;

    // Geeft de huidige kaart terug (voor planners en visualisatie)
    GridMap& GetMap();
    const GridMap& GetMap() const;

    // Sla de kaart op als PGM-debug-afbeelding
    bool SaveDebugMap(const std::string& filename) const;

private:
    GridMap map;
    bool    updated;
};

#endif // MAPPER_H