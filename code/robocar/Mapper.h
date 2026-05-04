#ifndef MAPPER_H
#define MAPPER_H

#include "Position.h"
#include "GridMap.h"
#include <string>

class Mapper {
public:
    Mapper(int width = 300, int height = 300, float resolution = 0.05f);

    void Update(const float ranges[], const float angles[],
                int count, const Position& position);
    void Update(const float scan360[], const Position& position);

    void PrintMap(float robotX, float robotY,
                  int scanCount, int coverage) const;

    int            GetCoverage()  const;
    GridMap&       GetMap();
    const GridMap& GetMap()       const;
    bool           SaveDebugMap(const std::string& filename) const;

private:
    GridMap map;
    bool    updated;
};

#endif // MAPPER_H