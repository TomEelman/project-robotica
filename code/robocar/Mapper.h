#ifndef MAPPER_H
#define MAPPER_H

#include "Position.h"
#include "GridMap.h"

class Mapper {
private:
    int width;
    int height;
    bool updated;

public:
    void Mapping();
    void Update(float scan[], Position position);
    int GetCoverage();
    GridMap GetMap();
};

#endif