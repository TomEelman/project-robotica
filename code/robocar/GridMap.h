#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include "Path.h"

class GridMap {
private:
    int width;
    int height;
    std::vector<std::vector<int>> grid;  // 0 = vrij, 1 = geblokkeerd
    bool updated;
    bool valid;

public:
    GridMap(int w, int h);

    bool UpdateCell(int x, int y, bool occupied);
    bool IsPathValid(Path path);

    // Geeft het interne grid terug voor de A* pathfinder
    const std::vector<std::vector<int>>& GetGrid() const;
};

#endif