#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include "Path.h"

class GridMap {
private:
    int width;
    int height;
    std::vector<std::vector<int>> grid; // 2D-Matrix
    bool updated;
    bool valid;

public:
    GridMap(int w, int h);
    bool UpdateCell(int x, int y, bool occupied);
    bool IsPathValid(Path path);
};

#endif