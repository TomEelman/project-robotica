#include "GridMap.h"

GridMap::GridMap(int w, int h)
    : width(w)
    , height(h)
    , grid(w, std::vector<int>(h, 0))  // Alles vrij initialiseren
    , updated(false)
    , valid(true)
{
}

bool GridMap::UpdateCell(int x, int y, bool occupied) {
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return false;
    }
    grid[x][y] = occupied ? 1 : 0;
    updated = true;
    return true;
}

bool GridMap::IsPathValid(Path path) {
    while (!path.IsEmpty()) {
        Position p = path.GetNextPoint();
        int x = static_cast<int>(p.GetX());
        int y = static_cast<int>(p.GetY());

        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        if (grid[x][y] == 1) return false;  // Geblokkeerde cel

        path.Advance();
    }
    return true;
}

const std::vector<std::vector<int>>& GridMap::GetGrid() const {
    return grid;
}