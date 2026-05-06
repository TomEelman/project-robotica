#include "PathPlanner.h"

#include <queue>
#include <vector>
#include <climits>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace {
struct AStarNode {
    int x, y, g, h, f;
    AStarNode(int _x=0, int _y=0, int _g=0, int _h=0)
        : x(_x), y(_y), g(_g), h(_h), f(_g+_h) {}
    bool operator>(const AStarNode& o) const { return f > o.f; }
};
}

PathPlanner::PathPlanner(const GridMap& map, bool allowUnknownIn)
    : gridMap(map), currentPath(), allowUnknown(allowUnknownIn)
{
}

void PathPlanner::UpdateMap(const GridMap& map) {
    gridMap = map;
}

Path PathPlanner::PlanPath(Position start, Position goal, const GridMap& map) {
    gridMap = map;

    int sx, sy, gx, gy;

    // WorldToGrid → WorldToCell (geeft void, InBounds voor de check)
    gridMap.WorldToCell(start.GetX(), start.GetY(), sx, sy);
    if (!gridMap.InBounds(sx, sy)) {
        std::cerr << "PathPlanner: start buiten grid\n";
        return Path();
    }

    gridMap.WorldToCell(goal.GetX(), goal.GetY(), gx, gy);
    if (!gridMap.InBounds(gx, gy)) {
        std::cerr << "PathPlanner: goal buiten grid\n";
        return Path();
    }

    // GetCell + Cell::OCCUPIED/UNKNOWN → IsOccupied / IsUnknown / IsFree
    auto walkable = [&](int x, int y) -> bool {
        if (!gridMap.InBounds(x, y))    return false;
        if (gridMap.IsOccupied(x, y))   return false;
        if (gridMap.IsUnknown(x, y) && !allowUnknown) return false;
        return true;
    };

    if (!walkable(sx, sy)) {
        std::cerr << "PathPlanner: start cel niet betreedbaar\n";
        return Path();
    }
    if (!walkable(gx, gy)) {
        std::cerr << "PathPlanner: goal cel niet betreedbaar\n";
        return Path();
    }

    const int W = gridMap.GetWidth();
    const int H = gridMap.GetHeight();

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    std::vector<std::vector<bool>> closed (W, std::vector<bool>(H, false));
    std::vector<std::vector<int>>  gScore (W, std::vector<int> (H, INT_MAX));
    std::vector<std::vector<int>>  parentX(W, std::vector<int> (H, -1));
    std::vector<std::vector<int>>  parentY(W, std::vector<int> (H, -1));

    const int dx[4] = {-1, 1,  0, 0};
    const int dy[4] = { 0, 0, -1, 1};

    auto heuristic = [&](int x, int y) {
        return std::abs(x - gx) + std::abs(y - gy);
    };

    gScore[sx][sy] = 0;
    open.emplace(sx, sy, 0, heuristic(sx, sy));

    bool found = false;
    while (!open.empty()) {
        AStarNode cur = open.top(); open.pop();
        if (closed[cur.x][cur.y]) continue;
        closed[cur.x][cur.y] = true;

        if (cur.x == gx && cur.y == gy) { found = true; break; }

        for (int i = 0; i < 4; ++i) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];
            if (!walkable(nx, ny) || closed[nx][ny]) continue;

            int newG = gScore[cur.x][cur.y] + 1;
            if (newG < gScore[nx][ny]) {
                gScore[nx][ny]  = newG;
                parentX[nx][ny] = cur.x;
                parentY[nx][ny] = cur.y;
                open.emplace(nx, ny, newG, heuristic(nx, ny));
            }
        }
    }

    if (!found) {
        std::cerr << "PathPlanner: geen pad gevonden\n";
        return Path();
    }

    // Reconstrueer pad van goal → start, dan omkeren
    std::vector<Position> waypoints;
    int cx = gx, cy = gy;
    while (!(cx == sx && cy == sy)) {
        float wx, wy;
        gridMap.CellToWorld(cx, cy, wx, wy);   // GridToWorld → CellToWorld
        waypoints.emplace_back(wx, wy, 0.0f);
        int px = parentX[cx][cy];
        int py = parentY[cx][cy];
        if (px < 0 || py < 0) break;
        cx = px; cy = py;
    }
    {
        float wx, wy;
        gridMap.CellToWorld(sx, sy, wx, wy);   // GridToWorld → CellToWorld
        waypoints.emplace_back(wx, wy, 0.0f);
    }
    std::reverse(waypoints.begin(), waypoints.end());

    currentPath = Path(waypoints);
    std::cout << "PathPlanner: pad gevonden met " << waypoints.size() << " waypoints\n";
    return currentPath;
}

Path PathPlanner::GetCurrentPath() const {
    return currentPath;
}