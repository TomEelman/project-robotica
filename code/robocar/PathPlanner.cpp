#include "PathPlanner.h"

#include <queue>
#include <vector>
#include <climits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstdint>

PathPlanner::PathPlanner(const GridMap& map, bool allowUnknownIn)
    : gridMap(map), currentPath(), allowUnknown(allowUnknownIn)
{
}

void PathPlanner::UpdateMap(const GridMap& map) {
    gridMap = map;
}

//  PlanPath — Wavefront BFS
Path PathPlanner::PlanPath(Position start, Position goal, const GridMap& map) {
    gridMap = map;

    constexpr float MM2M = 0.001f;
    constexpr float M2MM = 1000.0f;

    const int W = gridMap.GetWidth();
    const int H = gridMap.GetHeight();

    // convert world to cell
    int sx, sy, gx, gy;
    gridMap.WorldToCell(start.GetX() * MM2M, start.GetY() * MM2M, sx, sy);
    gridMap.WorldToCell(goal.GetX()  * MM2M, goal.GetY()  * MM2M, gx, gy);

    // if (!gridMap.InBounds(sx, sy)) { std::cerr << "Planner: start outside grid\n"; return Path(); }
    // if (!gridMap.InBounds(gx, gy)) { std::cerr << "Planner: goal outside grid\n";  return Path(); }

    // mark all cells within INFLATION_CELLS of a wall as blocked so planner can avoid the wall
    std::vector<uint8_t> blockedCell(W * H, 0);
    auto idx = [&](int x, int y) { return y * W + x; };

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (!gridMap.IsOccupied(x, y)) continue;
            // Mark this obstacle + all cells within the inflation radius
            for (int dy = -INFLATION_CELLS; dy <= INFLATION_CELLS; ++dy) {
                for (int dx = -INFLATION_CELLS; dx <= INFLATION_CELLS; ++dx) {
                    int nx = x + dx, ny = y + dy;
                    if (gridMap.InBounds(nx, ny)) blockedCell[idx(nx, ny)] = 1;
                }
            }
        }
    }

    // a cell is walkable if it is not occupied/inflated
    auto walkable = [&](int x, int y) -> bool {
        if (!gridMap.InBounds(x, y))   return false;
        if (blockedCell[idx(x, y)])    return false;
        if (gridMap.IsUnknown(x, y) && !allowUnknown) return false;
        return true;
    };

    if (!walkable(sx, sy)) {
        bool gevonden = false;
        for (int r = 1; r <= 6 && !gevonden; ++r) {
            for (int dx = -r; dx <= r && !gevonden; ++dx)
                for (int dy = -r; dy <= r && !gevonden; ++dy) {
                    if (std::abs(dx) != r && std::abs(dy) != r) continue; // ring edge only
                    if (walkable(sx + dx, sy + dy)) {
                        sx += dx; sy += dy; gevonden = true;
                    }
                }
        }
        // if (!gevonden) { std::cerr << "Planner: start trapped (no free neighbour)\n"; return Path(); }
    }
    if (gridMap.InBounds(gx, gy) && blockedCell[idx(gx, gy)]) {
        // std::cerr << "Planner: goal lies in obstacle/margin\n";
        return Path();
    }

    // Wavefront
    // dist[cell] = number of steps from that cell to the goal.
    // The wave starts at the goal (dist=0) and expands over
    // walkable neighbours. Because BFS works in layers, each cell
    // gets the shortest distance to the goal.
    std::vector<int> dist(W * H, -1);
    std::queue<int>  q;

    dist[idx(gx, gy)] = 0;
    q.push(idx(gx, gy));

    const int dx4[4] = {-1, 1, 0, 0};
    const int dy4[4] = { 0, 0,-1, 1};

    while (!q.empty()) {
        int cur = q.front(); q.pop();
        int cx = cur % W, cy = cur / W;

        if (cx == sx && cy == sy) break;  // wave reached the start

        for (int i = 0; i < 4; ++i) {
            int nx = cx + dx4[i], ny = cy + dy4[i];
            if (!walkable(nx, ny))      continue;
            if (dist[idx(nx, ny)] >= 0) continue;        // already visited
            dist[idx(nx, ny)] = dist[cur] + 1;
            q.push(idx(nx, ny));
        }
    }

    if (dist[idx(sx, sy)] < 0) {
        return Path();
    }

    // step from start to neighbor cell with LOWEST dist value this is guaranteed to reach the goal (dist=0)
    std::vector<std::pair<int,int>> rawCells;
    int cx = sx, cy = sy;
    rawCells.emplace_back(cx, cy);

    int safetyCounter = W * H;  
    while (!(cx == gx && cy == gy) && safetyCounter-- > 0) {
        int bestX2 = cx, bestY2 = cy;
        int bestDist2 = dist[idx(cx, cy)];
        for (int i = 0; i < 4; ++i) {
            int nx = cx + dx4[i], ny = cy + dy4[i];
            if (!gridMap.InBounds(nx, ny)) continue;

            int d = dist[idx(nx, ny)];
            if (d >= 0 && d < bestDist2) { bestDist2 = d; bestX2 = nx; bestY2 = ny; }
        }
        if (bestX2 == cx && bestY2 == cy) break;  // no further descent -> done
        cx = bestX2; cy = bestY2;
        rawCells.emplace_back(cx, cy);
    }

    constexpr int MIN_SEG = 5;  // minimum segment length between waypoints

    auto cellToWaypoint = [&](int x, int y) -> Position {
        float wx_m, wy_m;
        gridMap.CellToWorld(x, y, wx_m, wy_m);
        return Position(wx_m * M2MM, wy_m * M2MM, 0.0f);
    };

    std::vector<Position> waypoints;
    
    if (rawCells.size() <= 2) {
        for (size_t i = 0; i < rawCells.size(); ++i) {
            std::pair<int, int> currentCell = rawCells[i];
            int cellX = currentCell.first;
            int cellY = currentCell.second;
            
            Position processedPosition = cellToWaypoint(cellX, cellY);
            
            waypoints.push_back(processedPosition);
        }
    } else {
        waypoints.push_back(cellToWaypoint(rawCells[0].first, rawCells[0].second));

        int prevDx = rawCells[1].first  - rawCells[0].first;
        int prevDy = rawCells[1].second - rawCells[0].second;
        int segLen = 1;

        for (size_t i = 1; i + 1 < rawCells.size(); ++i) {
            int curDx = rawCells[i+1].first  - rawCells[i].first;
            int curDy = rawCells[i+1].second - rawCells[i].second;
            bool directionChanged = (curDx != prevDx || curDy != prevDy);

            if (directionChanged && segLen >= MIN_SEG) {
                waypoints.push_back(cellToWaypoint(rawCells[i].first, rawCells[i].second));
                segLen = 0;
            }
            prevDx = curDx; prevDy = curDy; ++segLen;
        }
        waypoints.push_back(cellToWaypoint(rawCells.back().first, rawCells.back().second));
    }

    currentPath = Path(waypoints);
    //std::cout << "Planner: Wavefront path with " << waypoints.size()
    //          << " waypoints (" << rawCells.size() << " cells)\n";
    return currentPath;
}

Path PathPlanner::GetCurrentPath() const {
    return currentPath;
}