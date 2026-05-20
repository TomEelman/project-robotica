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

// ─────────────────────────────────────────────────────────────────
//  PlanPath — Wavefront (BFS vanuit het doel)
// ─────────────────────────────────────────────────────────────────
Path PathPlanner::PlanPath(Position start, Position goal, const GridMap& map) {
    gridMap = map;

    constexpr float MM2M = 0.001f;
    constexpr float M2MM = 1000.0f;

    const int W = gridMap.GetWidth();
    const int H = gridMap.GetHeight();

    // ── 1. Wereld → cel voor start en doel ───────────────────────
    int sx, sy, gx, gy;
    gridMap.WorldToCell(start.GetX() * MM2M, start.GetY() * MM2M, sx, sy);
    gridMap.WorldToCell(goal.GetX()  * MM2M, goal.GetY()  * MM2M, gx, gy);

    if (!gridMap.InBounds(sx, sy)) { std::cerr << "Planner: start outside grid\n"; return Path(); }
    if (!gridMap.InBounds(gx, gy)) { std::cerr << "Planner: goal outside grid\n";  return Path(); }

    // ── 2. Inflatie-masker bouwen ────────────────────────────────
    // Mark every cell within INFLATIE_CELLEN of a wall as
    // "blockedCell". Zo plant de planner een veilige marge rond muren
    // robot does not scrape past them. Done once per plan.
    std::vector<uint8_t> blockedCell(W * H, 0);
    auto idx = [&](int x, int y) { return y * W + x; };

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            if (!gridMap.IsOccupied(x, y)) continue;
            // Mark this obstacle + all cells within the inflation radius
            for (int dy = -INFLATIE_CELLEN; dy <= INFLATIE_CELLEN; ++dy) {
                for (int dx = -INFLATIE_CELLEN; dx <= INFLATIE_CELLEN; ++dx) {
                    int nx = x + dx, ny = y + dy;
                    if (gridMap.InBounds(nx, ny)) blockedCell[idx(nx, ny)] = 1;
                }
            }
        }
    }

    // ── 3. Betreedbaarheidstest ──────────────────────────────────
    // A cell is walkable if it is not occupied/inflated and -
    // depending on allowUnknown - optionally also if unknown.
    auto walkable = [&](int x, int y) -> bool {
        if (!gridMap.InBounds(x, y))   return false;
        if (blockedCell[idx(x, y)])    return false;   // wall + inflation margin
        if (gridMap.IsUnknown(x, y) && !allowUnknown) return false;
        return true;
    };

    // ── 4. Start-cel herstel ─────────────────────────────────────
    // Door inflatie kan de robot net in een "blockedCelle" cel staan
    // (right next to a wall). Find the nearest walkable cell
    // in a growing ring, otherwise BFS can never start.
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
        if (!gevonden) { std::cerr << "Planner: start trapped (no free neighbour)\n"; return Path(); }
    }

    // The goal may be a frontier cell bordering unknown space; we
    // only require it is not inside a wall/inflation.
    if (gridMap.InBounds(gx, gy) && blockedCell[idx(gx, gy)]) {
        std::cerr << "Planner: goal lies in obstacle/margin\n";
        return Path();
    }

    // ── 5. Wavefront: BFS VANUIT HET DOEL ────────────────────────
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

    bool startReached = false;
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        int cx = cur % W, cy = cur / W;

        if (cx == sx && cy == sy) { startReached = true; break; } // wave reached the start

        for (int i = 0; i < 4; ++i) {
            int nx = cx + dx4[i], ny = cy + dy4[i];
            if (!walkable(nx, ny))      continue;
            if (dist[idx(nx, ny)] >= 0) continue;        // already visited
            dist[idx(nx, ny)] = dist[cur] + 1;
            q.push(idx(nx, ny));
        }
    }

    if (!startReached) { std::cerr << "Planner: no path (wave did not reach start)\n"; return Path(); }

    // ── 6. Padreconstructie: volg dalende afstanden ──────────────
    // From the start, step to the neighbour cell with the LOWEST
    // dist value. This is guaranteed to reach the goal (dist=0).
    std::vector<std::pair<int,int>> rawCells;
    int cx = sx, cy = sy;
    rawCells.emplace_back(cx, cy);

    int safetyCounter = W * H;  // guard against infinite loop on bad data
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

    // ── 7. Waypoint-verdunning: alleen hoekpunten bewaren ────────
    // The raw path is cell-by-cell. We only give the Navigator the
    // points where the DIRECTION changes (the corners), plus start
    // and end. This yields long straight runs -> less wobbling.
    constexpr int MIN_SEG = 5;  // minimum segment length between waypoints

    auto cellToWaypoint = [&](int x, int y) -> Position {
        float wx_m, wy_m;
        gridMap.CellToWorld(x, y, wx_m, wy_m);
        return Position(wx_m * M2MM, wy_m * M2MM, 0.0f);
    };

    std::vector<Position> waypoints;
    if (rawCells.size() <= 2) {
        for (auto& [x, y] : rawCells) waypoints.push_back(cellToWaypoint(x, y));
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
    std::cout << "Planner: Wavefront path with " << waypoints.size()
              << " waypoints (" << rawCells.size() << " cells)\n";
    return currentPath;
}

Path PathPlanner::GetCurrentPath() const {
    return currentPath;
}