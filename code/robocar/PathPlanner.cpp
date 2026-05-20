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

// Robot-breedte in cellen. Bij een gridresolutie van 0.03m/cel en een
// chassisbreedte van ~280mm = 0.28m → 0.28/0.03 ≈ 9 cellen.
// We gebruiken de helft als "clearance" (straal): 5 cellen = 150mm buffer.
// Verhoog dit als de robot nog steeds smalle stukken inrijdt.
static constexpr int INFLATIE_RADIUS = 5;

PathPlanner::PathPlanner(const GridMap& map, bool allowUnknownIn)
    : gridMap(map), currentPath(), allowUnknown(allowUnknownIn)
{
}

void PathPlanner::UpdateMap(const GridMap& map) {
    gridMap = map;
}

Path PathPlanner::PlanPath(Position start, Position goal, const GridMap& map) {
    gridMap = map;

    constexpr float MM2M = 0.001f;
    constexpr float M2MM = 1000.0f;

    int sx, sy, gx, gy;
    gridMap.WorldToCell(start.GetX() * MM2M, start.GetY() * MM2M, sx, sy);
    if (!gridMap.InBounds(sx, sy)) {
        std::cerr << "PathPlanner: start buiten grid\n";
        return Path();
    }

    gridMap.WorldToCell(goal.GetX() * MM2M, goal.GetY() * MM2M, gx, gy);
    if (!gridMap.InBounds(gx, gy)) {
        std::cerr << "PathPlanner: goal buiten grid\n";
        return Path();
    }

    // ── Basis walkable: niet bezet, niet onbekend (tenzij allowUnknown) ──
    auto basisWalkable = [&](int x, int y) -> bool {
        if (!gridMap.InBounds(x, y))                      return false;
        if (gridMap.IsOccupied(x, y))                     return false;
        if (gridMap.IsUnknown(x, y) && !allowUnknown)     return false;
        return true;
    };

    // ── Inflatie: cel is alleen betreedbaar als er binnen INFLATIE_RADIUS
    //    geen bezette buurcel is. Zo blijft de robot automatisch uit smalle
    //    gangen en rijdt hij niet langs muren. ────────────────────────────
    auto walkable = [&](int x, int y) -> bool {
        if (!basisWalkable(x, y)) return false;
        for (int ddx = -INFLATIE_RADIUS; ddx <= INFLATIE_RADIUS; ++ddx) {
            for (int ddy = -INFLATIE_RADIUS; ddy <= INFLATIE_RADIUS; ++ddy) {
                if (ddx*ddx + ddy*ddy > INFLATIE_RADIUS*INFLATIE_RADIUS) continue;
                int nx = x + ddx, ny = y + ddy;
                if (!gridMap.InBounds(nx, ny)) continue;
                if (gridMap.IsOccupied(nx, ny)) return false;
            }
        }
        return true;
    };

    // ── Als startcel te dicht bij muur is, zoek dichtstbijzijnde vrije cel ──
    if (!walkable(sx, sy)) {
        bool gevonden = false;
        for (int r = 1; r <= INFLATIE_RADIUS + 3 && !gevonden; ++r) {
            for (int ddx = -r; ddx <= r && !gevonden; ++ddx) {
                for (int ddy = -r; ddy <= r && !gevonden; ++ddy) {
                    if (std::abs(ddx) != r && std::abs(ddy) != r) continue;
                    int nx = sx + ddx, ny = sy + ddy;
                    if (walkable(nx, ny)) {
                        std::cerr << "PathPlanner: start te dicht bij muur, fallback naar ("
                                  << nx << ", " << ny << ")\n";
                        sx = nx; sy = ny;
                        gevonden = true;
                    }
                }
            }
        }
        if (!gevonden) {
            std::cerr << "PathPlanner: start cel niet betreedbaar (ook geen buur vrij)\n";
            return Path();
        }
    }

    // ── Goal-cel: als die ook te dicht bij muur is, zoek iets vrijs ────────
    if (!walkable(gx, gy)) {
        bool gevonden = false;
        for (int r = 1; r <= INFLATIE_RADIUS + 3 && !gevonden; ++r) {
            for (int ddx = -r; ddx <= r && !gevonden; ++ddx) {
                for (int ddy = -r; ddy <= r && !gevonden; ++ddy) {
                    if (std::abs(ddx) != r && std::abs(ddy) != r) continue;
                    int nx = gx + ddx, ny = gy + ddy;
                    if (walkable(nx, ny)) {
                        std::cerr << "PathPlanner: goal te dicht bij muur, fallback naar ("
                                  << nx << ", " << ny << ")\n";
                        gx = nx; gy = ny;
                        gevonden = true;
                    }
                }
            }
        }
        if (!gevonden) {
            std::cerr << "PathPlanner: goal cel niet betreedbaar\n";
            return Path();
        }
    }

    const int W = gridMap.GetWidth();
    const int H = gridMap.GetHeight();

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    std::vector<std::vector<bool>> closed (W, std::vector<bool>(H, false));
    std::vector<std::vector<int>>  gScore (W, std::vector<int> (H, INT_MAX));
    std::vector<std::vector<int>>  parentX(W, std::vector<int> (H, -1));
    std::vector<std::vector<int>>  parentY(W, std::vector<int> (H, -1));

    // ── 8-richting beweging: diagonaal + rechtlijnig ─────────────────────
    // Diagonale stap kost 14 (≈ sqrt(2)*10), rechtlijnig kost 10.
    // Dit geeft vloeiendere paden dan pure 4-richting.
    const int ddx[8] = {-1, 1,  0, 0, -1, -1,  1,  1};
    const int ddy[8] = { 0, 0, -1, 1, -1,  1, -1,  1};
    const int cost[8]= { 10,10, 10,10,  14, 14, 14, 14};

    auto heuristic = [&](int x, int y) {
        // Octile distance: betere heuristiek voor 8-richting
        int dx = std::abs(x - gx);
        int dy = std::abs(y - gy);
        return 10 * (dx + dy) + (14 - 2*10) * std::min(dx, dy);
    };

    gScore[sx][sy] = 0;
    open.emplace(sx, sy, 0, heuristic(sx, sy));

    bool found = false;
    while (!open.empty()) {
        AStarNode cur = open.top(); open.pop();
        if (closed[cur.x][cur.y]) continue;
        closed[cur.x][cur.y] = true;

        if (cur.x == gx && cur.y == gy) { found = true; break; }

        for (int i = 0; i < 8; ++i) {
            int nx = cur.x + ddx[i];
            int ny = cur.y + ddy[i];
            if (!walkable(nx, ny) || closed[nx][ny]) continue;

            // Bij diagonale stap: controleer ook de twee aangrenzende
            // rechtlijnige cellen om door smalle hoeken te snijden te voorkomen.
            if (i >= 4) {
                if (!walkable(cur.x + ddx[i], cur.y) ||
                    !walkable(cur.x, cur.y + ddy[i]))
                    continue;
            }

            int newG = gScore[cur.x][cur.y] + cost[i];
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

    // ── Reconstrueer pad goal → start ──────────────────────────────────────
    std::vector<std::pair<int,int>> rawCells;
    {
        int cx = gx, cy = gy;
        while (!(cx == sx && cy == sy)) {
            rawCells.emplace_back(cx, cy);
            int px = parentX[cx][cy];
            int py = parentY[cx][cy];
            if (px < 0 || py < 0) break;
            cx = px; cy = py;
        }
        rawCells.emplace_back(sx, sy);
        std::reverse(rawCells.begin(), rawCells.end());
    }

    // ── Waypoint-verdunning via "string pulling" (Theta*-achtig) ──────────
    // Ipv elk hoekpunt bewaren: trek een rechte lijn van het huidige waypoint
    // naar steeds verder weg gelegen cellen. Zolang die lijn vrij is (geen
    // muren raakt), sla je alle tussenpunten over. Zo krijg je lange rechte
    // stukken en vermijd je scherpe hoeken in smalle gangen.
    auto celNaarWaypoint = [&](int cx2, int cy2) -> Position {
        float wx_m, wy_m;
        gridMap.CellToWorld(cx2, cy2, wx_m, wy_m);
        return Position(wx_m * M2MM, wy_m * M2MM, 0.0f);
    };

    // Bresenham-lijn check: geeft true als alle cellen op de lijn walkable zijn
    auto lijVrij = [&](int x0, int y0, int x1, int y1) -> bool {
        int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
        int sx2 = (x0 < x1) ? 1 : -1;
        int sy2 = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int cx2 = x0, cy2 = y0;
        while (true) {
            if (!walkable(cx2, cy2)) return false;
            if (cx2 == x1 && cy2 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; cx2 += sx2; }
            if (e2 <  dx) { err += dx; cy2 += sy2; }
        }
        return true;
    };

    std::vector<Position> waypoints;
    if (rawCells.size() <= 2) {
        for (auto& [cx2, cy2] : rawCells)
            waypoints.push_back(celNaarWaypoint(cx2, cy2));
    } else {
        size_t anker = 0;
        waypoints.push_back(celNaarWaypoint(rawCells[0].first, rawCells[0].second));

        while (anker + 1 < rawCells.size()) {
            // Zoek het verste punt waarnaar een vrije rechte lijn bestaat
            size_t verste = anker + 1;
            for (size_t j = anker + 2; j < rawCells.size(); ++j) {
                if (lijVrij(rawCells[anker].first,  rawCells[anker].second,
                            rawCells[j].first,      rawCells[j].second))
                    verste = j;
                else
                    break; // pad geblokeerd, niet verder zoeken
            }
            waypoints.push_back(celNaarWaypoint(rawCells[verste].first,
                                                rawCells[verste].second));
            anker = verste;
        }
    }

    currentPath = Path(waypoints);
    std::cout << "PathPlanner: pad gevonden met " << waypoints.size()
              << " waypoints (van " << rawCells.size() << " raw cellen)\n";
    return currentPath;
}

Path PathPlanner::GetCurrentPath() const {
    return currentPath;
}