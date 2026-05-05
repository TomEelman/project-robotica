#include "PathPlanner.h"

#include <queue>
#include <vector>
#include <climits>
#include <cmath>
#include <algorithm>
#include <iostream>

// ----- Lokaal A*-knooppunt -----
namespace {
struct AStarNode {
    int x, y;
    int g, h, f;

    AStarNode(int _x = 0, int _y = 0, int _g = 0, int _h = 0)
        : x(_x), y(_y), g(_g), h(_h), f(_g + _h) {}

    // Min-heap op f
    bool operator>(const AStarNode& other) const { return f > other.f; }
};
} // anonymous namespace

PathPlanner::PathPlanner(const GridMap& map, bool allowUnknownIn)
    : gridMap(map)
    , currentPath()
    , allowUnknown(allowUnknownIn)
{
}

void PathPlanner::UpdateMap(const GridMap& map) {
    gridMap = map;
}

Path PathPlanner::PlanPath(Position start, Position goal, const GridMap& map) {
    gridMap = map;

    int sx, sy, gx, gy;
    if (!gridMap.WorldToGrid(start.GetX(), start.GetY(), sx, sy)) {
        std::cerr << "PathPlanner: start buiten grid\n";
        return Path();
    }
    if (!gridMap.WorldToGrid(goal.GetX(), goal.GetY(), gx, gy)) {
        std::cerr << "PathPlanner: goal buiten grid\n";
        return Path();
    }

    // Helper: mag deze cel betreden worden?
    auto walkable = [&](int x, int y) -> bool {
        if (!gridMap.InBounds(x, y)) return false;
        int c = gridMap.GetCell(x, y);
        if (c == Cell::OCCUPIED) return false;
        if (c == Cell::UNKNOWN && !allowUnknown) return false;
        return true;
    };

    // Start moet ook walkable zijn (anders zit de robot in een muur)
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
    std::vector<std::vector<bool>> closed(W, std::vector<bool>(H, false));
    std::vector<std::vector<int>>  gScore(W, std::vector<int>(H, INT_MAX));

    // parent[x][y] = (px, py); -1 = geen parent
    std::vector<std::vector<int>> parentX(W, std::vector<int>(H, -1));
    std::vector<std::vector<int>> parentY(W, std::vector<int>(H, -1));

    // 4-richtingen (geen diagonaal — simpeler en robot kan toch niet diagonaal rijden)
    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = { 0, 0,-1, 1};

    auto heuristic = [&](int x, int y) {
        return std::abs(x - gx) + std::abs(y - gy);   // Manhattan
    };

    gScore[sx][sy] = 0;
    open.emplace(sx, sy, 0, heuristic(sx, sy));

    bool found = false;
    while (!open.empty()) {
        AStarNode cur = open.top();
        open.pop();

        if (closed[cur.x][cur.y]) continue;
        closed[cur.x][cur.y] = true;

        if (cur.x == gx && cur.y == gy) {
            found = true;
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int nx = cur.x + dx[i];
            int ny = cur.y + dy[i];

            if (!walkable(nx, ny)) continue;
            if (closed[nx][ny])    continue;

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

    // Reconstrueer pad (van goal terug naar start)
    std::vector<Position> waypoints;
    int cx = gx, cy = gy;
    while (!(cx == sx && cy == sy)) {
        float wx, wy;
        gridMap.GridToWorld(cx, cy, wx, wy);
        waypoints.emplace_back(wx, wy, 0.0f);

        int px = parentX[cx][cy];
        int py = parentY[cx][cy];
        if (px < 0 || py < 0) break;   // safety
        cx = px;
        cy = py;
    }
    // Voeg start ook toe
    {
        float wx, wy;
        gridMap.GridToWorld(sx, sy, wx, wy);
        waypoints.emplace_back(wx, wy, 0.0f);
    }
    std::reverse(waypoints.begin(), waypoints.end());

    currentPath = Path(waypoints);
    std::cout << "PathPlanner: pad gevonden met " << waypoints.size()
              << " waypoints\n";
    return currentPath;
}

Path PathPlanner::GetCurrentPath() const {
    return currentPath;
}