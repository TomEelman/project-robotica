#include "ExplorationPlanner.h"
#include "Mapper.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <vector>
#include <climits>

// ─────────────────────────────────────────────────────────────────
//  Interne BFS — geeft padlengte in cellen van (sx,sy) naar (gx,gy)
//  door alleen FREE cellen. Geeft INT_MAX terug als onbereikbaar.
//  BFS (geen A*) want we hoeven alleen de lengte, geen volledig pad.
// ─────────────────────────────────────────────────────────────────
static int BFSAfstand(const GridMap& map, int sx, int sy, int gx, int gy) {
    const int W = map.GetWidth();
    const int H = map.GetHeight();
    if (!map.InBounds(sx,sy) || !map.InBounds(gx,gy)) return INT_MAX;
    if (map.IsOccupied(sx,sy) || map.IsOccupied(gx,gy)) return INT_MAX;

    // Gebruik een platte visited-vector voor snelheid
    std::vector<int> dist(W * H, -1);
    std::queue<int> q;

    auto idx = [&](int x, int y){ return y * W + x; };
    dist[idx(sx,sy)] = 0;
    q.push(idx(sx,sy));

    const int dx[4] = {-1,1, 0,0};
    const int dy[4] = { 0,0,-1,1};

    while (!q.empty()) {
        int cur = q.front(); q.pop();
        int cx = cur % W, cy = cur / W;
        int d  = dist[cur];

        if (cx == gx && cy == gy) return d;

        for (int i = 0; i < 4; ++i) {
            int nx = cx+dx[i], ny = cy+dy[i];
            if (!map.InBounds(nx,ny))      continue;
            if (map.IsOccupied(nx,ny))     continue;
            // Onbekende cellen toestaan als doorgang maar niet als doel —
            // de robot kent het pad en kan er overheen teruglopen
            int ni = idx(nx,ny);
            if (dist[ni] >= 0)             continue;
            dist[ni] = d + 1;
            q.push(ni);
        }
    }
    return INT_MAX;  // onbereikbaar
}

// ─────────────────────────────────────────────────────────────────
//  Blacklist hulpfuncties
// ─────────────────────────────────────────────────────────────────
void VoegToeAanBlacklist(std::vector<BlacklistItem>& blacklist,
                          float x_mm, float y_mm, int ttl)
{
    // Updaten als het doel al in de lijst staat
    for (auto& item : blacklist) {
        float dx = item.x_mm - x_mm, dy = item.y_mm - y_mm;
        if (dx*dx + dy*dy < 200.0f*200.0f) {   // binnen 200mm = zelfde frontier
            item.ttl = std::max(item.ttl, ttl);
            return;
        }
    }
    blacklist.push_back({x_mm, y_mm, ttl});
}

void TickBlacklist(std::vector<BlacklistItem>& blacklist) {
    for (auto& item : blacklist) --item.ttl;
    blacklist.erase(
        std::remove_if(blacklist.begin(), blacklist.end(),
                       [](const BlacklistItem& b){ return b.ttl <= 0; }),
        blacklist.end());
}

static bool InBlacklist(const std::vector<BlacklistItem>& blacklist,
                         float x_mm, float y_mm)
{
    for (const auto& item : blacklist) {
        float dx = item.x_mm - x_mm, dy = item.y_mm - y_mm;
        if (dx*dx + dy*dy < 200.0f*200.0f) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────
//  KiesFrontierDoel
//
//  Score per frontier-cel:
//    bereikbaarheid  (50%) — kortere BFS-afstand = hogere score
//                            onbereikbare cellen worden overgeslagen
//    LIDAR-ruimte    (30%) — meer ruimte in die richting = hogere score
//    nabijheid       (20%) — dichterbij = iets hoger (voorkomt
//                            verre doelen die toch onbereikbaar zijn)
//
//  Sampling: niet elk van de 40.000 cellen hoeft beoordeeld. We
//  bemonsteren frontiers op een rooster van SAMPLE_STAP cellen.
//  Bij 260×160 grid en SAMPLE_STAP=3: ~2800 kandidaten ipv 41600.
// ─────────────────────────────────────────────────────────────────
Position KiesFrontierDoel(const Mapper& mapper,
                           const Position& huidig,
                           const float lidarRanges[360],
                           const std::vector<BlacklistItem>& blacklist)
{
    constexpr float MM2M       = 0.001f;
    constexpr int   SAMPLE_STAP = 3;     // scan elke 3e cel — genoeg precisie, veel sneller
    constexpr float MAX_BFS     = 400.0f; // cellen; frontiers verder dan dit negeren

    const GridMap& map      = mapper.GetMap();
    const float huidigX_m  = huidig.GetX() * MM2M;
    const float huidigY_m  = huidig.GetY() * MM2M;
    const int W = map.GetWidth();
    const int H = map.GetHeight();

    int rx, ry;
    map.WorldToCell(huidigX_m, huidigY_m, rx, ry);

    const int ddx4[4] = {-1,1,0,0};
    const int ddy4[4] = { 0,0,-1,1};

    float bestScore = -1.0f;
    float bestWx_m  = huidigX_m;
    float bestWy_m  = huidigY_m;

    for (int cy = 0; cy < H; cy += SAMPLE_STAP) {
        for (int cx = 0; cx < W; cx += SAMPLE_STAP) {
            // Moet vrije cel zijn met minstens één onbekende buur (= frontier)
            if (!map.IsFree(cx, cy)) continue;

            bool heeftOnbekendeBuur = false;
            for (int d = 0; d < 4; ++d) {
                if (map.InBounds(cx+ddx4[d], cy+ddy4[d]) &&
                    map.IsUnknown(cx+ddx4[d], cy+ddy4[d])) {
                    heeftOnbekendeBuur = true; break;
                }
            }
            if (!heeftOnbekendeBuur) continue;

            float wx_m, wy_m;
            map.CellToWorld(cx, cy, wx_m, wy_m);

            // Sla over als in blacklist
            if (InBlacklist(blacklist, wx_m / MM2M, wy_m / MM2M)) continue;

            // ── BFS-afstand (in cellen) ───────────────────────────
            int bfsAfstand = BFSAfstand(map, rx, ry, cx, cy);
            if (bfsAfstand == INT_MAX) continue;         // onbereikbaar → overslaan
            if (bfsAfstand > (int)MAX_BFS) continue;     // te ver weg

            // ── LIDAR-ruimte in die richting ──────────────────────
            float ddxF    = wx_m - huidigX_m;
            float ddyF    = wy_m - huidigY_m;
            float hoek_rad = std::atan2(ddyF, ddxF)
                             - (huidig.GetTheta() * static_cast<float>(M_PI) / 180.0f);
            int   lidarIdx  = ((int)(hoek_rad * 180.0f / static_cast<float>(M_PI)) + 360) % 360;
            float vrijRuimte = lidarRanges[lidarIdx];
            if (vrijRuimte <= 0.0f || vrijRuimte > 8000.0f) vrijRuimte = 8000.0f;

            // ── Euclidische afstand (meter) ───────────────────────
            float dist_m = std::sqrt(ddxF*ddxF + ddyF*ddyF);
            if (dist_m < 0.3f) continue;

            // ── Score: bereikbaarheid 50%, LIDAR 30%, nabijheid 20% ──
            // bfsAfstand: lager = beter → omdraaien
            float scoreReach = 1.0f - std::min((float)bfsAfstand, MAX_BFS) / MAX_BFS;
            float scoreLidar = std::min(vrijRuimte, 2000.0f) / 2000.0f;
            float scoreDist  = 1.0f - std::min(dist_m, 8.0f) / 8.0f;

            float score = 0.5f * scoreReach
                        + 0.3f * scoreLidar
                        + 0.2f * scoreDist;

            if (score > bestScore) {
                bestScore = score;
                bestWx_m  = wx_m;
                bestWy_m  = wy_m;
            }
        }
    }

    // Geen scorende frontier gevonden → geef robotpositie terug (= geen doel)
    return Position(bestWx_m / MM2M, bestWy_m / MM2M, 0.0f);
}


// ─────────────────────────────────────────────────────────────────
//  TelFrontiers
// ─────────────────────────────────────────────────────────────────
int TelFrontiers(const Mapper& mapper) {
    const int W  = mapper.GetMap().GetWidth();
    const int H  = mapper.GetMap().GetHeight();
    const int dx[4] = {-1,1,0,0};
    const int dy[4] = { 0,0,-1,1};
    int count = 0;
    for (int cy = 0; cy < H; ++cy)
        for (int cx = 0; cx < W; ++cx) {
            if (!mapper.GetMap().IsFree(cx, cy)) continue;
            for (int d = 0; d < 4; ++d)
                if (mapper.GetMap().InBounds(cx+dx[d], cy+dy[d]) &&
                    mapper.GetMap().IsUnknown(cx+dx[d], cy+dy[d])) {
                    ++count; break;
                }
        }
    return count;
}


// ─────────────────────────────────────────────────────────────────
//  ExplorationPlanner (ongewijzigd)
// ─────────────────────────────────────────────────────────────────
ExplorationPlanner::ExplorationPlanner()
    : currentTarget(0.0f, 0.0f, 0.0f)
    , state(SEARCHING)
{}

Position ExplorationPlanner::ComputeNextTarget(const GridMap& map, Position currentPos) {
    state = SEARCHING;

    int W = map.GetWidth(), H = map.GetHeight();
    int rx, ry;
    map.WorldToCell(currentPos.GetX(), currentPos.GetY(), rx, ry);
    bool robotInGrid = map.InBounds(rx, ry);

    int bestX = -1, bestY = -1, bestDistSq = -1;
    const int dx[4] = {-1,1,0,0};
    const int dy[4] = { 0,0,-1,1};

    for (int x = 0; x < W; ++x) {
        for (int y = 0; y < H; ++y) {
            if (!map.IsFree(x, y)) continue;
            bool isFrontier = false;
            for (int i = 0; i < 4; ++i)
                if (map.IsUnknown(x+dx[i], y+dy[i])) { isFrontier = true; break; }
            if (!isFrontier) continue;

            int ddx = robotInGrid ? (x - rx) : x;
            int ddy = robotInGrid ? (y - ry) : y;
            int distSq = ddx*ddx + ddy*ddy;
            if (bestDistSq < 0 || distSq < bestDistSq) {
                bestDistSq = distSq; bestX = x; bestY = y;
            }
        }
    }

    if (bestX < 0) { state = IDLE; return currentTarget; }

    float wx, wy;
    map.CellToWorld(bestX, bestY, wx, wy);
    currentTarget = Position(wx, wy, 0.0f);
    state = MOVING;
    return currentTarget;
}

Position         ExplorationPlanner::GetCurrentTarget() const { return currentTarget; }
ExplorationState ExplorationPlanner::GetState()         const { return state; }
bool             ExplorationPlanner::HasTarget()        const { return state == MOVING; }