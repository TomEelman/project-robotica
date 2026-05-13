#include "ExplorationPlanner.h"
#include "Mapper.h"
#include <cmath>

// ─────────────────────────────────────────────────────────────────
//  KiesFrontierDoel
//
//  Score = 0.4 × (afstand / 4m) + 0.6 × (LIDAR-ruimte / 2m)
//  Frontiers binnen 0.3m van de robot worden overgeslagen.
// ─────────────────────────────────────────────────────────────────

Position KiesFrontierDoel(const Mapper& mapper, const Position& huidig,
                           const float lidarRanges[360])
{
    constexpr float MM2M = 0.001f;
    const float huidigX_m = huidig.GetX() * MM2M;
    const float huidigY_m = huidig.GetY() * MM2M;
    const int W = mapper.GetMap().GetWidth();
    const int H = mapper.GetMap().GetHeight();
    const int dx[4] = {-1,1,0,0};
    const int dy[4] = { 0,0,-1,1};

    float bestScore = -1.0f;
    float bestWx_m  = huidigX_m;
    float bestWy_m  = huidigY_m;

    for (int cy = 0; cy < H; ++cy) {
        for (int cx = 0; cx < W; ++cx) {
            if (!mapper.GetMap().IsFree(cx, cy)) continue;

            bool heeftOnbekendeBuur = false;
            for (int d = 0; d < 4; ++d) {
                if (mapper.GetMap().InBounds(cx+dx[d], cy+dy[d]) &&
                    mapper.GetMap().IsUnknown(cx+dx[d], cy+dy[d])) {
                    heeftOnbekendeBuur = true; break;
                }
            }
            if (!heeftOnbekendeBuur) continue;

            float wx_m, wy_m;
            mapper.GetMap().CellToWorld(cx, cy, wx_m, wy_m);

            float ddx    = wx_m - huidigX_m;
            float ddy    = wy_m - huidigY_m;
            float dist_m = std::sqrt(ddx*ddx + ddy*ddy);
            if (dist_m < 0.3f) continue;

            float hoek_rad  = std::atan2(ddy, ddx)
                              - (huidig.GetTheta() * static_cast<float>(M_PI) / 180.0f);
            int   lidarIdx  = ((int)(hoek_rad * 180.0f / static_cast<float>(M_PI)) + 360) % 360;
            float vrijRuimte = lidarRanges[lidarIdx];
            if (vrijRuimte <= 0.0f || vrijRuimte > 8000.0f) vrijRuimte = 8000.0f;

            float score = 0.4f * std::min(dist_m, 4.0f) / 4.0f
                        + 0.6f * std::min(vrijRuimte, 2000.0f) / 2000.0f;

            if (score > bestScore) {
                bestScore = score;
                bestWx_m  = wx_m;
                bestWy_m  = wy_m;
            }
        }
    }
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
//  ExplorationPlanner
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