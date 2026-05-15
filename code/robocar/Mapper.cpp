#include "Mapper.h"
#include <iostream>
#include <cstdio>

Mapper::Mapper(int width, int height, float resolution)
    : map(width, height, resolution)
    , updated(false)
{
}

// ── Scan-verwerking ───────────────────────────────────────────────

void Mapper::Update(const float ranges[], const float angles[],
                    int count, const Position& position)
{
    map.IntegrateScan(
        position.GetX(),
        position.GetY(),
        position.GetTheta(),
        angles, ranges, count
    );
    updated = true;
}

void Mapper::Update(const float scan360[], const Position& position) {
    static float angles[360];
    static bool  init = false;
    if (!init) {
        for (int i = 0; i < 360; ++i) angles[i] = static_cast<float>(i);
        init = true;
    }
    Update(scan360, angles, 360, position);
}

// In Mapper.cpp:
void Mapper::UpdateMotionCorrected(const float ranges[], const float angles[],
                                    int count, const Position& position,
                                    float omegaDegS, float scanDuurSec)
{
    map.IntegrateScanMotionCorrected(
        position.GetX(), position.GetY(), position.GetTheta(),
        omegaDegS, scanDuurSec,
        angles, ranges, count
    );
    updated = true;
}

// ── Visualisatie ──────────────────────────────────────────────────

void Mapper::PrintMap(float robotX, float robotY,
                      int scanCount, int coverage) const
{
    constexpr int WIN_W = 124, WIN_H = 24;

    int rx, ry;
    map.WorldToCell(robotX, robotY, rx, ry);
    int startX = rx - WIN_W / 2;
    int startY = ry - WIN_H / 2;

    std::cout << "\033[H";
    printf("╔══ LIDAR Map  [scans: %4d]  dekking: %2d%% ═══════════╗\n",
           scanCount, coverage);

    for (int cy = startY + WIN_H - 1; cy >= startY; --cy) {
        std::cout << "║";
        for (int cx = startX; cx < startX + WIN_W; ++cx) {
            if      (cx == rx && cy == ry)         std::cout << "\033[92mR\033[0m";
            else if (!map.InBounds(cx, cy))         std::cout << ' ';
            else if (map.IsOccupied(cx, cy))        std::cout << "\033[91m#\033[0m";
            else if (map.IsFree(cx, cy))            std::cout << "\033[90m.\033[0m";
            else                                     std::cout << ' ';
        }
        std::cout << "║\n";
    }

    printf("╚══ R=robot  #=muur  .=vrij  spatie=onbekend");
    for (int i = 0; i < WIN_W - 38; i++) std::cout << ' ';
    std::cout << "══╝\n";
}

// ── Queries ───────────────────────────────────────────────────────

int Mapper::GetCoverage() const {
    return static_cast<int>(map.GetCoveragePercent());
}

GridMap&       Mapper::GetMap()       { return map; }
const GridMap& Mapper::GetMap() const { return map; }

bool Mapper::SaveDebugMap(const std::string& filename) const {
    // SavePGMCropped: bijgesneden op verkend gebied + schaalbalken
    return map.SavePGMCropped(filename);
}