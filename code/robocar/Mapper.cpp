#include "Mapper.h"
#include <cmath>

// ═══════════════════════════════════════════════════════════════════
//  Constructor
// ═══════════════════════════════════════════════════════════════════

Mapper::Mapper(int width, int height, float resolution)
    : map(width, height, resolution)
    , updated(false)
{
}

// ═══════════════════════════════════════════════════════════════════
//  Update  –  arbitraire hoeken + afstanden
// ═══════════════════════════════════════════════════════════════════

void Mapper::Update(const float ranges[], const float angles[],
                    int count, const Position& position)
{
    map.IntegrateScan(
        position.GetX(),
        position.GetY(),
        position.GetTheta(),   // in radialen
        angles,
        ranges,
        count
    );
    updated = true;
}

// ═══════════════════════════════════════════════════════════════════
//  Update  –  impliciete hoeken 0..359° (RPLIDAR 360-array)
// ═══════════════════════════════════════════════════════════════════

void Mapper::Update(const float scan360[], const Position& position) {
    // Statische hoekentabel, één keer aangemaakt
    static float angles[360];
    static bool  init = false;
    if (!init) {
        for (int i = 0; i < 360; ++i) angles[i] = static_cast<float>(i);
        init = true;
    }

    Update(scan360, angles, 360, position);
}

// ═══════════════════════════════════════════════════════════════════
//  GetCoverage  –  percentage verkende cellen (0..100)
// ═══════════════════════════════════════════════════════════════════

int Mapper::GetCoverage() const {
    return static_cast<int>(map.GetCoveragePercent());
}

// ═══════════════════════════════════════════════════════════════════
//  GetMap
// ═══════════════════════════════════════════════════════════════════

GridMap& Mapper::GetMap() {
    return map;
}

const GridMap& Mapper::GetMap() const {
    return map;
}

// ═══════════════════════════════════════════════════════════════════
//  SaveDebugMap
// ═══════════════════════════════════════════════════════════════════

bool Mapper::SaveDebugMap(const std::string& filename) const {
    return map.SavePGM(filename);
}