#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <cstdint>
#include <string>
#include "Path.h"

class GridMap {
public:
    // ── Celstatussen ─────────────────────────────────────────
    static constexpr int8_t  CELL_FREE     = -30;
    static constexpr int8_t  CELL_OCCUPIED =  30;
    static constexpr int8_t  CELL_UNKNOWN  =   0;

    // ── Log-odds update stappen ───────────────────────────────
    static constexpr int8_t  L_OCC   =  10;
    static constexpr int8_t  L_FREE  =  -5;
    static constexpr int8_t  L_MIN   = -100;
    static constexpr int8_t  L_MAX   =  100;

    // ── Constructor ───────────────────────────────────────────
    // width/height in cellen; resolution in meter/cel (bv. 0.05 = 5 cm)
    GridMap(int width, int height, float resolution = 0.05f);

    // ── Kernfuncties ─────────────────────────────────────────
    bool UpdateCell(int cellX, int cellY, bool occupied);
    void RaycastUpdate(int x0, int y0, int x1, int y1);
    void IntegrateScan(float robotX, float robotY, float robotTheta,
                       const float angles[], const float ranges[], int count,
                       float maxRange = 12000.0f);
    bool IsPathValid(Path path) const;

    // ── Queries ───────────────────────────────────────────────
    bool  IsOccupied(int cellX, int cellY) const;
    bool  IsFree    (int cellX, int cellY) const;
    bool  IsUnknown (int cellX, int cellY) const;
    float GetCoveragePercent() const;

    // ── Coördinaten conversie ─────────────────────────────────
    void WorldToCell(float wx, float wy, int& cx, int& cy) const;
    void CellToWorld(int   cx, int   cy, float& wx, float& wy) const;
    bool InBounds(int cx, int cy) const;

    // ── Grid-toegang ──────────────────────────────────────────
    const std::vector<std::vector<int8_t>>& GetLogOddsGrid() const;
    const std::vector<std::vector<int>>&    GetGrid() const;

    int   GetWidth()      const { return width;     }
    int   GetHeight()     const { return height;    }
    float GetResolution() const { return resolution; }

    void Clear();

    // ── PGM export ────────────────────────────────────────────
    // Standaard: volledige kaart opslaan
    bool SavePGM(const std::string& filename) const;

    // Bijgesneden PGM: alleen het verkende gebied, altijd gecentreerd,
    // met schaalbalken op het canvas (pixels per meter automatisch berekend).
    // margin_m = witruimte rondom het verkende gebied in meter (standaard 0.5m)
    bool SavePGMCropped(const std::string& filename, float margin_m = 0.5f) const;

private:
    int   width;
    int   height;
    float resolution;
    float originX;
    float originY;

    std::vector<std::vector<int8_t>> logOdds;
    mutable std::vector<std::vector<int>> binaryGrid;
    mutable bool binaryDirty;

    void RebuildBinaryGrid() const;
    void ClampLogOdds(int8_t& val) const;
    void BresenhamLine(int x0, int y0, int x1, int y1,
                       std::vector<std::pair<int,int>>& cells) const;
};

#endif // GRIDMAP_H