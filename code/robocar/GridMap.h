#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <cstdint>
#include <string>
#include "Path.h"

// ═══════════════════════════════════════════════════════════════
//  GridMap  –  2D Occupancy Grid met log-odds updates
//
//  Elke cel slaat een log-odds waarde op (int8_t, -127..127).
//  Conversie:  p = 1 - 1/(1+exp(l))
//
//  Coördinaten:  (cellX, cellY) = (worldX/resolution, worldY/resolution)
//  Oorsprong van de kaart zit in het midden van het grid.
// ═══════════════════════════════════════════════════════════════

class GridMap {
public:
    // ── Cellstatussen ────────────────────────────────────────
    static constexpr int8_t  CELL_FREE     = -30;   // log-odds drempel vrij
    static constexpr int8_t  CELL_OCCUPIED =  30;   // log-odds drempel bezet
    static constexpr int8_t  CELL_UNKNOWN  =   0;   // onbekend

    // ── Log-odds update stappen ───────────────────────────────
    static constexpr int8_t  L_OCC   =  10;   // +10 per bezette meting
    static constexpr int8_t  L_FREE  =  -5;   // -5  per vrij-meting
    static constexpr int8_t  L_MIN   = -100;
    static constexpr int8_t  L_MAX   =  100;

    // ── Constructor ───────────────────────────────────────────
    // width/height in cellen; resolution in meter/cel (bv. 0.05 = 5 cm)
    GridMap(int width, int height, float resolution = 0.05f);

    // ═══════════════════════ Kernfuncties ════════════════════

    // Markeer één cel als bezet (+L_OCC) of vrij (+L_FREE)
    bool UpdateCell(int cellX, int cellY, bool occupied);

    // Update het pad van (x0,y0) naar (x1,y1) via Bresenham ray-cast
    // Tussenliggende cellen → vrij, eindcel → bezet
    void RaycastUpdate(int x0, int y0, int x1, int y1);

    // Verwerk een volledige LIDAR-scan in één aanroep
    // robotX/Y/Theta in wereldcoördinaten (m), angles/ranges in graden/mm
    void IntegrateScan(float robotX, float robotY, float robotTheta,
                       const float angles[], const float ranges[], int count,
                       float maxRange = 12000.0f);

    // Controleer of een pad vrij is
    bool IsPathValid(Path path) const;

    // ═══════════════════════ Queries ═════════════════════════

    bool IsOccupied(int cellX, int cellY) const;
    bool IsFree    (int cellX, int cellY) const;
    bool IsUnknown (int cellX, int cellY) const;

    // Percentage bezette + vrije cellen (voor GetCoverage)
    float GetCoveragePercent() const;

    // Conversiehulp: wereld (m) ↔ cel
    void  WorldToCell(float wx, float wy, int& cx, int& cy) const;
    void  CellToWorld(int   cx, int   cy, float& wx, float& wy) const;

    bool  InBounds(int cx, int cy) const;

    // Directe toegang (voor A*-planner / visualisatie)
    const std::vector<std::vector<int8_t>>& GetLogOddsGrid() const;

    // Compatibiliteitslaag met het originele GridMap-interface
    const std::vector<std::vector<int>>& GetGrid() const;   // 0=vrij, 1=bezet

    int   GetWidth()      const { return width;      }
    int   GetHeight()     const { return height;      }
    float GetResolution() const { return resolution;  }

    // Reset alle cellen naar onbekend
    void  Clear();

    // Sla de kaart op als PGM-afbeelding (voor debug)
    bool  SavePGM(const std::string& filename) const;

private:
    int   width;
    int   height;
    float resolution;   // meter per cel
    float originX;      // wereldcoördinaat van cel (0,0), in meter
    float originY;

    // Log-odds grid: rij = Y, kolom = X
    std::vector<std::vector<int8_t>> logOdds;

    // Compatibiliteitsgrid (lazy update)
    mutable std::vector<std::vector<int>> binaryGrid;
    mutable bool binaryDirty;

    void RebuildBinaryGrid() const;
    void ClampLogOdds(int8_t& val) const;

    // Bresenham helper
    void BresenhamLine(int x0, int y0, int x1, int y1,
                       std::vector<std::pair<int,int>>& cells) const;
};

#endif // GRIDMAP_H