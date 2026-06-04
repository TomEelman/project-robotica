#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <cstdint>
#include <string>
#include "Path.h"

class GridMap {
public:
    // Log-Odds thresholds: weights added or subtracted when a laser ray hits or misses a cell
    static constexpr int8_t  L_OCC   =   5;   // add 5 points if a laser hits a wall here
    static constexpr int8_t  L_FREE  =  -4;   // subtract 4 points if a laser passes clean through here
    static constexpr int8_t  L_MIN   = -60;   // the absolute floor score (caps the certainty of empty space)
    static constexpr int8_t  L_MAX   =  60;   // the absolute ceiling score (caps the certainty of solid walls)

    // Certainty barriers: how a cell's numerical score is translated into a state
    static constexpr int8_t  CELL_FREE     = -20; // any score under -20 means the cell is definitely empty
    static constexpr int8_t  CELL_OCCUPIED =  20; // any score over 20 means the cell is definitely a wall
    static constexpr int8_t  CELL_UNKNOWN  =   0; // a score of 0 means the robot hasn't explored here yet

    GridMap(int width, int height, float resolution = 0.03f);

    // updates a straight path of cells between two points using Bresenham's raycasting math
    void RaycastUpdate(int x0, int y0, int x1, int y1);

    // drops a raw laser scan sweep into the map to update wall and empty space probabilities
    void IntegrateScan(
        float robotX, float robotY, float robotTheta, const float angles[], 
        const float ranges[], int count, float maxRange = 12000.0f);

    // fixes laser smear errors if the robot was spinning while scanning
    void IntegrateScanMotionCorrected(
        float robotX, float robotY, float robotThetaDeg, float omegaDegS, float scanDuurSec,
        const float angles[], const float ranges[], int count, float maxRange = 12000.0f);

    bool  IsOccupied(int cellX, int cellY) const; // returns true if the cell is blocked by a wall
    bool  IsFree    (int cellX, int cellY) const; // returns true if the cell is clear to drive through
    bool  IsUnknown (int cellX, int cellY) const; // returns true if the cell hasn't been scanned yet
    
    float GetCoveragePercent() const; // returns total map exploration percentage
    void  GetRoomCoverage(float& outerWallPct, float& interiorPct, float& relCoveragePct) const;

    // converts real world coordinates (meters) to 2D grid index coordinates (cells)
    void WorldToCell(float wx, float wy, int& cx, int& cy) const;
    // converts 2D grid index coordinates (cells) back into real world coordinates (meters)
    void CellToWorld(int   cx, int   cy, float& wx, float& wy) const;
    // returns true if the cell coordinates fall inside the actual map grid
    bool InBounds(int cx, int cy) const;

    // Fast inline getter methods to fetch basic map details
    int   GetWidth()      const { return width;     }
    int   GetHeight()     const { return height;    }
    float GetResolution() const { return resolution; }

    void Clear();

    void SetWaypoints(const std::vector<std::pair<float,float>>& wps_m);
    bool SavePGM(const std::string& filename) const;
    bool SavePGMCropped(const std::string& filename, float margin_m = 0.5f) const;

private:
    int   width;      // width of the map grid in cells
    int   height;     // height of the map grid in cells
    float resolution; // size of each cell side in meters (e.g., 0.05 = 5cm)
    float originX;    // real-world X anchor coordinate for the (0,0) corner cell
    float originY;    // real-world Y anchor coordinate for the (0,0) corner cell
    std::vector<std::vector<int8_t>> logOdds; // 2D vector array storing the raw log odds scores
    mutable std::vector<std::vector<int>> binaryGrid;
    mutable bool binaryDirty; 

    std::vector<std::pair<float,float>> robotPath;     // trace history of where the robot has driven
    std::vector<std::pair<float,float>> waypointList;  // target list of coordinates to visit

    void clampLogOdds(int8_t& val) const;
    void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<std::pair<int,int>>& cells) const;
    static void drawDigit(std::vector<uint8_t>& img, int imgW, int imgH, int px, int py, int digit,
                            uint8_t r, uint8_t g, uint8_t b);
};

#endif