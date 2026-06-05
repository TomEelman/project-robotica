#include "GridMap.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <stdexcept>

// 3x5 bitmap-font voor de cijfers 0-9 (debug-kaart rendering). Op
// bestandsniveau gedefinieerd zodat beide teken-helpers eronder hem zien.
static const uint8_t kDigitFont[10][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111}, // 0
    {0b010, 0b110, 0b010, 0b010, 0b111}, // 1
    {0b111, 0b001, 0b111, 0b100, 0b111}, // 2
    {0b111, 0b001, 0b111, 0b001, 0b111}, // 3
    {0b101, 0b101, 0b111, 0b001, 0b001}, // 4
    {0b111, 0b100, 0b111, 0b001, 0b111}, // 5
    {0b111, 0b100, 0b111, 0b101, 0b111}, // 6
    {0b111, 0b001, 0b001, 0b001, 0b001}, // 7
    {0b111, 0b101, 0b111, 0b101, 0b111}, // 8
    {0b111, 0b101, 0b111, 0b001, 0b111}, // 9
};

GridMap::GridMap(int w, int h, float res)
    : width(w)
    , height(h)
    , resolution(res)
    , originX(-static_cast<float>(w) * res * 0.5f)
    , originY(-static_cast<float>(h) * res * 0.5f)
    , logOdds(h, std::vector<int8_t>(w, CELL_UNKNOWN))
    , binaryGrid(h, std::vector<int>(w, 0))
    , binaryDirty(false)
{}

void GridMap::RaycastUpdate(int x0, int y0, int x1, int y1) {
    // fast thread local cache array to hold the cells along the line path
    static thread_local std::vector<std::pair<int,int>> cells;
    bresenhamLine(x0, y0, x1, y1, cells);

    // Step 1: Loop through all cells EXCEPT the final one to mark them as empty space
    for (size_t i = 0; i + 1 < cells.size(); ++i) {
        auto [cx, cy] = cells[i];
        if (!InBounds(cx, cy)) continue;

        // subtract points because the laser passed through here safely without hitting anything
        int8_t& cell = logOdds[cy][cx];
        cell = static_cast<int8_t>(cell + L_FREE);
        clampLogOdds(cell);
    }

    // Step 2: The very last cell in the vector represents the solid obstacle hit
    if (!cells.empty()) {
        auto [cx, cy] = cells.back();
        if (InBounds(cx, cy)) {
            // add points because the laser actively collided with a wall here
            int8_t& cell = logOdds[cy][cx];
            cell = static_cast<int8_t>(cell + L_OCC);
            clampLogOdds(cell);
        }
    }

    binaryDirty = true;
}

void GridMap::IntegrateScan(float robotX, float robotY, float robotTheta,
                             const float angles[], const float ranges[], int count, float maxRange) {
    // conversion factors localization delivers data in mm, but map operations require meters
    constexpr float MM2M    = 0.001f;
    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    float robotX_m = robotX * MM2M;
    float robotY_m = robotY * MM2M;

    // convert the robot's physical position to starting grid cell coordinates
    int rx;
    int ry;
    WorldToCell(robotX_m, robotY_m, rx, ry);
    
    // save this position to our trail array to show where the robot drove
    robotPath.emplace_back(robotX_m, robotY_m);

    // loop through every single laser beam returned in this sweep
    for (int i = 0; i < count; ++i) {
        float dist_mm = ranges[i];

        // skip reading points that are out of bounds
        if (dist_mm <= 0.0f || dist_mm > maxRange) continue;

        float dist_m = dist_mm * MM2M;

        // World angle = robot angle orientation + relative angle of this specific laser stream
        float globalAngle = robotTheta * DEG2RAD + angles[i] * DEG2RAD;

        // use trigonometry to find the exact target endpoint coordinates in meters
        float wx = robotX_m + dist_m * std::cos(globalAngle);
        float wy = robotY_m + dist_m * std::sin(globalAngle);

        // convert the calculated target endpoint into cell index coordinates
        int ex; 
        int ey;
        WorldToCell(wx, wy, ex, ey);

        // fire the raycast updates between start position and endpoint targets
        RaycastUpdate(rx, ry, ex, ey);
    }
}

void GridMap::IntegrateScanMotionCorrected(
    float robotX, float robotY, float robotThetaDeg, float omegaDegS, float scanDuration,
    const float angles[], const float ranges[], int count, float maxRange) {

    // conversion factors: localization values are in mm, but map operations require meters
    constexpr float MM2M    = 0.001f;
    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    // convert the robot's physical position to starting grid cell coordinates
    float robotX_m = robotX * MM2M;
    float robotY_m = robotY * MM2M;

    // convert the robot's physical position to starting grid cell coordinates
    int rx;
    int ry;
    WorldToCell(robotX_m, robotY_m, rx, ry);
    
    // save this position to our trail array to show where the robot drove
    robotPath.emplace_back(robotX_m, robotY_m);

    // find the exact midpoint time of the scan to use as our stationary anchor reference
    float halfDuration = scanDuration * 0.5f;

    // loop through every single laser beam returned in this sweep
    for (int i = 0; i < count; ++i) {
        float dist_mm = ranges[i];

        // skip reading points that are out of bounds
        if (dist_mm <= 0.0f || dist_mm > maxRange) continue;

        // calculate the timestamp of this specific beam (0 = start of scan, scanDuration = end)
        float t_i = (static_cast<float>(i) / static_cast<float>(count)) * scanDuration;

        // calculate exactly how many degrees the robot had turned when this beam fired
        // (t_i - halfDuur) gives the time offset relative to the middle of the scan
        float thetaCorrected = robotThetaDeg - omegaDegS * (t_i - halfDuration);

        // World angle = untwisted robot orientation + relative angle of this specific laser stream
        float globalAngle = thetaCorrected * DEG2RAD + angles[i] * DEG2RAD;

        // use trigonometry to find the exact target endpoint coordinates in meters
        float dist_m = dist_mm * MM2M;
        float wx = robotX_m + dist_m * std::cos(globalAngle);
        float wy = robotY_m + dist_m * std::sin(globalAngle);

        // convert the calculated target endpoint into cell index coordinates
        int ex; 
        int ey;
        WorldToCell(wx, wy, ex, ey);

        // fire the raycast updates between start position and endpoint targets
        RaycastUpdate(rx, ry, ex, ey);
    }
}

bool GridMap::IsOccupied(int cx, int cy) const {
    // if cell is completely outside the grid it's not a valid wall
    if (!InBounds(cx, cy)) return false;
    
    // returns true if the probability score has crossed above our wall certainty threshold
    return logOdds[cy][cx] >= CELL_OCCUPIED;
}

bool GridMap::IsFree(int cx, int cy) const {
    // if cell is outside the grid it's not clear to drive through
    if (!InBounds(cx, cy)) return false;
    
    // returns true if the score is low enough to prove the cell is empty path space
    return logOdds[cy][cx] <= CELL_FREE;
}

bool GridMap::IsUnknown(int cx, int cy) const {
    // if cell is outside the map boundaries treat it as unmapped/unknown
    if (!InBounds(cx, cy)) return true;
    
    // grab the raw cell probability score
    int8_t v = logOdds[cy][cx];
    
    // returns true if the score is stuck in the gray area between "definitely free" and "definitely wall"
    return (v > CELL_FREE && v < CELL_OCCUPIED);
}

void GridMap::GetRoomCoverage(float& outerWallPct, float& interiorPct, float& relCoveragePct) const {
    // border thickness (5 cells deep) to capture peripheral walls
    constexpr int BAND = 5; 

    // Step 1: Find the bounding limits of the explored region
    int minX = width,  maxX = -1;
    int minY = height, maxY = -1;

    // loop through the entire map to find the edges of where we have actually scanned
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!IsUnknown(x, y)) {
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }
    }

    // if nothing has been mapped yet return clean zeroes and exit
    if (maxX < 0) {
        outerWallPct = interiorPct = relCoveragePct = 0.0f;
        return;
    }

    // calculate the width and height of our active bounding box canvas
    int bboxW = maxX - minX + 1;
    int bboxH = maxY - minY + 1;

    // PHASE 2: Calculate overall relative coverage inside the active canvas area
    int bboxTotal = bboxW * bboxH;
    int bboxKnown = 0;

    // look through only the bounding box region to count how many cells are explored
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            if (!IsUnknown(x, y)) ++bboxKnown;
        }
    }
    
    // relative coverage percentage
    relCoveragePct = 100.0f * static_cast<float>(bboxKnown) / static_cast<float>(bboxTotal);

    // Step 3: Segregate the outer ring (walls) from the inner space (interior room floor)
    int outerTotal = 0;
    int outerOccupied = 0;
    int innerTotal = 0;
    int innerKnown   = 0;

    // loop through the bounding box cells again to separate them into bands
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            // checks if this cell sits within 5 blocks of any outer bounding boundary
            bool inBand = (x < minX + BAND || x > maxX - BAND ||
                           y < minY + BAND || y > maxY - BAND);

            // if it sits on the outer edges, evaluate it as a wall candidate
            if (inBand) {
                ++outerTotal;
                if (IsOccupied(x, y)) ++outerOccupied; // count confirmed walls
            } 
            // if it's completely inside, evaluate it as explored interior floor space
            else {
                ++innerTotal;
                if (!IsUnknown(x, y)) ++innerKnown;   // count mapped floor space
            }
        }
    }

    // calculate final percentage ratios safely protecting against zero division bugs
    if (outerTotal > 0) {
        outerWallPct = 100.0f * static_cast<float>(outerOccupied) / static_cast<float>(outerTotal);
    } else {
        outerWallPct = 0.0f;
    }

    // if the bounding box was too small to even have an interior, consider the floor 100% complete
    if (innerTotal > 0) {
        interiorPct = 100.0f * static_cast<float>(innerKnown) / static_cast<float>(innerTotal);
    } else {
        interiorPct = 100.0f;
    }
}

void GridMap::WorldToCell(float wx, float wy, int& cx, int& cy) const {
    // subtract the origin anchor offset, divide by cell scale size, and drop decimals to get the X column index
    cx = static_cast<int>((wx - originX) / resolution);
    
    // subtract the origin anchor offset, divide by cell scale size, and drop decimals to get the Y row index
    cy = static_cast<int>((wy - originY) / resolution);
}

void GridMap::CellToWorld(int cx, int cy, float& wx, float& wy) const {
    // adding 0.5f targets the exact physical center of the cell rather than its bottom-left edge
    wx = originX + (static_cast<float>(cx) + 0.5f) * resolution;
    wy = originY + (static_cast<float>(cy) + 0.5f) * resolution;
}

bool GridMap::InBounds(int cx, int cy) const {
    // returns true only if BOTH coordinates fall securely between 0 and the map's width/height limits
    return cx >= 0 && cx < width && cy >= 0 && cy < height;
}

void GridMap::Clear() {
    // loop through the master log-odds 2D array and fill every row with 0 (unexplored/unknown territory)
    for (auto& row : logOdds) {
        std::fill(row.begin(), row.end(), CELL_UNKNOWN);
    }
    
    // clear out the history of where the robot has previously driven
    robotPath.clear();
    
    // erase the destination path list targets from memory
    waypointList.clear();
    
    binaryDirty = true;
}

void GridMap::SetWaypoints(const std::vector<std::pair<float,float>>& wps_m) {
    // overwrite the active waypoint coordinates list with a new set of metric targets
    waypointList = wps_m;
}

bool GridMap::SavePGM(const std::string& filename) const {
    // open the file in binary write mode
    std::ofstream f(filename, std::ios::binary);

    // check if file was created
    if (!f) {
        return false;
    }

    // write the standard PGM header metadata: 
    // "P5" sets it as a binary greyscale image, followed by width, height, and max pixel brightness (255)
    f << "P5\n" << width << " " << height << "\n255\n";

    // loop backwards through the rows (top-to-bottom) because image files treat (0,0) as the top-left corner,
    // whereas standard Cartesian mapping coordinates treat (0,0) as the bottom-left corner
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            // pull the raw 8-bit probability score for this specific cell
            int8_t v = logOdds[y][x];
            uint8_t px;

            // Wall : black
            if (v >= CELL_OCCUPIED) {
                px = 0;
            }
            // Pathway : white
            else if (v <= CELL_FREE) {
                px = 255;
            } 
            // Unexplored : gray
            else {
                px = 127;
            }
            
            // drop the calculated grayscale byte directly into the image file stream
            f.put(static_cast<char>(px));
        }
    }
    return true;
}

// 3x5 pixel font bitmaps for digits 0-9 (used by SavePGMCropped for coordinate labels)
static const uint8_t kDigitFont[10][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111}, // 0
    {0b010, 0b110, 0b010, 0b010, 0b111}, // 1
    {0b111, 0b001, 0b111, 0b100, 0b111}, // 2
    {0b111, 0b001, 0b111, 0b001, 0b111}, // 3
    {0b101, 0b101, 0b111, 0b001, 0b001}, // 4
    {0b111, 0b100, 0b111, 0b001, 0b111}, // 5
    {0b111, 0b100, 0b111, 0b101, 0b111}, // 6
    {0b111, 0b001, 0b001, 0b001, 0b001}, // 7
    {0b111, 0b101, 0b111, 0b101, 0b111}, // 8
    {0b111, 0b101, 0b111, 0b001, 0b111}, // 9
};

bool GridMap::SavePGMCropped(const std::string& filename, float margin_m) const {
    // Step 1: Scan the grid to calculate the bounding limits of all explored territory
    int minX = width,  maxX = -1;
    int minY = height, maxY = -1;
    std::vector<std::pair<int,int>> wallPts;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // skip cells the robot hasn't seen yet
            if (IsUnknown(x, y)) continue;
            
            // expand the bounding box to encapsulate this known cell
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            
            // if this cell is actively flagged as a solid wall, collect its coordinates
            if (IsOccupied(x, y)) {
                wallPts.emplace_back(x, y);
            }
        }
    }

    // if the map is completely blank, save a standard uncropped layout instead
    if (maxX < 0) {
        return SavePGM(filename);
    }

    // Step 2: Convert the real-world meter margin request into an equivalent count of grid cells
    int mg = static_cast<int>(margin_m / resolution) + 1;

    // pad out our window limits while guaranteeing they do not cross past the true array borders
    int x0 = std::max(0,      minX - mg);
    int y0 = std::max(0,      minY - mg);
    int x1 = std::min(width,  maxX + mg + 1);
    int y1 = std::min(height, maxY + mg + 1);
    
    // calculate the width and height dimensions of our cropped grid canvas box
    int cW = x1 - x0;
    int cH = y1 - y0;

    // Step 3: assign every single grid cell to span across 3x3 high-fidelity image pixels
    constexpr int SCALE = 3;
    int imgW = cW * SCALE;
    int imgH = cH * SCALE;

    using Pt = std::pair<int,int>;
    {
        std::vector<Pt> kept;
        kept.reserve(wallPts.size());
        
        // Loop through the entire list of wall points using a traditional index loop
        for (size_t i = 0; i < wallPts.size(); ++i) {
            Pt currentPoint = wallPts[i];
            int currentX = currentPoint.first;
            int currentY = currentPoint.second;
            
            bool hasNeighbor = false;
            
            // Check a 3x3 window around the current wall cell
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    
                    // Skip checking the center cell itself (where dx == 0 and dy == 0)
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    
                    // If we already found a neighbor in a previous iteration, stop checking further
                    if (hasNeighbor) {
                        continue;
                    }
                    
                    // Look up the neighboring cell coordinates
                    int neighborX = currentX + dx;
                    int neighborY = currentY + dy;
                    
                    // If a valid touching wall cell is found, mark our flag
                    if (IsOccupied(neighborX, neighborY)) {
                        hasNeighbor = true;
                    }
                }
            }
            
            // If the wall pixel has at least one touching neighbor, it's valid structural data
            if (hasNeighbor) {
                kept.push_back(currentPoint);
            }
        }
        
        // If our filtered list isn't empty, overwrite the original array with our cleaned dataset
        if (!kept.empty()) {
            wallPts.swap(kept);
        }
    }

    // if no walls were discovered fallback to using the corners of our grid bounding box
    if (wallPts.empty()){
        wallPts = { {minX, minY}, {maxX, minY}, {minX, maxY}, {maxX, maxY} };
    }

    // CONVEX HULL GENERATION (Andrew's Monotone Chain Math)
    std::sort(wallPts.begin(), wallPts.end());
    wallPts.erase(std::unique(wallPts.begin(), wallPts.end()), wallPts.end());
    
    auto cross = [](const Pt& o, const Pt& a, const Pt& b) {
        return static_cast<double>(a.first - o.first) * (b.second - o.second)
             - static_cast<double>(a.second - o.second) * (b.first - o.first);
    };

    int np = static_cast<int>(wallPts.size());
    std::vector<Pt> hull(2 * np);
    int k = 0;

    // assemble the bottom segment of our enclosing convex hull skin
    for (int i = 0; i < np; ++i) {                
        while (k >= 2 && cross(hull[k-2], hull[k-1], wallPts[i]) <= 0) {
            --k;
        }
        hull[k++] = wallPts[i];
    }
    // assemble the top segment of our enclosing convex hull skin
    for (int i = np - 2, t = k + 1; i >= 0; --i) { 
        while (k >= t && cross(hull[k-2], hull[k-1], wallPts[i]) <= 0) {
            --k;
        }
        hull[k++] = wallPts[i];
    }
    hull.resize(std::max(1, k - 1));

    // test every edge of the hull to find the orientation that fits tightest
    double bestArea = 1e30, ux = 1, uy = 0;
    double bMinU = 0, bMaxU = 0, bMinV = 0, bMaxV = 0;
    int H = static_cast<int>(hull.size());
    
    for (int i = 0; i < H; ++i) {
        double ex = hull[(i+1) % H].first  - hull[i].first;
        double ey = hull[(i+1) % H].second - hull[i].second;
        double len = std::hypot(ex, ey);
        if (len < 1e-9) continue;
        
        double dux = ex / len, duy = ey / len;     // primary testing axis orientation alignment
        double dvx = -duy,     dvy = dux;          // secondary axis orientation (perfectly perpendicular)
        double mnU = 1e30, mxU = -1e30, mnV = 1e30, mxV = -1e30;

        // project all hull coordinate points onto the testing layout axes
        for (size_t j = 0; j < hull.size(); ++j) {
            std::pair<int, int> hullPoint = hull[j];
            int hullX = hullPoint.first;
            int hullY = hullPoint.second;
            
            // calculate the projected coordinate position along the primary layout axis (U-axis)
            double projectedU = (hullX * dux) + (hullY * duy);
            
            // calculate the projected coordinate position along the perpendicular layout axis (V-axis)
            double projectedV = (hullX * dvx) + (hullY * dvy);
            
            // update the minimum and maximum boundaries found along the U-axis track so far
            mnU = std::min(mnU, projectedU);
            mxU = std::max(mxU, projectedU);
            
            // update the minimum and maximum boundaries found along the V-axis track so far
            mnV = std::min(mnV, projectedV);
            mxV = std::max(mxV, projectedV);
        }
        
        double area = (mxU - mnU) * (mxV - mnV);
        // if this orientation wraps tighter than any previously calculated box, save it
        if (area < bestArea) {
            bestArea = area;  
            ux = dux;  
            uy = duy;
            bMinU = mnU;  bMaxU = mxU;  
            bMinV = mnV;  bMaxV = mxV;
        }
    }
    // set up secondary alignment axis components
    double vx = -uy, vy = ux;                      

    // Evaluate which structural axis aligns closest with our horizontal viewing plain
    bool uHoriz = std::abs(ux) >= std::abs(uy);

    // assign spatial direction components to our true width and height vectors based on that alignment
    float realW_m;
    float realH_m;
    double ahx, ahy, avx, avy, minH, maxH, minV, maxV;

    if (uHoriz) {
        ahx = ux;   ahy = uy;   
        avx = vx;   avy = vy;   
        minH = bMinU; maxH = bMaxU;
        minV = bMinV; maxV = bMaxV;
    } else {
        ahx = vx;   ahy = vy;   
        avx = ux;   avy = uy;   
        minH = bMinV; maxH = bMaxV;
        minV = bMinU; maxV = bMaxU;
    }

    // calculate true physical width and height metric dimensions of the room structure
    realW_m = static_cast<float>((maxH - minH) * resolution);   
    realH_m = static_cast<float>((maxV - minV) * resolution);

    // compute cell location coordinate vertex points for all 4 corners of our fitted tracking box
    auto toCellX = [&](double h, double v) { return h * ahx + v * avx; };
    auto toCellY = [&](double h, double v) { return h * ahy + v * avy; };
    double rectCellX[4] = { toCellX(minH, minV), toCellX(maxH, minV), toCellX(maxH, maxV), toCellX(minH, maxV) };
    double rectCellY[4] = { toCellY(minH, minV), toCellY(maxH, minV), toCellY(maxH, maxV), toCellY(minH, maxV) };

    int totalH = imgH;

    // STEP 5: Initialize image buffer array with a soft gray backdrop (235 RGB value)
    std::vector<uint8_t> img(static_cast<size_t>(imgW * totalH * 3), 235);

    // Rasterize grid map data - transforming each cell block into a 3x3 color block cluster
    for (int y = y1 - 1; y >= y0; --y) {
        int cellRow = (y1 - 1 - y);
        for (int x = x0; x < x1; ++x) {
            int cellCol = x - x0;
            uint8_t px;
            
            // COLOR CODING DETERMINATION
            if (!InBounds(x, y)) {
                px = 200; // dark gray canvas edges
            } else if (logOdds[y][x] >= CELL_OCCUPIED) {
                px = 0;   // pure black solid structural walls
            } else if (logOdds[y][x] <= CELL_FREE) {
                px = 255; // pure white safe clear path driving zones
            } else {
                px = 160; // medium gray unexplored territory
            }

            // populate the 3x3 pixel area in the image buffer for this grid cell
            for (int sy = 0; sy < SCALE; ++sy) {
                for (int sx = 0; sx < SCALE; ++sx) {
                    int idx = ((cellRow * SCALE + sy) * imgW + cellCol * SCALE + sx) * 3;
                    img[static_cast<size_t>(idx)]     = px; // Red channel
                    img[static_cast<size_t>(idx + 1)] = px; // Green channel
                    img[static_cast<size_t>(idx + 2)] = px; // Blue channel
                }
            }
        }
    }

    // Step 6: Overlay the historical track of the robot using a bright red color
    for (size_t i = 0; i < robotPath.size(); ++i) {
        std::pair<float, float> pathPoint = robotPath[i];
        float worldX = pathPoint.first;
        float worldY = pathPoint.second;

        int cx, cy;
        WorldToCell(worldX, worldY, cx, cy);

        // skip rendering points that fall outside our cropped frame view window
        if (cx < x0) { continue; }
        if (cx >= x1) { continue; }
        if (cy < y0) { continue; }
        if (cy >= y1) { continue; }

        // calculate the exact pixel offsets where this 3x3 block starts on the upscaled image buffer
        int col = (cx - x0) * SCALE;
        int row = (y1 - 1 - cy) * SCALE;

        // paint the SCALE x SCALE pixel area red
        for (int sy = 0; sy < SCALE; ++sy) {
            for (int sx = 0; sx < SCALE; ++sx) {
                
                // compute the flat index in our 1D RGB vector array (multiply by 3 for R, G, B channels)
                size_t idx = static_cast<size_t>((row + sy) * imgW + col + sx) * 3;
                
                // safety tracker: verify the index doesn't step past our allocated vector memory
                if (idx + 2 >= img.size()) {
                    continue;
                }

                // apply the bright crimson red color palette values
                img[idx]     = 220; // Red channel
                img[idx + 1] = 50;  // Green channel
                img[idx + 2] = 50;  // Blue channel
            }
        }
    }

    // Step 7: Render target objectives with numbered white texts packed into high-visibility green labels
    for (size_t i = 0; i < waypointList.size(); ++i) {
        auto [wx, wy] = waypointList[i];
        int cx, cy;
        WorldToCell(wx, wy, cx, cy);
        if (cx < x0 || cx >= x1 || cy < y0 || cy >= y1) continue;

        int col = (cx - x0) * SCALE;
        int row = (y1 - 1 - cy) * SCALE;

        int num    = static_cast<int>(i) + 1;

        // calculate spacing bounds - handle width extension if label digits surpass 9
        int boxW;
        if (num >= 10) {
            boxW = 9;  // 9 pixels wide for 2 digit markers
        } else {
            boxW = 7;  // 7 pixels wide for single digit markers
        }

        int bx = col - boxW / 2;
        int by = row - 3;

        // Groene achtergrond
        for (int dy = 0; dy < 7; ++dy) {
            for (int dx = 0; dx < boxW; ++dx) {
                int px = bx + dx, py = by + dy;
                if (px < 0 || px >= imgW || py < 0 || py >= imgH) continue;
                size_t idx = static_cast<size_t>(py * imgW + px) * 3;
                img[idx] = 30; img[idx+1] = 160; img[idx+2] = 30;
            }
        }

        // Write white numbers into center alignment slots inside the green labels
        if (num < 10) {
            drawDigit(img, imgW, imgH, bx + 2, by + 1, num, 255, 255, 255);
        } else {
            drawDigit(img, imgW, imgH, bx + 1, by + 1, num / 10, 255, 255, 255);
            drawDigit(img, imgW, imgH, bx + 5, by + 1, num % 10, 255, 255, 255);
        }
    }

    // STEP 8: Render technical schematic dimension annotation layouts onto the map borders
    auto fill = [&](int rx, int ry, int rw, int rh, uint8_t r, uint8_t g, uint8_t b) {
        for (int dy = 0; dy < rh; ++dy) {
            for (int dx = 0; dx < rw; ++dx) {
                int px2 = rx + dx, py = ry + dy;
                if (px2 < 0 || px2 >= imgW || py < 0 || py >= totalH) continue;
                int idx = (py * imgW + px2) * 3;
                img[static_cast<size_t>(idx)]     = r;
                img[static_cast<size_t>(idx + 1)] = g;
                img[static_cast<size_t>(idx + 2)] = b;
            }
        }
    };

    constexpr int TS = 2;                       // text magnification multiplier factor
    const uint8_t TR = 20, TG = 20, TB = 160;   // font text coloring (blueprint dark navy blue)

    // Helper lambda to print an enlarged individual number character to the image array
    auto drawDigitBig = [&](int digit, int px, int py) {
        if (digit < 0 || digit > 9) return;
        const uint8_t* rows = kDigitFont[digit];
        for (int dy = 0; dy < 5; ++dy) {
            for (int dx = 0; dx < 3; ++dx) {
                if (rows[dy] & (1 << (2 - dx))) {
                    fill(px + dx * TS, py + dy * TS, TS, TS, TR, TG, TB);
                }
            }
        }
    };

    // Calculate number of characters ahead of decimal symbol to correctly handle text center balancing
    auto wholeDigits = [](float m) {
        int whole = static_cast<int>(m);
        if (whole == 0) return 1;
        int n = 0;
        for (int w = whole; w > 0; w /= 10) ++n;
        return n;
    };
    
    auto textW = [&](float m) { return (wholeDigits(m) * 4 + 2 + 8) * TS; };
    const int textH = 5 * TS;

    // Helper lambda to print string metrics formatted as clear "0.00m" meters values inside a safe white mask block
    auto drawMeters = [&](float meters, int px, int py) {
        fill(px - 2, py - 2, textW(meters) + 3, textH + 4, 255, 255, 255);  
        int whole = static_cast<int>(meters);
        int frac  = static_cast<int>(meters * 100.0f + 0.5f) % 100;
        int digits[8], n = 0;
        
        if (whole == 0) {
            digits[n++] = 0;
        } else {
            for (int v = whole; v > 0; v /= 10) digits[n++] = v % 10;
        }
        
        for (int i = n - 1; i >= 0; --i) { 
            drawDigitBig(digits[i], px, py); 
            px += 4 * TS; 
        }
        fill(px, py + 4 * TS, TS, TS, TR, TG, TB); // render sub-unit decimal separator point symbol
        px += 2 * TS;
        drawDigitBig(frac / 10, px, py); px += 4 * TS;       
        drawDigitBig(frac % 10, px, py);                     
    };

    // Convert cell float values to absolute pixel index coordinates (flipping tracking on Y axis)
    auto colPx = [&](double cx) { return static_cast<int>((cx - x0) * SCALE); };
    auto rowPx = [&](double cy) { return static_cast<int>((y1 - 1 - cy) * SCALE); };

    // Find the extreme external pixel borders of our box layout to clear space for dimension arrows
    int pxL = imgW, pxR = 0, pyT = totalH, pyB = 0;
    for (int i = 0; i < 4; ++i) {
        int cxp = colPx(rectCellX[i]), cyp = rowPx(rectCellY[i]);
        pxL = std::min(pxL, cxp);  pxR = std::max(pxR, cxp);
        pyT = std::min(pyT, cyp);  pyB = std::max(pyB, cyp);
    }

    const uint8_t LR = 90, LG = 90, LB = 90;

    // RENDER WIDTH MEASUREMENT LINES: place directly above the structural room outline bounds
    int yDim = std::max(1, pyT - 12);
    fill(pxL, yDim, pxR - pxL + 1, 1, LR, LG, LB);          // master spanning track line
    fill(pxL, yDim - 4, 1, 9, LR, LG, LB);                  // left terminal boundary tick
    fill(pxR, yDim - 4, 1, 9, LR, LG, LB);                  // right terminal boundary tick
    drawMeters(realW_m, (pxL + pxR) / 2 - textW(realW_m) / 2, std::max(0, yDim - textH - 4));

    // RENDER HEIGHT MEASUREMENT LINES: place directly to the left side of the structural room bounds
    int xDim = std::max(1, pxL - 12);
    fill(xDim, pyT, 1, pyB - pyT + 1, LR, LG, LB);          // master spanning track line
    fill(xDim - 4, pyT, 9, 1, LR, LG, LB);                  // top terminal boundary tick
    fill(xDim - 4, pyB, 9, 1, LR, LG, LB);                  // bottom terminal boundary tick
    
    int hx = xDim - textW(realH_m) - 4;
    if (hx < 0) {
        hx = xDim + 6; // adjust layout tracking position slightly inward if left margin clearance is too tight
    }
    drawMeters(realH_m, hx, (pyT + pyB) / 2 - textH / 2);

    // Step 9: Write compiled file data stream to local storage using full color PPM (P6) specifications
    std::ofstream f2(filename, std::ios::binary);
    if (!f2) {
        return false;
    }

    f2 << "P6\n";
    f2 << "# breedte=" << realW_m << "m  hoogte=" << realH_m << "m\n";
    f2 << "# schaal: 1px=" << resolution * 100.0f << "cm  |  maten op kaart in meter\n";
    f2 << imgW << " " << totalH << "\n255\n";
    f2.write(reinterpret_cast<const char*>(img.data()), static_cast<std::streamsize>(img.size()));
    
    return true;
}

void GridMap::clampLogOdds(int8_t& val) const {
void GridMap::clampLogOdds(int8_t& val) const {
    // if the value drops below the minimum threshold, cap it at the floor limit
    if (val < L_MIN) { 
        val = L_MIN; 
    }
    
    // if the value climbs past the maximum threshold, cap it at the ceiling limit
    if (val > L_MAX) { 
        val = L_MAX; 
    }
}

void GridMap::bresenhamLine(int x0, int y0, int x1, int y1, std::vector<std::pair<int,int>>& cells) const {
    cells.clear();

    int dx =  std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        cells.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void GridMap::drawDigit(std::vector<uint8_t>& img, int imgW, int imgH,
                        int px, int py, int digit,
                        uint8_t r, uint8_t g, uint8_t b)
{
    if (digit < 0 || digit > 9) return;
    const uint8_t* rows = kDigitFont[digit];
    for (int dy = 0; dy < 5; ++dy) {
        uint8_t row = rows[dy];
        for (int dx = 0; dx < 3; ++dx) {
            if (!(row & (1 << (2 - dx)))) continue;
            int x = px + dx, y = py + dy;
            if (x < 0 || x >= imgW || y < 0 || y >= imgH) continue;
            size_t idx = static_cast<size_t>(y * imgW + x) * 3;
            img[idx] = r; img[idx+1] = g; img[idx+2] = b;
        }
    }
}