#include "ExplorationPlanner.h"
#include "Mapper.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <climits>

ExplorationPlanner::ExplorationPlanner() : currentTarget(0.0f, 0.0f, 0.0f) {}

static int BFSDistance(const GridMap& map, int sx, int sy, int gx, int gy) {
    const int W = map.GetWidth();
    const int H = map.GetHeight();

    // safety check: if start or goal is outside the map boundaries, return max distance
    if (!map.InBounds(sx, sy) || !map.InBounds(gx, gy)) return INT_MAX;
    
    // safety check: if start or goal is trapped inside a wall, return max distance
    if (map.IsOccupied(sx, sy) || map.IsOccupied(gx, gy)) return INT_MAX;
    
    // vector representing the grid initialized with -1 (unvisited)
    std::vector<int> distance(W * H, -1);

    // queue to manage which cells to explore next (First In, First Out)
    std::queue<int> q;

    // get flat index for start coordinates, set its distance to 0, and add to queue
    int startIndex = (sy * W) + sx;
    distance[startIndex] = 0;
    q.push(startIndex);

    // coordinate shift arrays to easily look Left, Right, Up, and Down
    const int dx[4] = {-1,1, 0,0};
    const int dy[4] = { 0,0,-1,1};

    // keep exploring until we run out of reachable cells
    while (!q.empty()) {
        // grab the next cell in line and pop it from the queue
        int cur = q.front(); 
        q.pop();

        // decode the flat 1D index back into 2D (X, Y) coordinates
        int cx = cur % W;
        int cy = cur / W;

        // get the current step count for this cell
        int d  = distance[cur];

        // SUCCESS: if we reached the goal cell, return the total step distance!
        if (cx == gx && cy == gy) return d;

        // look at all 4 neighboring cells (Left, Right, Up, Down)
        for (int i = 0; i < 4; ++i) {
            int neighborX = cx + dx[i];
            int neighborY = cy + dy[i];
            
            // if neighbor is outside the map boundaries, skip it
            if (!map.InBounds(neighborX, neighborY)) continue;
            
            // if neighbor is a solid wall block, skip it
            if (map.IsOccupied(neighborX, neighborY)) continue;

            // get the neighbor's flat vector index
            int neighborIndex = (neighborY * W) + neighborX;
            
            // if we have already calculated a distance for this cell, skip it
            if (distance[neighborIndex] >= 0) continue;
            
            // save neighbor's distance as current distance + 1 step, then queue it up
            distance[neighborIndex] = d + 1;
            q.push(neighborIndex);
        }    

    }

    return INT_MAX;  // unreachable
}

void AddToBlackList(std::vector<BlacklistItem>& blacklist, float x_mm, float y_mm, int ttl)
{
    // loops through the existing blacklist to check if this location is already known
    for (size_t i = 0; i < blacklist.size(); ++i) {
        float dx = blacklist[i].x_mm - x_mm;
        float dy = blacklist[i].y_mm - y_mm;
        
        // calculates squared distance to see how close this point is to an existing item
        float d2 = (dx * dx) + (dy * dy);
        
        // if the point is within 200mm, we treat it as the exact same target area
        if (d2 < 200.0f * 200.0f) {   
            // update the item's expiration timer
            blacklist[i].ttl = std::max(blacklist[i].ttl, ttl);
            
            // exit early since we updated the existing spot and don't need a new entry
            return;
        }
    }
    
    // if the loop finishes without finding a close match, add this as a brand-new entry to the blacklist
    blacklist.push_back({x_mm, y_mm, ttl});
}

void TickBlacklist(std::vector<BlacklistItem>& blacklist) {
    // Step 1: loop through the blacklist and decrease the time to live for every item
    for (size_t i = 0; i < blacklist.size(); ++i) {
        --blacklist[i].ttl;
    }

    // Step 2: loop backwards to safely remove any expired items (ttl <= 0)
    for (int i = static_cast<int>(blacklist.size()) - 1; i >= 0; --i) {
        if (blacklist[i].ttl <= 0) {
            // erases this specific item from the vector list
            blacklist.erase(blacklist.begin() + i);
        }
    }
}

static bool inBlacklist(const std::vector<BlacklistItem>& blacklist, float x_mm, float y_mm) {
    // loops through the entire blacklist to check if our coordinates match an excluded zone
    for (size_t i = 0; i < blacklist.size(); ++i) {
        float dx = blacklist[i].x_mm - x_mm;
        float dy = blacklist[i].y_mm - y_mm;
        
        // calculates squared distance to find how close the target is to this blacklist item
        float d2 = (dx * dx) + (dy * dy);
        
        // if the target is within the 200mm radius threshold of a blacklisted spot
        if (d2 < 200.0f * 200.0f) {
            // match found and return true immediately to flag that this spot is blocked
            return true;
        }
    }
    
    // if the loop finishes checking every item and finds nothing nearby, the spot is safe
    return false;
}

Position ChooseFrontierGoal(const Mapper& mapper, const Position& currentPos, const float lidarRanges[360], 
    const std::vector<BlacklistItem>& blacklist) {
    //  mm to meter conversion
    constexpr float MM2M       = 0.001f;
    // skip every 3 cells to scan the map much faster with plenty of precision
    constexpr int   SAMPLE_STAP = 3;     
    // ignore any frontier cells further than 400 grid blocks away
    constexpr float MAX_BFS     = 400.0f; 

    const GridMap& map = mapper.GetMap();
    const float currentX_m  = currentPos.GetX() * MM2M;
    const float currentY_m  = currentPos.GetY() * MM2M;
    const int W = map.GetWidth();
    const int H = map.GetHeight();

    int rx;
    int ry;
    map.WorldToCell(currentX_m, currentY_m, rx, ry);

    // layout coordinate shifts to quickly look at 4 neighboring directions (Left, Right, Up, Down)
    const int ddx4[4] = {-1, 1,  0, 0};
    const int ddy4[4] = { 0, 0, -1, 1};

    // trackers to keep record of the highest scoring goal found so far
    float bestScore = -1.0f;
    float bestWx_m  = currentX_m;
    float bestWy_m  = currentY_m;

    // Step 1: Loop through the grid map row by row, jumping by the sample step size
    for (int cy = 0; cy < H; cy += SAMPLE_STAP) {
        for (int cx = 0; cx < W; cx += SAMPLE_STAP) {
            
            // if this specific grid cell is blocked by a wall or obstacle, skip it
            if (!map.IsFree(cx, cy)) continue;

            // check if this free cell qualifies as a "frontier" by bordering unmapped space
            bool hasUnknownNeighbor = false;

            for (int d = 0; d < 4; ++d) {
                int neighborX = cx + ddx4[d];
                int neighborY = cy + ddy4[d];

                if (map.InBounds(neighborX, neighborY) && map.IsUnknown(neighborX, neighborY)) {
                    hasUnknownNeighbor = true; 
                    break;
                }
            }

            // if all surrounding neighbors are already fully mapped, skip this cell
            if (!hasUnknownNeighbor) continue;

            // translate cell coordinates back to real-world meters
            float wx_m;
            float wy_m;

            map.CellToWorld(cx, cy, wx_m, wy_m);

            // check if this area falls within our blacklist
            if (inBlacklist(blacklist, wx_m / MM2M, wy_m / MM2M)) continue;

            // Step 2: Use pathfinding to see how many actual steps it takes to walk here
            int bfsDistance = BFSDistance(map, rx, ry, cx, cy);
            if (bfsDistance == INT_MAX) continue;         // unreachable path -> skip
            if (bfsDistance > (int)MAX_BFS) continue;     // too far away -> skip

            // Step 3: Lidar check Calculate the angle relative to the robot's current heading
            float ddxF    = wx_m - currentX_m;
            float ddyF    = wy_m - currentY_m;
            float angle_rad = std::atan2(ddyF, ddxF) - (currentPos.GetTheta() * static_cast<float>(M_PI) / 180.0f);
            int   lidarIdx  = ((int)(angle_rad * 180.0f / static_cast<float>(M_PI)) + 360) % 360;
            
            // extract how much physical clear space the lidar sensor spots in that direction
            float freeRoom = lidarRanges[lidarIdx];
            if (freeRoom <= 0.0f || freeRoom > 8000.0f) freeRoom = 8000.0f;

            // do not pick a target that is too close (under 30cm) to the robot
            float dist_m = std::sqrt(ddxF*ddxF + ddyF*ddyF);
            if (dist_m < 0.3f) continue;

            // Step 4: Information gain calculation Count unknown cells in a 21x21 square box around the target
            constexpr int GAIN_R = 10;
            int unknownCount = 0; 
            int totalCount = 0;

            for (int gy = cy - GAIN_R; gy <= cy + GAIN_R; gy += 2) {
                for (int gx = cx - GAIN_R; gx <= cx + GAIN_R; gx += 2) {
                    if (!map.InBounds(gx, gy)) continue;
                    ++totalCount;
                    if (map.IsUnknown(gx, gy)) ++unknownCount;
                }
            }
            
            // calculate the percentage of unexplored space this target would open up
            float scoreGain = totalCount > 0 ? static_cast<float>(unknownCount) / static_cast<float>(totalCount) : 0.0f;

            // Step 5: Normalize component scores between 0.0 and 1.0
            float scoreReach = 1.0f - std::min((float)bfsDistance, MAX_BFS) / MAX_BFS; // closer pathways score higher
            float scoreLidar = std::min(freeRoom, 2000.0f) / 2000.0f; // more open space scores higher

            // Step 6: Combine scores using a weighted formula (40% Reachability, 30% Information Gain, 30% Open Space)
            float score = 0.4f * scoreReach + 0.3f * scoreGain + 0.3f * scoreLidar;

            // if this spot achieves the highest overall rating, crown it as our new target
            if (score > bestScore) {
                bestScore = score;
                bestWx_m  = wx_m;
                bestWy_m  = wy_m;
            }
        }
    }

    // return the winning world coordinate destination converted back into millimeters
    return Position(bestWx_m / MM2M, bestWy_m / MM2M, 0.0f);
}