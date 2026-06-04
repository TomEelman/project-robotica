#include "Mapper.h"
#include <iostream>
#include <cstdio>
#include <vector>

Mapper::Mapper(int width, int height, float resolution)
    : map(width, height, resolution), updated(false)
{}

// update map while not moving
void Mapper::Update(const float ranges[], const float angles[], int count, const Position& position) {
    map.IntegrateScan(
        position.GetX(),
        position.GetY(),
        position.GetTheta(),
        angles, ranges, count
    );

    updated = true;
}

// update map while moving
void Mapper::UpdateMotionCorrected(const float ranges[], const float angles[], int count, const Position& position,
        float omegaDegS, float scanDuurSec) {
    map.IntegrateScanMotionCorrected(
        position.GetX(), position.GetY(), position.GetTheta(),
        omegaDegS, scanDuurSec, angles, ranges, count
    );

    updated = true;
}


void Mapper::SetWaypoints(const Path& path) {
    // mm to meter conversion
    constexpr float MM2M = 0.001f;

    std::vector<std::pair<float,float>> waypoints;
    Path temp = path;

    temp.Reset();
    while (!temp.IsEmpty()) {
        Position p = temp.GetCurrentWaypoint();
        // append waypoint to the vector
        waypoints.emplace_back(p.GetX() * MM2M, p.GetY() * MM2M);
        // move one waypoint forward
        temp.Advance();
    }
    
    // add to waypoint list
    map.SetWaypoints(waypoints);
}

int Mapper::GetCoverage() const {
    return static_cast<int>(map.GetCoveragePercent());
}

void Mapper::GetRoomCoverage(float& outerWallPct, float& interiorPct, float& relCoveragePct) const {
    map.GetRoomCoverage(outerWallPct, interiorPct, relCoveragePct);
}

GridMap& Mapper::GetMap() { 
    return map; 
}

const GridMap& Mapper::GetMap() const { 
    return map; 
}

bool Mapper::SaveDebugMap(const std::string& filename) const {
    return map.SavePGMCropped(filename);
}