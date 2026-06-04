#pragma once
#include <vector>
#include <cmath>

struct ScanPoint {
    float x, y;  // in mm
};

struct IcpResult {
    float dx;       // mm
    float dy;       // mm
    float dtheta;   // degrees
    float fitness;  // average distance after match (lower = better)
    bool  valid;    // false if matching fails
};

class ScanMatcher {
public:

    // max_trans_mm = max ICP correction on top of the encoder seed
    // max_rot_mm = max rotation correction while riding with a angle
    explicit ScanMatcher(float max_trans_mm = 40.0f, float max_rot_deg  = 12.0f);

    // ICP Match function
    IcpResult Match(const float ranges[360], float initDx = 0.0f, float initDy = 0.0f);

    // Resets last scan if inaccurate
    void Reset();

    bool HasPrevScan() const { return hasPrev; }

private:
    // for converting DEGrees to RADials and back
    static constexpr float DEG2RAD = 3.14159265f / 180.0f;
    static constexpr float RAD2DEG = 180.0f / 3.14159265f;

    static constexpr int    MAX_ITERATION = 20;
    static constexpr float  MAX_DIST_MM   = 80.0f; // max distance for a point in findcorrespondences
    static constexpr int    MIN_POINTS    = 30;    // min bruikbare punten

    float maxTransMm;
    float maxRotDeg;
    bool  hasPrev;

    // dynamic size array for scanpoints
    std::vector<ScanPoint> prevPoints;

    // convert LiDAR-ranges to 2D point in worldframe
    static std::vector<ScanPoint> RangesToPoints(const float ranges[360]);

    // calculate center of gravity
    static void Centroid(const std::vector<ScanPoint>& pts, float& cx, float& cy);

    // find closest neighboring points
    static void FindCorrespondences(
        const std::vector<ScanPoint>& src,
        const std::vector<ScanPoint>& dst,
        std::vector<int>& indices,
        std::vector<float>& dists);

    // 2D rotation and translation via cross covariantie 
    static bool ComputeTransform(
        const std::vector<ScanPoint>& src,
        const std::vector<ScanPoint>& dst,
        const std::vector<int>& idx,
        float& outDx, float& outDy, float& outDtheta);

    static void TransformPoints(std::vector<ScanPoint>& pts, float dx, float dy, float dtheta);
};