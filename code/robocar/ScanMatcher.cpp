#include "ScanMatcher.h"
#include <cmath>
#include <limits>
#include <cstdio>

// normalizes angles (in degrees) to always fall into +180 or -180
static float NormDeg(float d) {
    while (d >  180.0f) {
        d -= 360.0f;
    } 

    while (d < -180.0f) {
         d += 360.0f;
    }

    return d;
}

// initializes ScanMatcher
ScanMatcher::ScanMatcher(float max_trans_mm, float max_rot_deg)
    : maxTransMm(max_trans_mm)
    , maxRotDeg(max_rot_deg)
    , hasPrev(false)
{}

// Resets last scan if inaccurate
void ScanMatcher::Reset() {
    hasPrev = false;
    prevPoints.clear();
}

std::vector<ScanPoint> ScanMatcher::RangesToPoints( const float ranges[360]) {
    std::vector<ScanPoint> points;
    points.reserve(360);

    for (int a = 0; a < 360; ++a) {
        float r = ranges[a];

        // ignore if ranges is outside of lidar resrictions (min and max)
        if (r <= 50.0f || r > 8000.0f) continue;

        ScanPoint p;

        // converts from Polar coordinate (r, a) to Cartesian coordinate (x, y)
        // https://brilliant.org/wiki/convert-polar-coordinates-to-cartesian/
        p.x = r * std::cos(static_cast<float>(a) * DEG2RAD);
        p.y = r * std::sin(static_cast<float>(a) * DEG2RAD);

        // appends p to the end of the vector
        points.push_back(p);
    }

    return points;
}

void ScanMatcher::Centroid(const std::vector<ScanPoint>& points, float& cx, float& cy) {
    cx = 0.0f;
    cy = 0.0f;

    // ignore if points are empty
    if (points.empty()) return;

    // counts up all the x and y values for all points
    for (size_t i = 0; i < points.size(); ++i) 
    { 
        cx += points[i].x;
        cy += points[i].y;    
    }
    
    float n = static_cast<float>(points.size());

    // takes average to get center of gravity
    cx /= n; 
    cy /= n;
}

void ScanMatcher::FindCorrespondences( const std::vector<ScanPoint>& source, const std::vector<ScanPoint>& destination,
    std::vector<int>& indices, std::vector<float>& distances) {
    // resizes indices vector to source.size() amount and fills all slots with -1
    indices.resize(source.size(), -1);
    // resizes distances vector to source.size() amount and fills all slots with the max amount the float can be
    distances.resize(source.size(), std::numeric_limits<float>::max());

    for (size_t i = 0; i < source.size(); ++i) {
        // starts as the max allowed distance threshold, but will shrink to hold the closest distance found so far
        float bestD = MAX_DIST_MM * MAX_DIST_MM;
        // placeholder to store the index of the closest destination point if found
        int   bestJ = -1;

        for (size_t j = 0; j < destination.size(); ++j) {
            float dx = source[i].x - destination[j].x;
            float dy = source[i].y - destination[j].y;

            // Calculates squared distance (Pythagorean theorem)
            float d2 = dx * dx + dy * dy;

            // if this point is closer than the closest one found so far (and within the max threshold)
            if (d2 < bestD) { 
                // save this as the new current best candidate 
                bestD = d2; 
                bestJ = static_cast<int>(j); 
            }
        }

        // if a valid match was found (bestJ is no longer -1)
        if (bestJ >= 0) {
            // save the winning match index and its actual distance
            indices[i] = bestJ;
            distances[i]   = std::sqrt(bestD);
        }
    }
}

bool ScanMatcher::ComputeTransform( const std::vector<ScanPoint>& source, const std::vector<ScanPoint>& destination,
    const std::vector<int>& index, float& outDx, float& outDy, float& outDtheta) {
    // vectors to hold only the points that successfully found a match
    std::vector<ScanPoint> S, D;
    
    // loops through all indices to filter out valid pairs
    for (size_t i = 0; i < index.size(); ++i) {
        // if index is -1 (no match found) skip this point
        if (index[i] < 0) continue;
        
        // save the matching source and destination points into our new vectors
        S.push_back(source[i]);
        D.push_back(destination[static_cast<size_t>(index[i])]);
    }

    // if we don't have enough matched points to calculate alignment give up and return false
    if (static_cast<int>(S.size()) < MIN_POINTS) return false;

    // placeholders for the centers of gravity for both point sets
    float csx, csy, cdx, cdy;
    // calculates the average centers for our filtered points
    Centroid(S, csx, csy);
    Centroid(D, cdx, cdy);

    // variables for the covariance matrix (used to find the best rotation)
    float H00 = 0, H01 = 0, H10 = 0, H11 = 0;
    
    // cross-multiplies(matrix) points relative to their centers to see how the shapes correlate
    for (size_t i = 0; i < S.size(); ++i) {
        float sx = S[i].x - csx;
        float sy = S[i].y - csy;

        float dx = D[i].x - cdx;
        float dy = D[i].y - cdy;
        
        H00 += sx * dx;
        H01 += sx * dy;
        H10 += sy * dx;
        H11 += sy * dy;
    }

    // uses trigonometry on the matrix to calculate the rotation angle in radians
    float angle = std::atan2(H10 - H01, H00 + H11);
    // converts radians to degrees and normalizes it between -180 and 180
    outDtheta   = NormDeg(angle * RAD2DEG);

    // calculates trig values for the final translation math
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);
    
    // finds the X and Y movement (translation offset) between the two scans
    outDx = cdx - (cosA * csx - sinA * csy);
    outDy = cdy - (sinA * csx + cosA * csy);

    // returns true because the calculation succeeded
    return true;
}

void ScanMatcher::TransformPoints(std::vector<ScanPoint>& points,
    float dx, float dy, float dtheta) {
    // converts the rotation angle from degrees to radians and gets the trig values
    float c = std::cos(dtheta * DEG2RAD);
    float s = std::sin(dtheta * DEG2RAD);

    // loops through every point in the vector to move them one by one
    for (size_t i = 0; i < points.size(); ++i) {
        // applies 2D rotation matrix math and adds the X and Y translation offsets
        float nx = c * points[i].x - s * points[i].y + dx;
        float ny = s * points[i].x + c * points[i].y + dy;
        
        // updates the point's actual coordinates with the new calculated position
        points[i].x = nx; 
        points[i].y = ny;
    }
}

IcpResult ScanMatcher::Match(const float ranges[360], float initDx, float initDy) {
    // default blank result setup to return if tracking fails
    IcpResult result{0.0f, 0.0f, 0.0f, 9999.0f, false};
    
    // converts LiDAR ranges to (x, y) points
    auto curPoints = RangesToPoints(ranges);

    // safety check: if current scan is too empty, save it for next time and give up
    if (static_cast<int>(curPoints.size()) < MIN_POINTS) {
        prevPoints = curPoints;
        hasPrev    = true;
        return result;
    }

    // safety check: if we don't have a valid previous scan to compare against, save this one and give up
    if (!hasPrev || static_cast<int>(prevPoints.size()) < MIN_POINTS) {
        prevPoints = curPoints;
        hasPrev    = true;
        return result;
    }

    std::vector<ScanPoint> moving = curPoints;

    // if an initial guess from wheel encoders was given, apply it to jump-start the alignment
    if (initDx != 0.0f || initDy != 0.0f) {
        for (size_t i = 0; i < moving.size(); ++i) { 
            moving[i].x += initDx; 
            moving[i].y += initDy; 
        }
    }

    // running totals to track total movement over all alignment steps
    float totalDx = 0.0f; 
    float totalDy = 0.0f;
    float totalDtheta = 0.0f;

    // repeatedly nudges the points closer until they snap onto the map
    for (int iteration = 0; iteration < MAX_ITERATION; ++iteration) {
        std::vector<int>   index;
        std::vector<float> distance;

        // Step 1: Link up closest point pairs between the scans
        FindCorrespondences(moving, prevPoints, index, distance);

        float stepDx; 
        float stepDy;
        float stepDtheta;

        // Step 2: Calculate the best shift/rotation recipe for this step (break if math fails)
        if (!ComputeTransform(moving, prevPoints, index, stepDx, stepDy, stepDtheta)) break;

        // Step 3: Actually move the scan points using that recipe
        TransformPoints(moving, stepDx, stepDy, stepDtheta);

        // add this (recipe)step movement to our running totals
        totalDx     += stepDx;
        totalDy     += stepDy;
        totalDtheta  = NormDeg(totalDtheta + stepDtheta);

        // Pythagorean theorem
        float mag = std::sqrt(stepDx*stepDx + stepDy*stepDy);
        // if change is small stop early
        if (mag < 0.5f && std::fabs(stepDtheta) < 0.1f) break;
    }

    // link points one last time to see how good the final overlay is
    std::vector<int>   findex;
    std::vector<float> fdistance;
    FindCorrespondences(moving, prevPoints, findex, fdistance);

    float sumD = 0; 
    int cnt = 0;

    // calculate fitness (the average distance error of close matches)
    for (size_t i = 0; i < fdistance.size(); ++i) { 
        if (fdistance[i] < MAX_DIST_MM) { 
            sumD += fdistance[i]; 
            ++cnt; 
        } 
    }

    float fitness;

    // if we found at least one valid close match
    if (cnt > 0) {
        fitness = sumD / static_cast<float>(cnt);
    } else {
        fitness = 9999.0f;
    }

    // if the robot magically jumped too far or the fit error is terrible, reject it
    float transMag = std::sqrt(totalDx*totalDx + totalDy*totalDy);

    if (transMag > maxTransMm || std::fabs(totalDtheta) > maxRotDeg  || fitness > 80.0f)
    {
        // printf("[ICP] afgewezen: icp_corr=%.1fmm rot=%.1fdeg fit=%.1f\n", transMag, totalDtheta, fitness);
        prevPoints = curPoints;
        return result;
    }

    // SUCCESS: fill out the results combining the initial guess and the fine-tuned ICP totals
    result.dx      = initDx + totalDx;
    result.dy      = initDy + totalDy;
    result.dtheta  = totalDtheta;
    result.fitness = fitness;
    result.valid   = true;

    //printf("[ICP] OK: dx=%.1f dy=%.1f dth=%.2f fit=%.1f (enc=%.1f,%.1f icp_corr=%.1fmm)\n", result.dx, result.dy, result.dtheta, fitness, nitDx, initDy, transMag);

    // save this settled moving frame as our future anchor map for the next incoming scan
    prevPoints = moving;
    return result;
}