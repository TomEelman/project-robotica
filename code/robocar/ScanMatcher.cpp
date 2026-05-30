#include "ScanMatcher.h"
#include <cmath>
#include <limits>
#include <cstdio>

static constexpr float DEG2RAD = 3.14159265f / 180.0f;
static constexpr float RAD2DEG = 180.0f / 3.14159265f;

static float NormDeg(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

ScanMatcher::ScanMatcher(float max_trans_mm, float max_rot_deg)
    : maxTransMm(max_trans_mm)
    , maxRotDeg(max_rot_deg)
    , hasPrev(false)
{}

void ScanMatcher::Reset() {
    hasPrev = false;
    prevPoints.clear();
}

std::vector<ScanPoint> ScanMatcher::RangesToPoints(
    const float ranges[360], float /*theta*/)
{
    // BUG1 FIX: theta wordt NIET gebruikt voor rotatie naar wereldframe.
    //
    // Reden: ICP werkt scan-to-scan (relatief), niet scan-op-kaart.
    // Als we de vorige scan met theta_oud en de huidige met theta_nieuw
    // roteren, ziet ICP een rotatie van (theta_nieuw - theta_oud) zelfs
    // als de robot niet gedraaid heeft. De berekende dtheta is dan het
    // verschil in worldframe-heading, niet de echte rotatie tussen scans.
    //
    // Correct: beide scans in hetzelfde (robot-)frame laten staan (theta=0).
    // Dan is de dtheta die ICP vindt precies de rotatie tussen de twee scans,
    // en de encoder-seeding (initDx/initDy) compenseert de translatie.
    std::vector<ScanPoint> pts;
    pts.reserve(360);

    for (int a = 0; a < 360; ++a) {
        float r = ranges[a];
        if (r <= 50.0f || r > 8000.0f) continue;

        ScanPoint p;
        p.x = r * std::cos(static_cast<float>(a) * DEG2RAD);
        p.y = r * std::sin(static_cast<float>(a) * DEG2RAD);
        pts.push_back(p);
    }
    return pts;
}

void ScanMatcher::Centroid(const std::vector<ScanPoint>& pts,
                            float& cx, float& cy)
{
    cx = cy = 0.0f;
    if (pts.empty()) return;
    for (const auto& p : pts) { cx += p.x; cy += p.y; }
    float n = static_cast<float>(pts.size());
    cx /= n; cy /= n;
}

void ScanMatcher::FindCorrespondences(
    const std::vector<ScanPoint>& src,
    const std::vector<ScanPoint>& dst,
    std::vector<int>& indices,
    std::vector<float>& dists)
{
    indices.resize(src.size(), -1);
    dists.resize(src.size(), std::numeric_limits<float>::max());

    for (size_t i = 0; i < src.size(); ++i) {
        float bestD = MAX_DIST_MM * MAX_DIST_MM;
        int   bestJ = -1;

        for (size_t j = 0; j < dst.size(); ++j) {
            float dx = src[i].x - dst[j].x;
            float dy = src[i].y - dst[j].y;
            float d2 = dx*dx + dy*dy;
            if (d2 < bestD) { bestD = d2; bestJ = static_cast<int>(j); }
        }
        if (bestJ >= 0) {
            indices[i] = bestJ;
            dists[i]   = std::sqrt(bestD);
        }
    }
}

bool ScanMatcher::ComputeTransform(
    const std::vector<ScanPoint>& src,
    const std::vector<ScanPoint>& dst,
    const std::vector<int>& idx,
    float& outDx, float& outDy, float& outDtheta)
{
    // Verzamel geldige paren
    std::vector<ScanPoint> S, D;
    for (size_t i = 0; i < idx.size(); ++i) {
        if (idx[i] < 0) continue;
        S.push_back(src[i]);
        D.push_back(dst[static_cast<size_t>(idx[i])]);
    }

    if (static_cast<int>(S.size()) < MIN_POINTS) return false;

    // Zwaartepunten
    float csx, csy, cdx, cdy;
    Centroid(S, csx, csy);
    Centroid(D, cdx, cdy);

    // Cross-covariantie matrix H = sum((s-cs)*(d-cd)^T)
    float H00 = 0, H01 = 0, H10 = 0, H11 = 0;
    for (size_t i = 0; i < S.size(); ++i) {
        float sx = S[i].x - csx, sy = S[i].y - csy;
        float dx = D[i].x - cdx, dy = D[i].y - cdy;
        H00 += sx * dx;
        H01 += sx * dy;
        H10 += sy * dx;
        H11 += sy * dy;
    }

    // 2D rotatie uit cross-covariantie (atan2 van de antisymmetrische component)
    float angle = std::atan2(H10 - H01, H00 + H11);
    outDtheta   = NormDeg(angle * RAD2DEG);

    // Translatie: d_centroid - R * s_centroid
    float cosA = std::cos(angle), sinA = std::sin(angle);
    outDx = cdx - (cosA * csx - sinA * csy);
    outDy = cdy - (sinA * csx + cosA * csy);

    return true;
}

void ScanMatcher::TransformPoints(std::vector<ScanPoint>& pts,
                                   float dx, float dy, float dtheta)
{
    float c = std::cos(dtheta * DEG2RAD);
    float s = std::sin(dtheta * DEG2RAD);
    for (auto& p : pts) {
        float nx = c * p.x - s * p.y + dx;
        float ny = s * p.x + c * p.y + dy;
        p.x = nx; p.y = ny;
    }
}

IcpResult ScanMatcher::Match(const float ranges[360], float robotTheta,
                              float initDx, float initDy)
{
    IcpResult result{0.0f, 0.0f, 0.0f, 9999.0f, false};

    auto curPoints = RangesToPoints(ranges, robotTheta);

    if (static_cast<int>(curPoints.size()) < MIN_POINTS) {
        // Te weinig punten — sla op als nieuwe vorige scan
        prevPoints = curPoints;
        hasPrev    = true;
        return result;
    }

    if (!hasPrev || static_cast<int>(prevPoints.size()) < MIN_POINTS) {
        prevPoints = curPoints;
        hasPrev    = true;
        return result;
    }

    // ICP iteraties
    std::vector<ScanPoint> moving = curPoints;

    // Pas encoder-schatting toe als initiële verplaatsing zodat ICP
    // al dicht bij de oplossing begint.
    if (initDx != 0.0f || initDy != 0.0f) {
        for (auto& p : moving) { p.x += initDx; p.y += initDy; }
    }

    // BUG2 FIX: houd encoder-seed en ICP-correctie gescheiden.
    // totalDx/Dy/Dtheta = alleen de ICP-verfijning bovenop de encoder-seed.
    // De encoder-seed is de verwachte beweging en telt NIET mee in de
    // acceptatiecheck — anders wordt rijden zelf al afgewezen (encoder ~20mm
    // + kleine ICP-stap → totaal >maxTransMm → onterecht afgewezen).
    float totalDx = 0.0f, totalDy = 0.0f, totalDtheta = 0.0f;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        std::vector<int>   idx;
        std::vector<float> dists;
        FindCorrespondences(moving, prevPoints, idx, dists);

        float stepDx, stepDy, stepDtheta;
        if (!ComputeTransform(moving, prevPoints, idx, stepDx, stepDy, stepDtheta))
            break;

        TransformPoints(moving, stepDx, stepDy, stepDtheta);

        totalDx     += stepDx;
        totalDy     += stepDy;
        totalDtheta  = NormDeg(totalDtheta + stepDtheta);

        // Convergentiecheck
        float mag = std::sqrt(stepDx*stepDx + stepDy*stepDy);
        if (mag < CONV_THRESH && std::fabs(stepDtheta) < 0.1f) break;
    }

    // Bereken fitness (gemiddelde afstand na matching)
    std::vector<int>   fidx;
    std::vector<float> fdists;
    FindCorrespondences(moving, prevPoints, fidx, fdists);
    float sumD = 0; int cnt = 0;
    for (float d : fdists) { if (d < MAX_DIST_MM) { sumD += d; ++cnt; } }
    float fitness = cnt > 0 ? sumD / static_cast<float>(cnt) : 9999.0f;

    // Accepteer alleen als de ICP-CORRECTIE (niet encoder+correctie) binnen de limieten valt.
    // transMag = grootte van de verfijning die ICP zelf heeft toegevoegd bovenop de encoder-seed.
    float transMag = std::sqrt(totalDx*totalDx + totalDy*totalDy);
    if (transMag             > maxTransMm ||
        std::fabs(totalDtheta) > maxRotDeg  ||
        fitness              > 80.0f)
    {
        printf("[ICP] afgewezen: icp_corr=%.1fmm rot=%.1fdeg fit=%.1f\n",
               transMag, totalDtheta, fitness);
        prevPoints = curPoints;
        return result;
    }

    // Geef encoder-seed + ICP-verfijning terug als totale verplaatsing.
    // ApplyIcpCorrection gebruikt dit als anchor + delta.
    result.dx      = initDx + totalDx;
    result.dy      = initDy + totalDy;
    result.dtheta  = totalDtheta;
    result.fitness = fitness;
    result.valid   = true;

    printf("[ICP] OK: dx=%.1f dy=%.1f dth=%.2f fit=%.1f (enc=%.1f,%.1f icp_corr=%.1fmm)\n",
           result.dx, result.dy, result.dtheta, fitness,
           initDx, initDy, transMag);

    // Bewaar getransformeerde scan als nieuwe referentie
    prevPoints = moving;
    return result;
}