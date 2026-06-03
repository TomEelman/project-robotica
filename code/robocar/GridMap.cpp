#include "GridMap.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <stdexcept>

GridMap::GridMap(int w, int h, float res)
    : width(w)
    , height(h)
    , resolution(res)
    , originX(-static_cast<float>(w) * res * 0.5f)   // midden van het grid = (0,0) in de wereld
    , originY(-static_cast<float>(h) * res * 0.5f)
    , logOdds(h, std::vector<int8_t>(w, CELL_UNKNOWN))
    , binaryGrid(h, std::vector<int>(w, 0))
    , binaryDirty(false)
{
}

void GridMap::ClampLogOdds(int8_t& val) const {
    if (val < L_MIN) val = L_MIN;
    if (val > L_MAX) val = L_MAX;
}

bool GridMap::InBounds(int cx, int cy) const {
    return cx >= 0 && cx < width && cy >= 0 && cy < height;
}

void GridMap::WorldToCell(float wx, float wy, int& cx, int& cy) const {
    cx = static_cast<int>((wx - originX) / resolution);
    cy = static_cast<int>((wy - originY) / resolution);
}

void GridMap::CellToWorld(int cx, int cy, float& wx, float& wy) const {
    wx = originX + (static_cast<float>(cx) + 0.5f) * resolution;
    wy = originY + (static_cast<float>(cy) + 0.5f) * resolution;
}

bool GridMap::UpdateCell(int cx, int cy, bool occupied) {
    if (!InBounds(cx, cy)) return false;

    int8_t& cell = logOdds[cy][cx];
    cell = static_cast<int8_t>(cell + (occupied ? L_OCC : L_FREE));
    ClampLogOdds(cell);
    binaryDirty = true;
    return true;
}

void GridMap::BresenhamLine(int x0, int y0, int x1, int y1,
                             std::vector<std::pair<int,int>>& cells) const
{
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

void GridMap::RaycastUpdate(int x0, int y0, int x1, int y1) {
    static thread_local std::vector<std::pair<int,int>> cells;
    BresenhamLine(x0, y0, x1, y1, cells);

    // Alle cellen behalve de laatste: vrij markeren
    for (size_t i = 0; i + 1 < cells.size(); ++i) {
        auto [cx, cy] = cells[i];
        if (!InBounds(cx, cy)) continue;
        int8_t& cell = logOdds[cy][cx];
        cell = static_cast<int8_t>(cell + L_FREE);
        ClampLogOdds(cell);
    }

    // Laatste cel: bezet markeren
    if (!cells.empty()) {
        auto [cx, cy] = cells.back();
        if (InBounds(cx, cy)) {
            int8_t& cell = logOdds[cy][cx];
            cell = static_cast<int8_t>(cell + L_OCC);
            ClampLogOdds(cell);
        }
    }

    binaryDirty = true;
}

void GridMap::IntegrateScan(float robotX, float robotY, float robotTheta,
                             const float angles[], const float ranges[], int count,
                             float maxRange)
{
    // ── Eenheden ──────────────────────────────────────────────────
    // robotX, robotY  : mm  (van Localisation)
    // ranges[]        : mm  (van LIDAR)
    // resolution      : meter/cel
    // originX/Y       : meter
    // → alles omzetten naar METER voor WorldToCell en eindpunt berekening
    constexpr float MM2M    = 0.001f;
    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    float robotX_m = robotX * MM2M;
    float robotY_m = robotY * MM2M;

    int rx, ry;
    WorldToCell(robotX_m, robotY_m, rx, ry);
    robotPath.emplace_back(robotX_m, robotY_m);

    for (int i = 0; i < count; ++i) {
        float dist_mm = ranges[i];

        if (dist_mm <= 0.0f || dist_mm > maxRange) continue;

        float dist_m = dist_mm * MM2M;

        // Wereldhoek = robotrichting (graden → rad) + LIDARhoek (graden → rad)
        float globalAngle = robotTheta * DEG2RAD + angles[i] * DEG2RAD;

        // Eindpunt in meter
        float wx = robotX_m + dist_m * std::cos(globalAngle);
        float wy = robotY_m + dist_m * std::sin(globalAngle);

        int ex, ey;
        WorldToCell(wx, wy, ex, ey);

        RaycastUpdate(rx, ry, ex, ey);
    }
}

// ═══════════════════════════════════════════════════════════════════
//  IntegrateScanMotionCorrected
//
//  Een LIDAR-scan duurt scanDuurSec seconden (typisch 0.10-0.20s).
//  In die tijd draait de robot met omegaDegS graden per seconde.
//  Punt bij index 0 werd gemeten aan het BEGIN van de scan,
//  punt bij index (count-1) aan het EINDE.
//
//  Per straal berekenen we de robothoek op het moment van meten:
//    t_i = (i / count) * scanDuurSec          [seconden na scanstart]
//    theta_i = robotThetaDeg + omegaDegS * (t_i - scanDuurSec/2)
//
//  We gebruiken de positie van het MIDDEN van de scan als anker
//  (dat is al ingebakken in robotX/Y/robotThetaDeg die je meegeeft).
//  De positietranslatie tijdens de scan negeren we (bij 278mm/s en
//  0.15s scanduur = 42mm = < 1 cel — verwaarloosbaar).
// ═══════════════════════════════════════════════════════════════════

void GridMap::IntegrateScanMotionCorrected(
    float robotX, float robotY, float robotThetaDeg,
    float omegaDegS, float scanDuurSec,
    const float angles[], const float ranges[], int count,
    float maxRange)
{
    constexpr float MM2M    = 0.001f;
    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    float robotX_m = robotX * MM2M;
    float robotY_m = robotY * MM2M;

    int rx, ry;
    WorldToCell(robotX_m, robotY_m, rx, ry);
    robotPath.emplace_back(robotX_m, robotY_m);

    float halfDuur = scanDuurSec * 0.5f;

    for (int i = 0; i < count; ++i) {
        float dist_mm = ranges[i];
        if (dist_mm <= 0.0f || dist_mm > maxRange) continue;

        // Tijdstip van deze straal binnen de scan (0 = begin, scanDuurSec = einde)
        float t_i = (static_cast<float>(i) / static_cast<float>(count)) * scanDuurSec;

        // Gecorrigeerde robothoek op dit moment
        // (t_i - halfDuur): offset t.o.v. het midden van de scan
       // float thetaCorrected = robotThetaDeg + omegaDegS * (t_i - halfDuur);
        float thetaCorrected = robotThetaDeg - omegaDegS * (t_i - halfDuur);
        // Wereldhoek van deze straal
        float globalAngle = thetaCorrected * DEG2RAD + angles[i] * DEG2RAD;

        float dist_m = dist_mm * MM2M;
        float wx = robotX_m + dist_m * std::cos(globalAngle);
        float wy = robotY_m + dist_m * std::sin(globalAngle);

        int ex, ey;
        WorldToCell(wx, wy, ex, ey);

        RaycastUpdate(rx, ry, ex, ey);
    }
}

bool GridMap::IsOccupied(int cx, int cy) const {
    if (!InBounds(cx, cy)) return false;
    return logOdds[cy][cx] >= CELL_OCCUPIED;
}

bool GridMap::IsFree(int cx, int cy) const {
    if (!InBounds(cx, cy)) return false;
    return logOdds[cy][cx] <= CELL_FREE;
}

bool GridMap::IsUnknown(int cx, int cy) const {
    if (!InBounds(cx, cy)) return true;
    int8_t v = logOdds[cy][cx];
    return (v > CELL_FREE && v < CELL_OCCUPIED);
}

bool GridMap::IsPathValid(Path path) const {
    constexpr float MM2M = 0.001f;
    while (!path.IsEmpty()) {
        Position p = path.GetCurrentWaypoint();
        int cx, cy;
        WorldToCell(p.GetX() * MM2M, p.GetY() * MM2M, cx, cy);

        if (!InBounds(cx, cy))   return false;
        if (IsOccupied(cx, cy))  return false;

        path.Advance();
    }
    return true;
}

float GridMap::GetCoveragePercent() const {
    int known = 0;
    int total = width * height;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (!IsUnknown(x, y)) ++known;
    return total > 0 ? 100.0f * static_cast<float>(known) / static_cast<float>(total) : 0.0f;
}

// ─────────────────────────────────────────────────────────────────
//  GetRoomCoverage
//
//  Geeft drie waarden terug die samen beschrijven of de ruimte
//  volledig in kaart is gebracht:
//
//  outerWallPct   — % van de buitenste rand (BAND_WIDTH cellen dik)
//                   van het gescande gebied dat als bezet (muur)
//                   is geclassificeerd. Hoog = muren rondom gevonden.
//
//  interiorPct    — % van de cellen BINNEN die buitenste rand dat
//                   bekend is (vrij of bezet). Hoog = interieur gemapt.
//
//  relCoveragePct — % van de gehele bounding box dat bekend is.
//                   Correctere vervanging voor GetCoveragePercent()
//                   (die deelt door het totale grid en geeft zo een
//                   veel te laag getal voor kleine kamers).
//
//  BAND_WIDTH = 5 cellen ≈ 15 cm bij 3 cm/cel — dik genoeg om
//  muren te vangen, klein genoeg om het interieur niet te claimen.
// ─────────────────────────────────────────────────────────────────
void GridMap::GetRoomCoverage(float& outerWallPct,
                               float& interiorPct,
                               float& relCoveragePct) const
{
    constexpr int BAND = 5;   // dikte buitenste rand in cellen

    // ── Bounding box van alle bekende cellen ─────────────────────
    int minX = width,  maxX = -1;
    int minY = height, maxY = -1;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (!IsUnknown(x, y)) {
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }

    if (maxX < 0) {
        // Nog niets gescand
        outerWallPct = interiorPct = relCoveragePct = 0.0f;
        return;
    }

    int bboxW = maxX - minX + 1;
    int bboxH = maxY - minY + 1;

    // ── Relatieve coverage over bounding box ─────────────────────
    int bboxTotal = bboxW * bboxH;
    int bboxKnown = 0;
    for (int y = minY; y <= maxY; ++y)
        for (int x = minX; x <= maxX; ++x)
            if (!IsUnknown(x, y)) ++bboxKnown;
    relCoveragePct = 100.0f * static_cast<float>(bboxKnown)
                             / static_cast<float>(bboxTotal);

    // ── Buitenste rand: ring van BAND cellen aan de binnenkant ───
    // Een cel behoort tot de buitenste rand als hij binnen BAND
    // cellen van een bbox-rand valt.
    int outerTotal = 0, outerOccupied = 0;
    int innerTotal = 0, innerKnown   = 0;

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            bool inBand = (x < minX + BAND || x > maxX - BAND ||
                           y < minY + BAND || y > maxY - BAND);

            if (inBand) {
                ++outerTotal;
                if (IsOccupied(x, y)) ++outerOccupied;
            } else {
                ++innerTotal;
                if (!IsUnknown(x, y)) ++innerKnown;
            }
        }
    }

    outerWallPct = (outerTotal > 0)
        ? 100.0f * static_cast<float>(outerOccupied)
                 / static_cast<float>(outerTotal)
        : 0.0f;

    interiorPct = (innerTotal > 0)
        ? 100.0f * static_cast<float>(innerKnown)
                 / static_cast<float>(innerTotal)
        : 100.0f;   // te klein voor interieur → beschouw als volledig
}

const std::vector<std::vector<int8_t>>& GridMap::GetLogOddsGrid() const {
    return logOdds;
}

void GridMap::RebuildBinaryGrid() const {
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            binaryGrid[y][x] = (logOdds[y][x] >= CELL_OCCUPIED) ? 1 : 0;
    binaryDirty = false;
}

const std::vector<std::vector<int>>& GridMap::GetGrid() const {
    if (binaryDirty) RebuildBinaryGrid();
    return binaryGrid;
}

void GridMap::SetWaypoints(const std::vector<std::pair<float,float>>& wps_m) {
    waypointList = wps_m;
}

void GridMap::Clear() {
    for (auto& row : logOdds)
        std::fill(row.begin(), row.end(), CELL_UNKNOWN);
    robotPath.clear();
    waypointList.clear();
    binaryDirty = true;
}

//  SavePGM  –  volledige kaart, wit=vrij, zwart=bezet, grijs=onbekend

bool GridMap::SavePGM(const std::string& filename) const {
    std::ofstream f(filename, std::ios::binary);
    if (!f) return false;

    f << "P5\n" << width << " " << height << "\n255\n";

    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            int8_t v = logOdds[y][x];
            uint8_t px;
            if      (v >= CELL_OCCUPIED) px = 0;
            else if (v <= CELL_FREE)     px = 255;
            else                          px = 127;
            f.put(static_cast<char>(px));
        }
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  SavePGMCropped  –  bijgesneden kaart met afmetingen
//
//  Werking:
//    1. Zoek de bounding box van alle bekende cellen (vrij + bezet)
//    2. Voeg een marge toe rondom dit gebied
//    3. Schrijf een PPM (kleur) zodat pad/waypoints gekleurd kunnen zijn
//    4. Toon onder de kaart de afmetingen van de kamer:
//       "<breedte> x <hoogte>" in meter als tekst
// ═══════════════════════════════════════════════════════════════════

// ── 3×5 pixel-font voor cijfers 0-9 (bit 2 = links) ──────────────
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

static void DrawDigit(std::vector<uint8_t>& img, int imgW, int imgH,
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

// ═══════════════════════════════════════════════════════════════════
//  SavePGMCropped  –  bijgesneden kaart met afmetingen
//  Snijdt automatisch bij op het verkende gebied + marge.
//  Elke cel wordt SCALE×SCALE pixels (scherper beeld).
//  Onder de kaart: de afmetingen van de kamer ("breedte x hoogte" in m).
//  Schrijft als PPM (kleur) zodat pad/waypoints gekleurd kunnen zijn.
// ═══════════════════════════════════════════════════════════════════

bool GridMap::SavePGMCropped(const std::string& filename, float margin_m) const {
    // ── Stap 1: bounding box + muur-cellen verzamelen ─────────────
    //  minX..maxY  = alle bekende cellen (voor bijsnijden v/d afbeelding)
    //  wallPts     = alle muur-cellen (bezet) → echte kamer-afmeting via PCA
    int minX = width,  maxX = -1;
    int minY = height, maxY = -1;
    std::vector<std::pair<int,int>> wallPts;

    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            if (IsUnknown(x, y)) continue;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            if (IsOccupied(x, y)) wallPts.emplace_back(x, y);   // muur
        }

    if (maxX < 0) return SavePGM(filename);   // niets gescand

    // ── Stap 2: marge toevoegen ───────────────────────────────────
    int mg = static_cast<int>(margin_m / resolution) + 1;
    int x0 = std::max(0,      minX - mg);
    int y0 = std::max(0,      minY - mg);
    int x1 = std::min(width,  maxX + mg + 1);
    int y1 = std::min(height, maxY + mg + 1);
    int cW = x1 - x0;
    int cH = y1 - y0;

    // ── Stap 3: upscaling – elke cel = SCALE×SCALE pixels ────────
    constexpr int SCALE = 3;
    int imgW = cW * SCALE;
    int imgH = cH * SCALE;

    // ── Echte kamer-afmetingen via kleinste omhullende rechthoek ──
    //  De kamer staat schuin én is niet rechthoekig (inham/schiereiland).
    //  PCA faalt dan: bij een symmetrische puntenwolk kiest het de
    //  diagonaal. Daarom: convex hull (pakt alleen de buitenmuren – de
    //  concave inham valt er vanzelf binnen) + de minimum-area rechthoek,
    //  die 'vastklikt' op de echte muur-richting. De zijden van die
    //  rechthoek zijn de werkelijke breedte en hoogte.
    using Pt = std::pair<int,int>;

    // Losse ruis-cellen (geen enkele bezette buur) weggooien, zodat de
    // convex hull niet door één uitschieter wordt opgeblazen.
    {
        std::vector<Pt> kept;
        kept.reserve(wallPts.size());
        for (const auto& p : wallPts) {
            bool hasNb = false;
            for (int dy = -1; dy <= 1 && !hasNb; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if ((dx || dy) && IsOccupied(p.first + dx, p.second + dy)) { hasNb = true; break; }
                }
            if (hasNb) kept.push_back(p);
        }
        if (!kept.empty()) wallPts.swap(kept);
    }

    if (wallPts.empty())                          // geen muren → val terug op bbox
        wallPts = { {minX, minY}, {maxX, minY}, {minX, maxY}, {maxX, maxY} };

    // Convex hull (monotone chain).
    std::sort(wallPts.begin(), wallPts.end());
    wallPts.erase(std::unique(wallPts.begin(), wallPts.end()), wallPts.end());
    auto cross = [](const Pt& o, const Pt& a, const Pt& b) {
        return static_cast<double>(a.first - o.first) * (b.second - o.second)
             - static_cast<double>(a.second - o.second) * (b.first - o.first);
    };
    int np = static_cast<int>(wallPts.size());
    std::vector<Pt> hull(2 * np);
    int k = 0;
    for (int i = 0; i < np; ++i) {                // onderrand
        while (k >= 2 && cross(hull[k-2], hull[k-1], wallPts[i]) <= 0) --k;
        hull[k++] = wallPts[i];
    }
    for (int i = np - 2, t = k + 1; i >= 0; --i) { // bovenrand
        while (k >= t && cross(hull[k-2], hull[k-1], wallPts[i]) <= 0) --k;
        hull[k++] = wallPts[i];
    }
    hull.resize(std::max(1, k - 1));

    // Minimum-area rechthoek: probeer elke hull-zijde als oriëntatie.
    double bestArea = 1e30, ux = 1, uy = 0;
    double bMinU = 0, bMaxU = 0, bMinV = 0, bMaxV = 0;
    int H = static_cast<int>(hull.size());
    for (int i = 0; i < H; ++i) {
        double ex = hull[(i+1) % H].first  - hull[i].first;
        double ey = hull[(i+1) % H].second - hull[i].second;
        double len = std::hypot(ex, ey);
        if (len < 1e-9) continue;
        double dux = ex / len, duy = ey / len;     // langs deze zijde
        double dvx = -duy,     dvy = dux;          // loodrecht erop
        double mnU = 1e30, mxU = -1e30, mnV = 1e30, mxV = -1e30;
        for (const auto& p : hull) {
            double du = p.first * dux + p.second * duy;
            double dv = p.first * dvx + p.second * dvy;
            mnU = std::min(mnU, du);  mxU = std::max(mxU, du);
            mnV = std::min(mnV, dv);  mxV = std::max(mxV, dv);
        }
        double area = (mxU - mnU) * (mxV - mnV);
        if (area < bestArea) {
            bestArea = area;  ux = dux;  uy = duy;
            bMinU = mnU;  bMaxU = mxU;  bMinV = mnV;  bMaxV = mxV;
        }
    }
    double vx = -uy, vy = ux;                      // tweede as van de rechthoek

    // Welke as is 'horizontaal' (breedte) en welke 'verticaal' (hoogte)?
    bool uHoriz = std::abs(ux) >= std::abs(uy);
    double ahx = uHoriz ? ux : vx, ahy = uHoriz ? uy : vy;   // horizontale as
    double avx = uHoriz ? vx : ux, avy = uHoriz ? vy : uy;   // verticale as
    double minH = uHoriz ? bMinU : bMinV, maxH = uHoriz ? bMaxU : bMaxV;
    double minV = uHoriz ? bMinV : bMinU, maxV = uHoriz ? bMaxV : bMaxU;

    float realW_m = static_cast<float>((maxH - minH) * resolution);   // breedte
    float realH_m = static_cast<float>((maxV - minV) * resolution);   // hoogte

    // Vier hoeken van de min-area rechthoek (celcoördinaten) – voor de maatlijnen.
    //   cel = h·(horizontale as) + v·(verticale as)
    auto toCellX = [&](double h, double v) { return h * ahx + v * avx; };
    auto toCellY = [&](double h, double v) { return h * ahy + v * avy; };
    double rectCellX[4] = { toCellX(minH, minV), toCellX(maxH, minV),
                            toCellX(maxH, maxV), toCellX(minH, maxV) };
    double rectCellY[4] = { toCellY(minH, minV), toCellY(maxH, minV),
                            toCellY(maxH, maxV), toCellY(minH, maxV) };

    // ── Stap 4: geen onderbalk meer; maten staan op de kaart ──────
    int totalH = imgH;

    // ── Stap 5: pixel-buffer vullen ───────────────────────────────
    std::vector<uint8_t> img(static_cast<size_t>(imgW * totalH * 3), 235);

    // Kaartpixels – elke cel → SCALE×SCALE pixels
    for (int y = y1 - 1; y >= y0; --y) {
        int cellRow = (y1 - 1 - y);
        for (int x = x0; x < x1; ++x) {
            int cellCol = x - x0;
            uint8_t px;
            if      (!InBounds(x, y))                px = 200;
            else if (logOdds[y][x] >= CELL_OCCUPIED) px = 0;
            else if (logOdds[y][x] <= CELL_FREE)     px = 255;
            else                                      px = 160;

            for (int sy = 0; sy < SCALE; ++sy)
                for (int sx = 0; sx < SCALE; ++sx) {
                    int idx = ((cellRow * SCALE + sy) * imgW + cellCol * SCALE + sx) * 3;
                    img[static_cast<size_t>(idx)]     = px;
                    img[static_cast<size_t>(idx + 1)] = px;
                    img[static_cast<size_t>(idx + 2)] = px;
                }
        }
    }

    // ── Stap 6: robotpad in rood tekenen ─────────────────────────
    for (auto& [wx, wy] : robotPath) {
        int cx, cy;
        WorldToCell(wx, wy, cx, cy);
        if (cx < x0 || cx >= x1 || cy < y0 || cy >= y1) continue;
        int col = (cx - x0) * SCALE;
        int row = (y1 - 1 - cy) * SCALE;
        for (int sy = 0; sy < SCALE; ++sy)
            for (int sx = 0; sx < SCALE; ++sx) {
                size_t idx = static_cast<size_t>((row + sy) * imgW + col + sx) * 3;
                if (idx + 2 >= img.size()) continue;
                img[idx]     = 220;
                img[idx + 1] = 50;
                img[idx + 2] = 50;
            }
    }

    // ── Stap 7: waypoints genummerd in groen tekenen ─────────────
    for (size_t i = 0; i < waypointList.size(); ++i) {
        auto [wx, wy] = waypointList[i];
        int cx, cy;
        WorldToCell(wx, wy, cx, cy);
        if (cx < x0 || cx >= x1 || cy < y0 || cy >= y1) continue;
        int col = (cx - x0) * SCALE;
        int row = (y1 - 1 - cy) * SCALE;

        int num    = static_cast<int>(i) + 1;
        bool twoD  = (num >= 10);
        int boxW   = twoD ? 9 : 7;   // 9px voor twee cijfers, 7px voor één

        int bx = col - boxW / 2;
        int by = row - 3;

        // Groene achtergrond
        for (int dy = 0; dy < 7; ++dy)
            for (int dx = 0; dx < boxW; ++dx) {
                int px = bx + dx, py = by + dy;
                if (px < 0 || px >= imgW || py < 0 || py >= imgH) continue;
                size_t idx = static_cast<size_t>(py * imgW + px) * 3;
                img[idx] = 30; img[idx+1] = 160; img[idx+2] = 30;
            }

        // Wit cijfer (of twee cijfers)
        if (!twoD) {
            DrawDigit(img, imgW, imgH, bx + 2, by + 1, num, 255, 255, 255);
        } else {
            DrawDigit(img, imgW, imgH, bx + 1, by + 1, num / 10, 255, 255, 255);
            DrawDigit(img, imgW, imgH, bx + 5, by + 1, num % 10, 255, 255, 255);
        }
    }

    // ── Stap 8: afmetingen van de kamer tekenen ──────────────────
    auto fill = [&](int rx, int ry, int rw, int rh,
                    uint8_t r, uint8_t g, uint8_t b) {
        for (int dy = 0; dy < rh; ++dy)
            for (int dx = 0; dx < rw; ++dx) {
                int px2 = rx + dx, py = ry + dy;
                if (px2 < 0 || px2 >= imgW || py < 0 || py >= totalH) continue;
                int idx = (py * imgW + px2) * 3;
                img[static_cast<size_t>(idx)]     = r;
                img[static_cast<size_t>(idx + 1)] = g;
                img[static_cast<size_t>(idx + 2)] = b;
            }
    };

    // Afmetingen naast de muren tekenen (technische-plattegrond-stijl).
    constexpr int TS = 2;                       // tekst-vergroting (font-pixel = TS×TS)
    const uint8_t TR = 20, TG = 20, TB = 160;   // tekstkleur (donkerblauw)

    // Eén vergroot cijfer tekenen via de fill-lambda.
    auto drawDigitBig = [&](int digit, int px, int py) {
        if (digit < 0 || digit > 9) return;
        const uint8_t* rows = kDigitFont[digit];
        for (int dy = 0; dy < 5; ++dy)
            for (int dx = 0; dx < 3; ++dx)
                if (rows[dy] & (1 << (2 - dx)))
                    fill(px + dx * TS, py + dy * TS, TS, TS, TR, TG, TB);
    };

    // Aantal cijfers in het gehele deel (voor centreren) + tekstbreedte in px.
    auto wholeDigits = [](float m) {
        int whole = static_cast<int>(m);
        if (whole == 0) return 1;
        int n = 0;
        for (int w = whole; w > 0; w /= 10) ++n;
        return n;
    };
    auto textW = [&](float m) { return (wholeDigits(m) * 4 + 2 + 8) * TS; };
    const int textH = 5 * TS;

    // Eén maat tekenen met witte achtergrond (leesbaar over muren/vrij gebied).
    auto drawMeters = [&](float meters, int px, int py) {
        fill(px - 2, py - 2, textW(meters) + 3, textH + 4, 255, 255, 255);  // wit kader
        int whole = static_cast<int>(meters);
        int frac  = static_cast<int>(meters * 100.0f + 0.5f) % 100;
        int digits[8], n = 0;
        if (whole == 0) digits[n++] = 0;
        else for (int v = whole; v > 0; v /= 10) digits[n++] = v % 10;
        for (int i = n - 1; i >= 0; --i) { drawDigitBig(digits[i], px, py); px += 4 * TS; }
        fill(px, py + 4 * TS, TS, TS, TR, TG, TB);          // decimaalpunt
        px += 2 * TS;
        drawDigitBig(frac / 10, px, py); px += 4 * TS;       // 1e decimaal
        drawDigitBig(frac % 10, px, py);                     // 2e decimaal
    };

    // Celcoördinaat (float) → pixel in de afbeelding (y wordt gespiegeld).
    auto colPx = [&](double cx) { return static_cast<int>((cx - x0) * SCALE); };
    auto rowPx = [&](double cy) { return static_cast<int>((y1 - 1 - cy) * SCALE); };

    // Pixel-omhullende van de rechthoek (voor de maatlijnen net buiten de muren).
    int pxL = imgW, pxR = 0, pyT = totalH, pyB = 0;
    for (int i = 0; i < 4; ++i) {
        int cxp = colPx(rectCellX[i]), cyp = rowPx(rectCellY[i]);
        pxL = std::min(pxL, cxp);  pxR = std::max(pxR, cxp);
        pyT = std::min(pyT, cyp);  pyB = std::max(pyB, cyp);
    }

    const uint8_t LR = 90, LG = 90, LB = 90;    // grijze maatlijnen

    // Breedte-maatlijn: net boven de kamer, met eindstreepjes en getal erboven.
    int yDim = std::max(1, pyT - 12);
    fill(pxL, yDim, pxR - pxL + 1, 1, LR, LG, LB);          // maatlijn
    fill(pxL, yDim - 4, 1, 9, LR, LG, LB);                  // eindstreep links
    fill(pxR, yDim - 4, 1, 9, LR, LG, LB);                  // eindstreep rechts
    drawMeters(realW_m, (pxL + pxR) / 2 - textW(realW_m) / 2,
               std::max(0, yDim - textH - 4));

    // Hoogte-maatlijn: net links van de kamer, met eindstreepjes en getal ernaast.
    int xDim = std::max(1, pxL - 12);
    fill(xDim, pyT, 1, pyB - pyT + 1, LR, LG, LB);          // maatlijn
    fill(xDim - 4, pyT, 9, 1, LR, LG, LB);                  // eindstreep boven
    fill(xDim - 4, pyB, 9, 1, LR, LG, LB);                  // eindstreep onder
    int hx = xDim - textW(realH_m) - 4;
    if (hx < 0) hx = xDim + 6;                              // te weinig marge links → rechts v/d lijn
    drawMeters(realH_m, hx, (pyT + pyB) / 2 - textH / 2);

    // ── Stap 8: schrijf PPM bestand ───────────────────────────────
    std::ofstream f2(filename, std::ios::binary);
    if (!f2) return false;

    f2 << "P6\n";
    f2 << "# breedte=" << realW_m << "m  hoogte=" << realH_m << "m\n";
    f2 << "# schaal: 1px=" << resolution * 100.0f << "cm  |  maten op kaart in meter\n";
    f2 << imgW << " " << totalH << "\n255\n";
    f2.write(reinterpret_cast<const char*>(img.data()),
             static_cast<std::streamsize>(img.size()));
    return true;
}