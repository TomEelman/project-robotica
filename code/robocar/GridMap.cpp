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

    // Loop over de vrije cellen (alles behalve de laatste).
    // Zodra we een cel tegenkomen die al stevig bezet is (>= CELL_OCCUPIED),
    // stoppen we: er kan nooit een muur ACHTER een bestaande muur geplaatst
    // worden. De straal eindigt hier — het eindpunt wordt genegeerd.
    // Dit voorkomt ghost-muren bij slecht gematche scans.
    for (size_t i = 0; i + 1 < cells.size(); ++i) {
        auto [cx, cy] = cells[i];
        if (!InBounds(cx, cy)) continue;

        int8_t& cell = logOdds[cy][cx];

        if (cell >= CELL_OCCUPIED) {
            // Bestaande muur in de weg: straal afkappen, eindpunt negeren.
            binaryDirty = true;
            return;
        }

        cell = static_cast<int8_t>(cell + L_FREE);
        ClampLogOdds(cell);
    }

    // Eindcel: bezet markeren — alleen bereikbaar als de straal niet geblokkeerd werd.
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
        float thetaCorrected = robotThetaDeg + omegaDegS * (t_i - halfDuur);

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
//  SavePGMCropped  –  bijgesneden kaart met schaalbalken
//
//  Werking:
//    1. Zoek de bounding box van alle bekende cellen (vrij + bezet)
//    2. Voeg een marge toe rondom dit gebied
//    3. Schrijf een PPM (kleur) zodat de schaalbalken rood kunnen zijn
//    4. Schaalbalken: horizontaal (breedte) + vertikaal (hoogte) in meter
//       getekend als rode lijn met witte achtergrond onder de kaart
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
//  SavePGMCropped  –  bijgesneden kaart met schaalbalken
//  Snijdt automatisch bij op het verkende gebied + marge.
//  Elke cel wordt SCALE×SCALE pixels (scherper beeld).
//  Onder de kaart: rode maatlijn van 1 meter.
//  Schrijft als PPM (kleur) zodat de balk rood kan zijn.
// ═══════════════════════════════════════════════════════════════════

bool GridMap::SavePGMCropped(const std::string& filename, float margin_m) const {
    // ── Stap 1: bounding box van bekende cellen ───────────────────
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

    float realW_m = static_cast<float>(cW) * resolution;
    float realH_m = static_cast<float>(cH) * resolution;

    // ── Stap 4: schaalblok onderaan ───────────────────────────────
    int scaleBarH   = 60;
    int pixPerMeter = static_cast<int>(1.0f / resolution) * SCALE;
    int totalH      = imgH + scaleBarH;

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

    // ── Stap 8: schaalbalken tekenen ─────────────────────────────
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

    // Horizontale maatlijn (rood) – 1 meter
    int barY  = imgH + scaleBarH / 2;
    int barX0 = 20;
    int barX1 = barX0 + pixPerMeter;
    if (barX1 > imgW - 20) barX1 = imgW - 20;

    fill(barX0, barY - 1, barX1 - barX0, 3, 200, 40, 40);  // lijn
    fill(barX0 - 1, barY - 8, 3, 17, 200, 40, 40);          // eindstreep links
    fill(barX1 - 1, barY - 8, 3, 17, 200, 40, 40);          // eindstreep rechts

    // Pijlpuntenf
    for (int i = 0; i < 8; ++i) {
        int half = (4 - i / 2);
        fill(barX0 + i, barY - half, 1, half * 2 + 1, 200, 40, 40);
        fill(barX1 - i - 1, barY - half, 1, half * 2 + 1, 200, 40, 40);
    }

    // Label kader "1m"
    fill(barX1 + 8,  barY - 6, 30, 14, 200, 40, 40);   // rood kader
    fill(barX1 + 9,  barY - 5, 28, 12, 235, 235, 235); // wit binnenin

    // ── Stap 8: schrijf PPM bestand ───────────────────────────────
    std::ofstream f2(filename, std::ios::binary);
    if (!f2) return false;

    f2 << "P6\n";
    f2 << "# breedte=" << realW_m << "m  hoogte=" << realH_m << "m\n";
    f2 << "# schaal: 1px=" << resolution * 100.0f << "cm  |  rode balk=1m\n";
    f2 << imgW << " " << totalH << "\n255\n";
    f2.write(reinterpret_cast<const char*>(img.data()),
             static_cast<std::streamsize>(img.size()));
    return true;
}