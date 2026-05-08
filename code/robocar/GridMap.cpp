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
    // Robotpositie naar cellen
    int rx, ry;
    WorldToCell(robotX, robotY, rx, ry);

    constexpr float DEG2RAD = 3.14159265f / 180.0f;
    constexpr float MM2M    = 0.001f;

    for (int i = 0; i < count; ++i) {
        float dist = ranges[i];

        // Ongeldige of te verre metingen negeren
        if (dist <= 0.0f || dist > maxRange) continue;

        // Wereldhoek = robotrichting + LIDARhoek
        float globalAngle = robotTheta + angles[i] * DEG2RAD;

        // Eindpunt in meter (wereld)
        float wx = robotX + dist * MM2M * std::cos(globalAngle);
        float wy = robotY + dist * MM2M * std::sin(globalAngle);

        // Eindpunt naar cellen
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
    while (!path.IsEmpty()) {
        Position p = path.GetCurrentWaypoint();
        int cx, cy;
        WorldToCell(p.GetX(), p.GetY(), cx, cy);

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

void GridMap::Clear() {
    for (auto& row : logOdds)
        std::fill(row.begin(), row.end(), CELL_UNKNOWN);
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

bool GridMap::SavePGMCropped(const std::string& filename, float margin_m) const {
    // ── Stap 1: bounding box van bekende cellen ───────────────────
    int minX = width,  maxX = -1;
    int minY = height, maxY = -1;

    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (!IsUnknown(x, y)) {
                if (x < minX) minX = x;  if (x > maxX) maxX = x;
                if (y < minY) minY = y;  if (y > maxY) maxY = y;
            }

    if (maxX < 0) return SavePGM(filename);  // niets gescand

    // ── Stap 2: marge toevoegen ───────────────────────────────────
    int mg  = static_cast<int>(margin_m / resolution) + 1;
    int x0  = std::max(0,      minX - mg);
    int y0  = std::max(0,      minY - mg);
    int x1  = std::min(width,  maxX + mg + 1);
    int y1  = std::min(height, maxY + mg + 1);
    int cW  = x1 - x0;   // cel-breedte
    int cH  = y1 - y0;   // cel-hoogte

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

    // Kaartpixels (elk cel → SCALE×SCALE pixels)
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

    // ── Stap 6: schaalbalken tekenen ──────────────────────────────
    // Helper: kleur een blok pixels
    auto fill = [&](int rx, int ry, int rw, int rh, uint8_t r, uint8_t g, uint8_t b) {
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

    // Horizontale maatlijn (rood) — 1 meter
    int barY  = imgH + scaleBarH / 2;
    int barX0 = 20;
    int barX1 = barX0 + pixPerMeter;
    if (barX1 > imgW - 20) barX1 = imgW - 20;

    fill(barX0, barY - 1, barX1 - barX0, 3, 200, 40, 40);   // lijn
    fill(barX0 - 1, barY - 8, 3, 17, 200, 40, 40);           // eindstreep links
    fill(barX1 - 1, barY - 8, 3, 17, 200, 40, 40);           // eindstreep rechts

    // Pijlpunt links (driehoek)
    for (int i = 0; i < 8; ++i)
        fill(barX0 + i, barY - (4 - i/2), 1, (4 - i/2) * 2 + 1, 200, 40, 40);

    // Pijlpunt rechts
    for (int i = 0; i < 8; ++i)
        fill(barX1 - i - 1, barY - (4 - i/2), 1, (4 - i/2) * 2 + 1, 200, 40, 40);

    // Label blokje "1m" naast de balk (simpel wit kader met rode fill)
    fill(barX1 + 8, barY - 6, 30, 14, 200, 40, 40);   // rood blokje als label
    fill(barX1 + 9, barY - 5, 28, 12, 235, 235, 235); // wit binnenin

    // ── Stap 7: schrijf PPM bestand ───────────────────────────────
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
nclude "GridMap.h"
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
    // Robotpositie naar cellen
    int rx, ry;
    WorldToCell(robotX, robotY, rx, ry);

    constexpr float DEG2RAD = 3.14159265f / 180.0f;
    constexpr float MM2M    = 0.001f;

    for (int i = 0; i < count; ++i) {
        float dist = ranges[i];

        // Ongeldige of te verre metingen negeren
        if (dist <= 0.0f || dist > maxRange) continue;

        // Wereldhoek = robotrichting + LIDARhoek
        float globalAngle = robotTheta + angles[i] * DEG2RAD;

        // Eindpunt in meter (wereld)
        float wx = robotX + dist * MM2M * std::cos(globalAngle);
        float wy = robotY + dist * MM2M * std::sin(globalAngle);

        // Eindpunt naar cellen
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
    while (!path.IsEmpty()) {
        Position p = path.GetCurrentWaypoint();
        int cx, cy;
        WorldToCell(p.GetX(), p.GetY(), cx, cy);

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

void GridMap::Clear() {
    for (auto& row : logOdds)
        std::fill(row.begin(), row.end(), CELL_UNKNOWN);
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

bool GridMap::SavePGMCropped(const std::string& filename, float margin_m) const {
    // ── Stap 1: bounding box van bekende cellen ───────────────────
    int minX = width,  maxX = -1;
    int minY = height, maxY = -1;

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

    if (maxX < 0) {
        // Niets gescand — schrijf lege kaart
        return SavePGM(filename);
    }

    // ── Stap 2: marge toevoegen ───────────────────────────────────
    int margin_cells = static_cast<int>(margin_m / resolution) + 1;
    int x0 = std::max(0,       minX - margin_cells);
    int y0 = std::max(0,       minY - margin_cells);
    int x1 = std::min(width,   maxX + margin_cells + 1);
    int y1 = std::min(height,  maxY + margin_cells + 1);

    int cropW = x1 - x0;
    int cropH = y1 - y0;

    // ── Upscaling: elke cel wordt SCALE×SCALE pixels ──────────────
    // Dit geeft scherpere lijnen zonder anti-aliasing waas
    constexpr int SCALE = 3;
    int outW = cropW * SCALE;
    int outH_map = cropH * SCALE;

    // ── Afmetingen in meters berekenen ────────────────────────────
    float realW_m = static_cast<float>(cropW) * resolution;
    float realH_m = static_cast<float>(cropH) * resolution;

    // ── Stap 3: schaalbalken ──────────────────────────────────────
    int scaleBarH   = 60;   // groter voor betere leesbaarheid
    int pixPerMeter = static_cast<int>(1.0f / resolution) * SCALE;
    int totalH      = outH_map + scaleBarH;
    // Alias voor compatibiliteit met rest van de code
    int& cropWScaled = outW;

    // Schrijf als PPM (P6, kleur) zodat schaalbalken rood zijn
    std::ofstream f(filename, std::ios::binary);
    if (!f) return false;

    f << "P6\n";
    f << "# Kaart  breedte=" << realW_m << "m  hoogte=" << realH_m << "m\n";
    f << "# Schaal: 1 pixel = " << resolution * 100.0f << " cm\n";
    f << cropW << " " << totalH << "\n255\n";

    // ── Kaartpixels ───────────────────────────────────────────────
    for (int y = y1 - 1; y >= y0; --y) {   // PGM: Y van boven naar onder
        for (int x = x0; x < x1; ++x) {
            uint8_t px;
            if      (!InBounds(x, y))          px = 200;  // buiten grid = lichtgrijs
            else if (logOdds[y][x] >= CELL_OCCUPIED) px = 0;    // muur = zwart
            else if (logOdds[y][x] <= CELL_FREE)     px = 255;  // vrij = wit
            else                                      px = 160;  // onbekend = grijs
            // RGB: grijs tint
            f.put(static_cast<char>(px));
            f.put(static_cast<char>(px));
            f.put(static_cast<char>(px));
        }
    }

    // ── Schaalblok: witte achtergrond ─────────────────────────────
    for (int sy = 0; sy < scaleBarH; ++sy) {
        for (int sx = 0; sx < cropW; ++sx) {
            f.put(static_cast<char>(240));  // lichtgrijs
            f.put(static_cast<char>(240));
            f.put(static_cast<char>(240));
        }
    }

    // ── Schaalbalken opnieuw schrijven als rode lijn ──────────────
    // We gaan terug naar het begin van het schaalblok en overschrijven
    // Simpelere aanpak: bouw schaalblok als pixel-buffer

    // Heropen als we kunnen, maar we gebruiken een buffer
    f.close();

    // Herbouw het schaalblok als buffer (upgeschaald met SCALE)
    int totalPixels = outW * totalH * 3;
    std::vector<uint8_t> img(static_cast<size_t>(totalPixels), 240);

    // Kaartpixels in buffer — elke cel wordt SCALE×SCALE pixels
    for (int y = y1 - 1; y >= y0; --y) {
        int cellRow = (y1 - 1 - y);
        for (int x = x0; x < x1; ++x) {
            int cellCol = x - x0;
            uint8_t px;
            if      (!InBounds(x, y))                    px = 200;
            else if (logOdds[y][x] >= CELL_OCCUPIED)     px = 0;
            else if (logOdds[y][x] <= CELL_FREE)         px = 255;
            else                                          px = 160;

            // Vul SCALE×SCALE pixels voor deze cel
            for (int sy = 0; sy < SCALE; ++sy) {
                for (int sx = 0; sx < SCALE; ++sx) {
                    int row = cellRow * SCALE + sy;
                    int col = cellCol * SCALE + sx;
                    int idx = (row * outW + col) * 3;
                    img[static_cast<size_t>(idx)]     = px;
                    img[static_cast<size_t>(idx + 1)] = px;
                    img[static_cast<size_t>(idx + 2)] = px;
                }
            }
        }
    }

    // Schaalblok: witte achtergrond al ingevuld (240)
    // Teken rode horizontale balk voor breedte (1 meter)
    {
        int barRow  = cropH + scaleBarH / 2;   // midden van schaalblok
        int barStartX = 10;
        int barEndX   = barStartX + pixPerMeter;
        if (barEndX > cropW) barEndX = cropW - 10;

        // Rode balk (3 pixels dik)
        for (int thick = -1; thick <= 1; ++thick) {
            int r = barRow + thick;
            if (r < 0 || r >= totalH) continue;
            for (int sx = barStartX; sx < barEndX; ++sx) {
                int idx = (r * cropW + sx) * 3;
                img[static_cast<size_t>(idx)]     = 220;  // R
                img[static_cast<size_t>(idx + 1)] = 50;   // G
                img[static_cast<size_t>(idx + 2)] = 50;   // B
            }
        }
        // Eindstreepjes
        for (int thick = -5; thick <= 5; ++thick) {
            auto markCol = [&](int col) {
                if (col < 0 || col >= cropW) return;
                int idx = (barRow * cropW + col) * 3;
                img[static_cast<size_t>(idx)]     = 220;
                img[static_cast<size_t>(idx + 1)] = 50;
                img[static_cast<size_t>(idx + 2)] = 50;
            };
            markCol(barStartX + thick);
            markCol(barEndX   + thick);
        }
    }

    // Teken rode verticale balk voor hoogte (1 meter)
    {
        int barColCenter = cropW - 20;
        int barStartY    = cropH + 5;
        int barEndY      = barStartY + pixPerMeter;
        if (barEndY > totalH) barEndY = totalH - 2;

        for (int thick = -1; thick <= 1; ++thick) {
            int c = barColCenter + thick;
            if (c < 0 || c >= cropW) continue;
            for (int sy = barStartY; sy < barEndY; ++sy) {
                int idx = (sy * cropW + c) * 3;
                img[static_cast<size_t>(idx)]     = 50;
                img[static_cast<size_t>(idx + 1)] = 100;
                img[static_cast<size_t>(idx + 2)] = 220;  // blauw voor hoogte
            }
        }
    }

    // ── Schaalblok: tekst simuleren met pixel-blokjes ────────────
    // Teken maatlijnen met pijlpunten en labels als pixel-blokjes

    // Helper: teken een gevuld rechthoekje (voor pixel-letters)
    auto drawRect = [&](int rx0, int ry0, int rw, int rh,
                        uint8_t r, uint8_t g, uint8_t b) {
        for (int ry = ry0; ry < ry0 + rh && ry < totalH; ++ry) {
            for (int rx = rx0; rx < rx0 + rw && rx < outW; ++rx) {
                if (rx < 0 || ry < 0) continue;
                int idx = (ry * cropW + rx) * 3;
                img[static_cast<size_t>(idx)]     = r;
                img[static_cast<size_t>(idx + 1)] = g;
                img[static_cast<size_t>(idx + 2)] = b;
            }
        }
    };

    // Maatlijn horizontaal: pijl van links naar rechts over 1 meter
    int mBarY   = outH_map + scaleBarH / 2;
    int mStartX = 15;
    int mEndX   = mStartX + pixPerMeter;
    if (mEndX > outW - 5) mEndX = outW - 5;

    // Lijn
    drawRect(mStartX, mBarY - 1, mEndX - mStartX, 3, 180, 30, 30);
    // Pijlpunt links
    for (int i = 0; i < 6; ++i)
        drawRect(mStartX + i, mBarY - (5 - i), 2, (5 - i) * 2 + 1, 180, 30, 30);
    // Pijlpunt rechts
    for (int i = 0; i < 6; ++i)
        drawRect(mEndX - i - 2, mBarY - (5 - i), 2, (5 - i) * 2 + 1, 180, 30, 30);
    // Eindstreepjes
    drawRect(mStartX, mBarY - 6, 2, 13, 180, 30, 30);
    drawRect(mEndX - 2, mBarY - 6, 2, 13, 180, 30, 30);

    // Label "1 m" als dikke blokjes naast de balk
    // Simpele pixel-font: elke letter 5x7 blokjes van 2x2 px
    int lx = mEndX + 8;
    int ly = mBarY - 5;
    // "1"
    drawRect(lx+2, ly,   2, 2, 80,80,80);
    drawRect(lx,   ly+2, 2, 2, 80,80,80);
    drawRect(lx+2, ly+2, 2, 7*2, 80,80,80);
    drawRect(lx,   ly+14,6, 2, 80,80,80);
    lx += 14;
    // "m"
    drawRect(lx,   ly,   2, 14, 80,80,80);
    drawRect(lx+2, ly+2, 2, 2,  80,80,80);
    drawRect(lx+4, ly,   2, 14, 80,80,80);
    drawRect(lx+6, ly+2, 2, 2,  80,80,80);
    drawRect(lx+8, ly,   2, 14, 80,80,80);

    // Breedte-label rechtsboven in schaalblok: bv "6.2 m"
    // Schrijf dimensie als kleine blokjes
    // (vereenvoudigd: teken een rechthoek als placeholder voor de waarde)
    // In plaats van volledige font: schrijf dimensies in PGM comment

    // Schrijf buffer naar bestand
    std::ofstream f2(filename, std::ios::binary);
    if (!f2) return false;

    f2 << "P6\n";
    f2 << "# Kaart  breedte=" << realW_m << "m  hoogte=" << realH_m << "m\n";
    f2 << "# Rode balk = 1m  |  1 pixel = " << resolution * 100.0f << " cm\n";
    f2 << outW << " " << totalH << "\n255\n";
    f2.write(reinterpret_cast<const char*>(img.data()),
             static_cast<std::streamsize>(img.size()));

    return true;
}