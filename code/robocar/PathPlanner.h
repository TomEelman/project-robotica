#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "GridMap.h"
#include "Path.h"
#include "Position.h"

// ─────────────────────────────────────────────────────────────────
//  PathPlanner — Wavefront (BFS) padplanner
//
//  In plaats van A* gebruiken we een Wavefront-aanpak: BFS die vanuit
//  het DOEL naar buiten "golft" en elke cel het aantal stappen tot het
//  doel geeft. Het pad volgt dan simpelweg de dalende getallen vanaf de
//  start. Voordelen t.o.v. A*:
//    - Geen heuristiek die rare paden kiest; altijd het kortste pad.
//    - Eenvoudiger te debuggen (de afstandskaart is letterlijk leesbaar).
//    - Vindt gegarandeerd een pad als er een bestaat — en als hij GEEN
//      pad vindt, weet je zeker dat de robot echt klem zit.
//
//  allowUnknown:
//    true  → robot mag door onbekend gebied plannen (nodig voor exploratie)
//    false → alleen door bevestigd vrije cellen (veilig terugrijden)
// ─────────────────────────────────────────────────────────────────
class PathPlanner {
private:
    GridMap gridMap;
    Path    currentPath;
    bool    allowUnknown;

public:
    explicit PathPlanner(const GridMap& map, bool allowUnknown = false);

    // Plant een pad van start naar goal. Beide in wereldcoordinaten (mm).
    // Returnt een lege Path als er geen pad is.
    Path PlanPath(Position start, Position goal, const GridMap& map);

    Path GetCurrentPath() const;

    // Update interne map-kopie (lidar heeft nieuwe info gegeven)
    void UpdateMap(const GridMap& map);

    // Zet of UNKNOWN-cellen betreedbaar zijn (exploratie aan/uit)
    void SetAllowUnknown(bool v) { allowUnknown = v; }

private:
    // Aantal cellen dat we rondom obstakels "verboden" maken zodat de
    // robot (chassis ~280mm) niet rakelings langs muren plant.
    // Bij resolutie 0.05m/cel = 5cm/cel → 2 cellen ≈ 10cm marge per kant.
    static constexpr int INFLATIE_CELLEN = 2;
};

#endif